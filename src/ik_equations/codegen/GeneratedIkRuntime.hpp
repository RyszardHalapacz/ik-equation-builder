#pragma once

#include "ik_equations/solver/detail/AnalyticIkNumerics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <numeric>
#include <vector>

namespace kinemaforge::ik::generated {

struct Vec3
{
    double x{};
    double y{};
    double z{};
};

using Mat3 = std::array<std::array<double, 3>, 3>;

struct Pose
{
    Vec3 position;
    Mat3 orientation{};
};

struct Axis
{
    Vec3 point;
    Vec3 direction;
};

struct JointLimit
{
    bool enabled{};
    double lower{};
    double upper{};
};

// Everything robot-specific the generated solver needs.  No URDF, no strings,
// no expression DAG and no heap-owned model survive code generation.
struct Parameters
{
    Axis axes[6]{};
    Vec3 wristCenterHome;
    Vec3 wristInTargetFrame;
    Mat3 targetFrameHomeRotation{};

    Vec3 planarX;
    Vec3 planarY;
    double link1{};
    double link2{};
    double link2HomeAngle{};
    double axis3Sign{1.0};

    Mat3 wristBasis{};
    double axis6Sign{1.0};

    JointLimit limits[6]{};
};

enum class Status
{
    Solutions,
    NoSolution,
    InfiniteFamily,
    InvalidTarget,
    // The result would hold more branches than the caller's budget. Nothing is
    // returned; Result::requiredBranches says how many were needed.
    ResourceLimitExceeded,
    // The target lies on a singular family this solver does not parameterise
    // (the wrist centre on the joint-1 axis, where q1 is free, or -- with equal
    // arm links -- on the joint-2 axis, where q2 is free). Solutions may
    // well exist; this is NOT NoSolution. Mirrors the general solver's
    // UnsupportedKinematicStructure for the same case.
    UnsupportedSingularity
};

struct Branch
{
    std::array<double, 6> q{};
    bool singular{};
    // Valid only for singular wrist branches. q[3] and q[5] are NaN there;
    // q6 = dependentOffset + dependentMultiplier*q4, modulo 2*pi.
    double freeLower{};
    double freeUpper{};
    double dependentMultiplier{};
    double dependentOffset{};
};

struct Result
{
    Status status{Status::NoSolution};
    std::vector<Branch> branches;   // empty for ResourceLimitExceeded
    // Branches the full result needs (upper estimate, saturated at SIZE_MAX).
    std::size_t requiredBranches{};
    // Branches reproduce the rotation NEAREST to the target orientation; this
    // is its largest element-wise distance from the orientation passed in.
    double orientationCorrection{};
};

namespace detail {

inline constexpr double kLinearTolerance = 1e-9;
inline constexpr double kAngularTolerance = 1e-10;
inline constexpr double kOrientationTolerance = 1e-8;
inline constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;

inline Vec3 add(Vec3 a, Vec3 b) noexcept { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline Vec3 sub(Vec3 a, Vec3 b) noexcept { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline Vec3 scale(Vec3 v,double s) noexcept { return {v.x*s,v.y*s,v.z*s}; }
inline double dot(Vec3 a,Vec3 b) noexcept { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline Vec3 cross(Vec3 a,Vec3 b) noexcept { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
inline double norm(Vec3 v) noexcept { return std::hypot(v.x,v.y,v.z); }
inline Vec3 projectPerpendicular(Vec3 v,Vec3 a) noexcept { return sub(v,scale(a,dot(v,a))); }

inline Mat3 transpose(const Mat3& m) noexcept
{
    Mat3 r{};
    for(std::size_t i=0;i<3;++i) for(std::size_t j=0;j<3;++j) r[i][j]=m[j][i];
    return r;
}

inline Mat3 multiply(const Mat3& a,const Mat3& b) noexcept
{
    Mat3 r{};
    for(std::size_t i=0;i<3;++i) for(std::size_t j=0;j<3;++j)
        for(std::size_t k=0;k<3;++k) r[i][j]+=a[i][k]*b[k][j];
    return r;
}

inline Vec3 multiply(const Mat3& a,Vec3 v) noexcept
{
    return {a[0][0]*v.x+a[0][1]*v.y+a[0][2]*v.z,
            a[1][0]*v.x+a[1][1]*v.y+a[1][2]*v.z,
            a[2][0]*v.x+a[2][1]*v.y+a[2][2]*v.z};
}

inline Mat3 rotation(Vec3 a,double q) noexcept
{
    const double c=std::cos(q),s=std::sin(q),t=1.0-c;
    const double x=a.x,y=a.y,z=a.z;
    return {{{c+x*x*t,x*y*t-z*s,x*z*t+y*s},
             {y*x*t+z*s,c+y*y*t,y*z*t-x*s},
             {z*x*t-y*s,z*y*t+x*s,c+z*z*t}}};
}

inline Vec3 rotatePoint(Vec3 p,const Axis& a,double q) noexcept
{
    return add(a.point,multiply(rotation(a.direction,q),sub(p,a.point)));
}

inline double normalizeAngle(double q) noexcept
{
    double r=std::remainder(q,kTwoPi);
    if(r>=std::numbers::pi_v<double>) r-=kTwoPi;
    return r;
}

inline bool validTarget(const Pose& target) noexcept
{
    if(!std::isfinite(target.position.x)||!std::isfinite(target.position.y)||!std::isfinite(target.position.z)) return false;
    for(const auto& row:target.orientation) for(double v:row) if(!std::isfinite(v)) return false;
    for(std::size_t i=0;i<3;++i) for(std::size_t j=0;j<3;++j)
    {
        double p=0.0; for(std::size_t k=0;k<3;++k) p+=target.orientation[k][i]*target.orientation[k][j];
        if(std::abs(p-(i==j?1.0:0.0))>kOrientationTolerance) return false;
    }
    const auto& m=target.orientation;
    const double d=m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-
                   m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+
                   m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
    return std::abs(d-1.0)<=kOrientationTolerance;
}

inline void distinctAngle(std::vector<double>& out,double q,double tol)
{
    q=normalizeAngle(q);
    for(double e:out) if(std::abs(normalizeAngle(e-q))<=tol) return;
    out.push_back(q);
}

inline std::vector<double> solveSinCos(double a,double b,double c,double tol)
{
    const double r=std::hypot(a,b);
    if(r<=tol) return {};
    double s=c/r;
    if(s < -1.0-tol || s > 1.0+tol) return {};
    s=std::clamp(s,-1.0,1.0);
    const double phase=std::atan2(b,a), first=std::asin(s);
    std::vector<double> out;
    distinctAngle(out,first-phase,tol);
    distinctAngle(out,std::numbers::pi_v<double>-first-phase,tol);
    return out;
}

struct PlanarBranch { double q2{},q3{}; };
inline std::vector<PlanarBranch> solvePlanar(double x,double y,const Parameters& p)
{
    const double rr=x*x+y*y;
    const double c=(rr-p.link1*p.link1-p.link2*p.link2)/(2.0*p.link1*p.link2);
    if(c < -1.0-kLinearTolerance || c > 1.0+kLinearTolerance) return {};
    const double m=std::acos(std::clamp(c,-1.0,1.0));
    const double target=std::atan2(y,x);
    std::vector<PlanarBranch> out;
    for(double internal:{m,-m})
    {
        double q3=(internal-p.link2HomeAngle)/p.axis3Sign;
        // An ill-conditioned elbow angle just outside its limit is put on the
        // limit and the shoulder angle re-solved for it; the FK check decides
        // (see kinemaforge::ik::detail::kElbowLimitSnapMargin).
        if(p.limits[2].enabled)
            if(const auto snapped=kinemaforge::ik::detail::nearbyLimit(q3,p.limits[2].lower,p.limits[2].upper,
                                                                       kinemaforge::ik::detail::kElbowLimitSnapMargin))
            {
                q3=*snapped; internal=p.link2HomeAngle+p.axis3Sign*q3;
            }
        const double vx=p.link1+p.link2*std::cos(internal);
        const double vy=p.link2*std::sin(internal);
        const double q2=normalizeAngle(target-std::atan2(vy,vx));
        bool dup=false; for(const auto& e:out) dup|=std::abs(normalizeAngle(e.q2-q2))<kLinearTolerance&&std::abs(normalizeAngle(e.q3-q3))<kLinearTolerance;
        if(!dup) out.push_back({q2,normalizeAngle(q3)});
    }
    return out;
}

inline bool withinLimit(const JointLimit& l,double v) noexcept
{
    return std::isfinite(v)&&(!l.enabled||kinemaforge::ik::detail::withinLimit(v,l.lower,l.upper));
}

// Number of values equivalents() would return, without building them.
inline std::size_t equivalentCount(const JointLimit& l,double q) noexcept
{
    if(!std::isfinite(q))return 0U;
    if(!l.enabled)return 1U;
    return kinemaforge::ik::detail::turnCount(kinemaforge::ik::detail::turnsWithin(l.lower,l.upper,normalizeAngle(q)));
}

// Precondition for bounded joints: equivalentCount() was checked against the
// branch budget, so the vector built here is small.
inline std::vector<double> equivalents(const JointLimit& l,double q)
{
    q=normalizeAngle(q);
    if(!l.enabled) return {q};
    const auto turns=kinemaforge::ik::detail::turnsWithin(l.lower,l.upper,q);
    std::vector<double> out;
    for(auto k=turns.first;k<=turns.last;++k)
    {
        const double v=q+kTwoPi*static_cast<double>(k);
        // Within the numerical margin of a limit the value IS the limit; snap
        // it so no returned number lies outside [lower, upper].
        if(withinLimit(l,v)) out.push_back(kinemaforge::ik::detail::snapToLimit(v,l.lower,l.upper));
    }
    return out;
}

// One geometric branch before its in-limit full turns are expanded.
struct Candidate
{
    std::array<double,6> q{};      // q[3], q[5] unused when singular
    bool singular{};
    double multiplier{};
    double offset{};
    std::size_t count{};           // branches this one expands to (upper estimate)
};

inline kinemaforge::ik::detail::TurnRange dependentTurns(const Parameters& p,double multiplier,double offset) noexcept
{
    if(!p.limits[5].enabled)return {0,0}; // an unbounded dependent axis needs one modulo relation
    const double fl=p.limits[3].enabled?p.limits[3].lower:-std::numbers::pi_v<double>;
    const double fu=p.limits[3].enabled?p.limits[3].upper: std::numbers::pi_v<double>;
    const double a=offset+multiplier*fl,b=offset+multiplier*fu;
    return kinemaforge::ik::detail::turnsWithin(p.limits[5].lower-std::max(a,b),p.limits[5].upper-std::min(a,b));
}

inline std::size_t branchCount(const Parameters& p,const Candidate& c) noexcept
{
    using kinemaforge::ik::detail::saturatingMultiply;
    std::size_t n=1U;
    for(std::size_t i=0;i<6;++i)
    {
        if(c.singular&&(i==3||i==5))continue;
        n=saturatingMultiply(n,equivalentCount(p.limits[i],c.q[i]));
    }
    if(c.singular)n=saturatingMultiply(n,kinemaforge::ik::detail::turnCount(dependentTurns(p,c.multiplier,c.offset)));
    return n;
}

inline void appendRegular(Result& result,const Parameters& p,const std::array<double,6>& q)
{
    std::array<std::vector<double>,6> a;
    for(std::size_t i=0;i<6;++i){a[i]=equivalents(p.limits[i],q[i]);if(a[i].empty())return;}
    std::array<std::size_t,6> c{};
    while(true)
    {
        Branch b; for(std::size_t i=0;i<6;++i)b.q[i]=a[i][c[i]]; result.branches.push_back(b);
        std::size_t i=6; while(i>0){--i;if(++c[i]<a[i].size())break;c[i]=0;} if(i==0&&c[0]==0)break;
    }
}

inline void appendSingular(Result& result,const Parameters& p,const Candidate& candidate)
{
    const double multiplier=candidate.multiplier,offset=candidate.offset;
    auto q1=equivalents(p.limits[0],candidate.q[0]),q2=equivalents(p.limits[1],candidate.q[1]),
         q3=equivalents(p.limits[2],candidate.q[2]),q5s=equivalents(p.limits[4],candidate.q[4]);
    if(q1.empty()||q2.empty()||q3.empty()||q5s.empty())return;
    const double fl=p.limits[3].enabled?p.limits[3].lower:-std::numbers::pi_v<double>;
    const double fu=p.limits[3].enabled?p.limits[3].upper: std::numbers::pi_v<double>;
    const auto turns=dependentTurns(p,multiplier,offset);
    for(double a:q1)for(double b:q2)for(double c:q3)for(double e:q5s)
    for(auto k=turns.first;k<=turns.last;++k)
    {
        // Shared with the general solver: free bounds and the dependent values
        // at those bounds lie within the limits exactly (audit 2026-11-16 #7).
        const auto interval=kinemaforge::ik::detail::coupledFreeInterval(
            fl,fu,p.limits[5].enabled,p.limits[5].lower,p.limits[5].upper,
            offset+kTwoPi*static_cast<double>(k),multiplier,1e-12);
        if(!interval)continue;
        const double off=interval->offset,lo=interval->lower,hi=interval->upper;
        Branch br; br.q={a,b,c,std::numeric_limits<double>::quiet_NaN(),e,std::numeric_limits<double>::quiet_NaN()};
        br.singular=true;br.freeLower=lo;br.freeUpper=hi;br.dependentMultiplier=multiplier;br.dependentOffset=off;result.branches.push_back(br);
    }
}

// Forward kinematics from the generated numeric parameters alone (space-axis
// product applied to the home target frame, including the TCP offset).
inline bool forwardMatches(const Parameters& p,const std::array<double,6>& q,const Pose& target)
{
    for(double v:q)if(!std::isfinite(v))return false;
    Mat3 orientation=p.targetFrameHomeRotation;
    Vec3 position=sub(p.wristCenterHome,multiply(orientation,p.wristInTargetFrame));
    for(std::size_t i=q.size();i-->0;)
    {
        const Mat3 r=rotation(p.axes[i].direction,q[i]);
        position=add(p.axes[i].point,multiply(r,sub(position,p.axes[i].point)));
        orientation=multiply(r,orientation);
    }
    const double distance=norm(sub(position,target.position));
    if(!std::isfinite(distance)||distance>2e-9)return false;
    for(std::size_t i=0;i<3;++i)for(std::size_t j=0;j<3;++j)
        if(!std::isfinite(orientation[i][j])||std::abs(orientation[i][j]-target.orientation[i][j])>2e-9)return false;
    return true;
}

inline bool poseMatches(const Parameters& p,const std::array<double,6>& q,const Pose& target)
{
    for(std::size_t i=0;i<q.size();++i)
        if(!withinLimit(p.limits[i],q[i]))return false;
    return forwardMatches(p,q,target);
}

inline bool branchMatches(const Parameters& p,const Branch& b,const Pose& target)
{
    if(!b.singular)return poseMatches(p,b.q,target);
    auto q=b.q;
    for(double free:{b.freeLower,std::midpoint(b.freeLower,b.freeUpper),b.freeUpper})
    {
        q[3]=free;
        q[5]=b.dependentOffset+b.dependentMultiplier*free;
        if(!poseMatches(p,q,target))return false;
    }
    return true;
}

inline bool sameGeometricBranch(const std::array<double,6>& a,const std::array<double,6>& b) noexcept
{
    for(std::size_t i=0;i<6;++i)if(std::abs(normalizeAngle(a[i]-b[i]))>1e-9)return false;
    return true;
}

} // namespace detail

// maxBranches bounds the number of branches (regular solutions and singular
// families) in one result. Every in-limit full turn of every joint is a
// separate branch, so the count is a product over joints; it is computed from
// the limits before anything is allocated. A result that would exceed the
// bound is reported as ResourceLimitExceeded and never truncated.
inline Result solve(const Parameters& p,const Pose& requestedTarget,std::size_t maxBranches=4096)
{
    using namespace detail;
    Result result;
    if(!validTarget(requestedTarget)){result.status=Status::InvalidTarget;return result;}

    // Angles reproduce exact rotations only. Solve for the rotation nearest to
    // the accepted input and report the distance.
    const auto projected=kinemaforge::ik::detail::nearestRotation(requestedTarget.orientation);
    if(!projected.converged||!(projected.correction<=kinemaforge::ik::detail::kMaxOrientationCorrection))
    {
        result.status=Status::InvalidTarget;return result;
    }
    const Pose target{requestedTarget.position,projected.rotation};
    result.orientationCorrection=projected.correction;

    const Vec3 targetWrist=add(target.position,multiply(target.orientation,p.wristInTargetFrame));
    const Axis& a1=p.axes[0]; const Vec3 d2=p.axes[1].direction;
    const Vec3 v=sub(targetWrist,a1.point);
    const double d1d2=dot(a1.direction,d2),d1v=dot(a1.direction,v);
    const double cc=dot(v,d2)-d1d2*d1v;
    const double sc=-dot(cross(a1.direction,v),d2);
    const double k=dot(a1.point,d2)+d1d2*d1v-dot(p.wristCenterHome,d2);
    // Wrist centre on the joint-1 axis: the equation for q1 vanishes
    // identically and q1 is free. Same test, same tolerance as the general
    // solver's ScalarSolutionKind::Infinite, which it reports as unsupported.
    if(std::hypot(sc,cc)<=kLinearTolerance&&std::abs(k)<=kLinearTolerance)
    {
        result.status=Status::UnsupportedSingularity;return result;
    }
    const auto q1s=solveSinCos(sc,cc,-k,kLinearTolerance);
    if(q1s.empty()){result.status=Status::NoSolution;return result;}

    // Phase A: geometric branches, verified and deduplicated before expansion.
    std::vector<Candidate> candidates;
    const auto addRegular=[&](const std::array<double,6>& q)
    {
        if(!forwardMatches(p,q,target))return;
        for(const auto& existing:candidates)
            if(!existing.singular&&sameGeometricBranch(existing.q,q))return;
        Candidate c; c.q=q; c.count=branchCount(p,c); candidates.push_back(c);
    };
    const auto addSingular=[&](double q1,double q2,double q3,double q5,double multiplier,double offset)
    {
        Candidate c; c.q={q1,q2,q3,0.0,q5,0.0}; c.singular=true; c.multiplier=multiplier; c.offset=offset;
        c.count=branchCount(p,c); candidates.push_back(c);
    };

    for(double q1:q1s)
    {
        const Vec3 unrot=rotatePoint(targetWrist,a1,-q1);
        const Vec3 pt=projectPerpendicular(sub(unrot,p.axes[1].point),d2);
        const double tx=dot(pt,p.planarX),ty=dot(pt,p.planarY);
        // Equal arm links with the wrist centre on the joint-2 axis (elbow
        // fully folded): q2 is free. Same test as the general solver, which
        // reports it as unsupported -- never one arbitrary q2.
        if(std::hypot(tx,ty)<=kLinearTolerance&&std::abs(p.link1-p.link2)<=kLinearTolerance)
        {
            result.status=Status::UnsupportedSingularity;return result;
        }
        for(const auto& arm:solvePlanar(tx,ty,p))
        {
            const Mat3 r123=multiply(multiply(rotation(p.axes[0].direction,q1),rotation(p.axes[1].direction,arm.q2)),rotation(p.axes[2].direction,arm.q3));
            const Mat3 wr=multiply(multiply(transpose(r123),target.orientation),transpose(p.targetFrameHomeRotation));
            const Mat3 local=multiply(multiply(transpose(p.wristBasis),wr),p.wristBasis);
            const auto angles=kinemaforge::ik::detail::decomposeXyx(local,kAngularTolerance);
            if(!angles.singular)
            {
                const double alpha=angles.alpha,beta=angles.beta,gamma=angles.gamma;
                addRegular({q1,arm.q2,arm.q3,normalizeAngle(alpha),normalizeAngle(beta),normalizeAngle(gamma/p.axis6Sign)});
                addRegular({q1,arm.q2,arm.q3,normalizeAngle(alpha+std::numbers::pi_v<double>),normalizeAngle(-beta),normalizeAngle((gamma+std::numbers::pi_v<double>)/p.axis6Sign)});
            }
            else if(local[0][0]>=0.0)
            {
                addSingular(q1,arm.q2,arm.q3,0.0,-1.0/p.axis6Sign,angles.gamma/p.axis6Sign);
            }
            else
            {
                addSingular(q1,arm.q2,arm.q3,-std::numbers::pi_v<double>,1.0/p.axis6Sign,angles.gamma/p.axis6Sign);
            }
        }
    }

    // Phase B: count before allocating.
    for(const auto& c:candidates)
        result.requiredBranches=kinemaforge::ik::detail::saturatingAdd(result.requiredBranches,c.count);
    if(kinemaforge::ik::detail::branchBudgetExceeded(result.requiredBranches,maxBranches)){result.status=Status::ResourceLimitExceeded;return result;}

    // Phase C: expand, then verify every branch that is returned.
    result.branches.reserve(result.requiredBranches);
    for(const auto& c:candidates)
    {
        if(c.count==0U)continue;
        if(c.singular)appendSingular(result,p,c);
        else appendRegular(result,p,c.q);
    }

    std::erase_if(result.branches,[&](const Branch& b){return !branchMatches(p,b,target);});
    if(result.branches.empty()){result.status=Status::NoSolution;return result;}
    result.status=std::any_of(result.branches.begin(),result.branches.end(),[](const Branch& b){return b.singular;})?Status::InfiniteFamily:Status::Solutions;
    return result;
}

} // namespace kinemaforge::ik::generated
