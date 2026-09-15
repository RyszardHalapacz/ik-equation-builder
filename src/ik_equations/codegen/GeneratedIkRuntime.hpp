#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
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
    InvalidTarget
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
    std::vector<Branch> branches;
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
        const double q3=(internal-p.link2HomeAngle)/p.axis3Sign;
        const double vx=p.link1+p.link2*std::cos(internal);
        const double vy=p.link2*std::sin(internal);
        const double q2=normalizeAngle(target-std::atan2(vy,vx));
        bool dup=false; for(const auto& e:out) dup|=std::abs(normalizeAngle(e.q2-q2))<kLinearTolerance&&std::abs(normalizeAngle(e.q3-q3))<kLinearTolerance;
        if(!dup) out.push_back({q2,normalizeAngle(q3)});
    }
    return out;
}

inline std::vector<double> equivalents(const JointLimit& l,double q)
{
    q=normalizeAngle(q);
    if(!l.enabled) return {q};
    const long first=static_cast<long>(std::ceil((l.lower-q)/kTwoPi-1e-12));
    const long last=static_cast<long>(std::floor((l.upper-q)/kTwoPi+1e-12));
    std::vector<double> out;
    for(long k=first;k<=last;++k)
    {
        const double v=q+kTwoPi*static_cast<double>(k);
        if(v>=l.lower-1e-12&&v<=l.upper+1e-12) out.push_back(v);
    }
    return out;
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

inline void appendSingular(Result& result,const Parameters& p,const std::array<double,4>& fixed,
                           double q5,double multiplier,double offset)
{
    auto q1=equivalents(p.limits[0],fixed[0]),q2=equivalents(p.limits[1],fixed[1]),
         q3=equivalents(p.limits[2],fixed[2]),q5s=equivalents(p.limits[4],q5);
    if(q1.empty()||q2.empty()||q3.empty()||q5s.empty())return;
    const double fl=p.limits[3].enabled?p.limits[3].lower:-std::numbers::pi_v<double>;
    const double fu=p.limits[3].enabled?p.limits[3].upper: std::numbers::pi_v<double>;
    for(double a:q1)for(double b:q2)for(double c:q3)for(double e:q5s)for(int k=-2;k<=2;++k)
    {
        double off=offset+kTwoPi*static_cast<double>(k),lo=fl,hi=fu;
        if(p.limits[5].enabled)
        {
            const double x=(p.limits[5].lower-off)/multiplier,y=(p.limits[5].upper-off)/multiplier;
            lo=std::max(lo,std::min(x,y)); hi=std::min(hi,std::max(x,y));
        }
        if(lo>hi+1e-12)continue;
        Branch br; br.q={a,b,c,std::numeric_limits<double>::quiet_NaN(),e,std::numeric_limits<double>::quiet_NaN()};
        br.singular=true;br.freeLower=lo;br.freeUpper=hi;br.dependentMultiplier=multiplier;br.dependentOffset=off;result.branches.push_back(br);
    }
}

} // namespace detail

inline Result solve(const Parameters& p,const Pose& target)
{
    using namespace detail;
    Result result;
    if(!validTarget(target)){result.status=Status::InvalidTarget;return result;}

    const Vec3 targetWrist=add(target.position,multiply(target.orientation,p.wristInTargetFrame));
    const Axis& a1=p.axes[0]; const Vec3 d2=p.axes[1].direction;
    const Vec3 v=sub(targetWrist,a1.point);
    const double d1d2=dot(a1.direction,d2),d1v=dot(a1.direction,v);
    const double cc=dot(v,d2)-d1d2*d1v;
    const double sc=-dot(cross(a1.direction,v),d2);
    const double k=dot(a1.point,d2)+d1d2*d1v-dot(p.wristCenterHome,d2);
    const auto q1s=solveSinCos(sc,cc,-k,kLinearTolerance);
    if(q1s.empty()){result.status=Status::NoSolution;return result;}

    for(double q1:q1s)
    {
        const Vec3 unrot=rotatePoint(targetWrist,a1,-q1);
        const Vec3 pt=projectPerpendicular(sub(unrot,p.axes[1].point),d2);
        const double tx=dot(pt,p.planarX),ty=dot(pt,p.planarY);
        for(const auto& arm:solvePlanar(tx,ty,p))
        {
            const Mat3 r123=multiply(multiply(rotation(p.axes[0].direction,q1),rotation(p.axes[1].direction,arm.q2)),rotation(p.axes[2].direction,arm.q3));
            const Mat3 wr=multiply(multiply(transpose(r123),target.orientation),transpose(p.targetFrameHomeRotation));
            const Mat3 local=multiply(multiply(transpose(p.wristBasis),wr),p.wristBasis);
            const double beta=std::acos(std::clamp(local[0][0],-1.0,1.0));
            if(std::abs(std::sin(beta))>kAngularTolerance)
            {
                const double alpha=std::atan2(local[1][0],-local[2][0]);
                const double gamma=std::atan2(local[0][1],local[0][2]);
                appendRegular(result,p,{q1,arm.q2,arm.q3,normalizeAngle(alpha),normalizeAngle(beta),normalizeAngle(gamma/p.axis6Sign)});
                appendRegular(result,p,{q1,arm.q2,arm.q3,normalizeAngle(alpha+std::numbers::pi_v<double>),normalizeAngle(-beta),normalizeAngle((gamma+std::numbers::pi_v<double>)/p.axis6Sign)});
            }
            else if(local[0][0]>=0.0)
            {
                const double theta=std::atan2(local[2][1],local[1][1]);
                appendSingular(result,p,{q1,arm.q2,arm.q3,0.0},0.0,-1.0/p.axis6Sign,theta/p.axis6Sign);
            }
            else
            {
                const double theta=std::atan2(local[1][2],local[1][1]);
                appendSingular(result,p,{q1,arm.q2,arm.q3,0.0},-std::numbers::pi_v<double>,1.0/p.axis6Sign,-theta/p.axis6Sign);
            }
        }
    }

    if(result.branches.empty()){result.status=Status::NoSolution;return result;}
    result.status=std::any_of(result.branches.begin(),result.branches.end(),[](const Branch& b){return b.singular;})?Status::InfiniteFamily:Status::Solutions;
    return result;
}

} // namespace kinemaforge::ik::generated
