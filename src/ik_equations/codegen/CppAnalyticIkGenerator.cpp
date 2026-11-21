#include "ik_equations/codegen/CppAnalyticIkGenerator.hpp"

#include "ik_equations/builders/KinematicGeometryBuilder.hpp"
#include "ik_equations/codegen/GeneratedIkRuntime.hpp"
#include "ik_equations/model/IkPattern.hpp"
#include "ik_equations/solver/IkPatternDetector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace kinemaforge::ik {
namespace {

using generated::Mat3;
using generated::Parameters;
using generated::Vec3;

struct NumericTransform
{
    Mat3 rotation{};
    Vec3 translation{};
};

Vec3 vec(Vector3 v) noexcept { return {v.x, v.y, v.z}; }
Vec3 add(Vec3 a,Vec3 b) noexcept { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
Vec3 sub(Vec3 a,Vec3 b) noexcept { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
Vec3 scale(Vec3 v,double s) noexcept { return {v.x*s,v.y*s,v.z*s}; }
double dot(Vec3 a,Vec3 b) noexcept { return a.x*b.x+a.y*b.y+a.z*b.z; }
Vec3 cross(Vec3 a,Vec3 b) noexcept { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
double norm(Vec3 v) noexcept { return std::hypot(v.x,v.y,v.z); }
Vec3 projectPerpendicular(Vec3 v,Vec3 axis) noexcept { return sub(v,scale(axis,dot(v,axis))); }

Mat3 identity() noexcept { return {{{1,0,0},{0,1,0},{0,0,1}}}; }
Mat3 transpose(const Mat3& m) noexcept
{
    Mat3 r{}; for(std::size_t i=0;i<3;++i)for(std::size_t j=0;j<3;++j)r[i][j]=m[j][i]; return r;
}
Mat3 multiply(const Mat3& a,const Mat3& b) noexcept
{
    Mat3 r{}; for(std::size_t i=0;i<3;++i)for(std::size_t j=0;j<3;++j)for(std::size_t k=0;k<3;++k)r[i][j]+=a[i][k]*b[k][j]; return r;
}
Vec3 multiply(const Mat3& a,Vec3 v) noexcept
{
    return {a[0][0]*v.x+a[0][1]*v.y+a[0][2]*v.z,
            a[1][0]*v.x+a[1][1]*v.y+a[1][2]*v.z,
            a[2][0]*v.x+a[2][1]*v.y+a[2][2]*v.z};
}
Mat3 rpy(Vector3 value) noexcept
{
    const double cr=std::cos(value.x),sr=std::sin(value.x),cp=std::cos(value.y),sp=std::sin(value.y),cy=std::cos(value.z),sy=std::sin(value.z);
    return {{{cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr},
             {sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr},
             {-sp,cp*sr,cp*cr}}};
}
NumericTransform compose(const NumericTransform& a,const NumericTransform& b) noexcept
{
    return {multiply(a.rotation,b.rotation),add(a.translation,multiply(a.rotation,b.translation))};
}
NumericTransform home(const KinematicChain& chain,const std::optional<FixedRigidTransform>& tcp)
{
    NumericTransform result{identity(),{}};
    for(const auto& joint:chain.joints)
        result=compose(result,{rpy(joint.origin.rpy),vec(joint.origin.translation)});
    if(tcp)result=compose(result,{rpy(tcp->rpy),vec(tcp->translation)});
    return result;
}

bool rotary(JointType t) noexcept { return t==JointType::Revolute||t==JointType::Continuous; }

std::vector<const KinematicJoint*> actuated(const KinematicChain& chain)
{
    std::vector<const KinematicJoint*> out; for(const auto& j:chain.joints)if(j.type!=JointType::Fixed)out.push_back(&j); return out;
}

bool validIdentifier(std::string_view name)
{
    const auto letter = [](char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); };
    if (name.empty() || !(letter(name.front()) || name.front() == '_'))
        return false;
    if (!std::all_of(name.begin() + 1, name.end(), [&](char c) {
            return letter(c) || (c >= '0' && c <= '9') || c == '_';
        }))
        return false;

    static constexpr std::string_view keywords[] = {
        "alignas", "alignof", "and", "and_eq", "asm", "auto", "bitand", "bitor", "bool",
        "break", "case", "catch", "char", "char8_t", "char16_t", "char32_t", "class",
        "compl", "concept", "const", "consteval", "constexpr", "constinit", "const_cast",
        "continue", "co_await", "co_return", "co_yield", "decltype", "default", "delete",
        "do", "double", "dynamic_cast", "else", "enum", "explicit", "export", "extern",
        "false", "float", "for", "friend", "goto", "if", "inline", "int", "long", "mutable",
        "namespace", "new", "noexcept", "not", "not_eq", "nullptr", "operator", "or", "or_eq",
        "private", "protected", "public", "register", "reinterpret_cast", "requires", "return",
        "short", "signed", "sizeof", "static", "static_assert", "static_cast", "struct",
        "switch", "template", "this", "thread_local", "throw", "true", "try", "typedef",
        "typeid", "typename", "union", "unsigned", "using", "virtual", "void", "volatile",
        "wchar_t", "while", "xor", "xor_eq"
    };
    return std::ranges::find(keywords, name) == std::end(keywords);
}

std::unexpected<CodeGenerationError> fail(CodeGenerationErrorCode code,std::string message)
{
    return std::unexpected(CodeGenerationError{code,std::move(message)});
}

std::optional<Vector3> wristCenter(const std::vector<DetectedIkPattern>& patterns)
{
    for(const auto& p:patterns)if(p.kind==IkPatternKind::SixRotarySphericalWrist)return p.evidence.wristCenter;
    return std::nullopt;
}

Mat3 basis(Vec3 x,Vec3 y,Vec3 z) noexcept
{
    return {{{x.x,y.x,z.x},{x.y,y.y,z.y},{x.z,y.z,z.z}}};
}

bool finite(const Parameters& p) noexcept
{
    auto fv=[](Vec3 v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);};
    for(const auto& a:p.axes)if(!fv(a.point)||!fv(a.direction))return false;
    if(!fv(p.wristCenterHome)||!fv(p.wristInTargetFrame)||!fv(p.planarX)||!fv(p.planarY))return false;
    for(const auto& r:p.targetFrameHomeRotation)for(double v:r)if(!std::isfinite(v))return false;
    for(const auto& r:p.wristBasis)for(double v:r)if(!std::isfinite(v))return false;
    if(!std::isfinite(p.link1)||!std::isfinite(p.link2)||!std::isfinite(p.link2HomeAngle)||!std::isfinite(p.axis3Sign)||!std::isfinite(p.axis6Sign))return false;
    for(const auto& l:p.limits)if(l.enabled&&(!std::isfinite(l.lower)||!std::isfinite(l.upper)))return false;
    return true;
}

std::expected<Parameters,CodeGenerationError> buildParameters(const KinematicChain& chain,const std::optional<FixedRigidTransform>& tcp)
{
    const auto joints=actuated(chain);
    if(joints.size()!=6||!std::all_of(joints.begin(),joints.end(),[](const auto* j){return rotary(j->type);}))
        return fail(CodeGenerationErrorCode::UnsupportedKinematicStructure,"C++ backend supports exactly six rotary joints");

    const auto geometryResult=KinematicGeometryBuilder{}.build(chain);
    if(!geometryResult)return fail(CodeGenerationErrorCode::InvalidKinematicGeometry,geometryResult.error().message);
    const auto& g=*geometryResult;
    const auto patterns=IkPatternDetector{}.detect(g);
    const auto center=wristCenter(patterns);
    if(!center)return fail(CodeGenerationErrorCode::UnsupportedKinematicStructure,"supported 6R spherical-wrist pattern not detected");
    if(!axesParallel(g.actuatedJoints[3].axis,g.actuatedJoints[5].axis)||!axesPerpendicular(g.actuatedJoints[3].axis,g.actuatedJoints[4].axis))
        return fail(CodeGenerationErrorCode::UnsupportedKinematicStructure,"generated backend requires an orthogonal X-Y-X-equivalent spherical wrist");

    Parameters p{};
    for(std::size_t i=0;i<6;++i){p.axes[i].point=vec(g.actuatedJoints[i].axis.point);p.axes[i].direction=vec(g.actuatedJoints[i].axis.direction);}
    p.wristCenterHome=vec(*center);

    const NumericTransform h=home(chain,tcp);
    p.targetFrameHomeRotation=h.rotation;
    p.wristInTargetFrame=multiply(transpose(h.rotation),sub(p.wristCenterHome,h.translation));

    const Vec3 d=p.axes[1].direction;
    const Vec3 v1=projectPerpendicular(sub(p.axes[2].point,p.axes[1].point),d);
    p.link1=norm(v1);
    if(p.link1<=kGeometryLinearTolerance)return fail(CodeGenerationErrorCode::UnsupportedKinematicStructure,"planar link 1 is zero");
    p.planarX=scale(v1,1.0/p.link1);
    p.planarY=cross(d,p.planarX);
    const Vec3 p3=add(p.axes[1].point,v1);
    const Vec3 v2=projectPerpendicular(sub(p.wristCenterHome,p3),d);
    p.link2=norm(v2);
    if(p.link2<=kGeometryLinearTolerance)return fail(CodeGenerationErrorCode::UnsupportedKinematicStructure,"planar link 2 is zero");
    p.link2HomeAngle=std::atan2(dot(v2,p.planarY),dot(v2,p.planarX));
    p.axis3Sign=dot(p.axes[2].direction,d)>=0.0?1.0:-1.0;

    Vec3 z=cross(p.axes[3].direction,p.axes[4].direction);
    const double zn=norm(z);
    if(zn<=kGeometryAngularTolerance)return fail(CodeGenerationErrorCode::UnsupportedKinematicStructure,"wrist basis is degenerate");
    z=scale(z,1.0/zn);
    p.wristBasis=basis(p.axes[3].direction,p.axes[4].direction,z);
    p.axis6Sign=dot(p.axes[5].direction,p.axes[3].direction)>=0.0?1.0:-1.0;

    for(std::size_t i=0;i<6;++i)
    {
        p.limits[i].enabled=joints[i]->type!=JointType::Continuous&&joints[i]->limits.hasPositionLimits;
        p.limits[i].lower=joints[i]->limits.lower;
        p.limits[i].upper=joints[i]->limits.upper;
    }
    if(!finite(p))return fail(CodeGenerationErrorCode::NonFiniteParameter,"derived generated-solver parameter is non-finite");
    return p;
}

std::string number(double value)
{
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    stream << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
    return stream.str();
}
std::string vecText(Vec3 v)
{
    return "{"+number(v.x)+", "+number(v.y)+", "+number(v.z)+"}";
}
std::string matText(const Mat3& m)
{
    return "{{{"+number(m[0][0])+", "+number(m[0][1])+", "+number(m[0][2])+"}, {"+
        number(m[1][0])+", "+number(m[1][1])+", "+number(m[1][2])+"}, {"+
        number(m[2][0])+", "+number(m[2][1])+", "+number(m[2][2])+"}}}";
}

std::string emit(const Parameters& p,std::string_view ns)
{
    std::ostringstream o;
    o << "#pragma once\n\n#include \"ik_equations/codegen/GeneratedIkRuntime.hpp\"\n\n";
    o << "namespace " << ns << " {\n\n";
    o << "namespace rt = kinemaforge::ik::generated;\nusing Pose = rt::Pose;\nusing Result = rt::Result;\nusing Status = rt::Status;\nusing Branch = rt::Branch;\nusing CompactResult = rt::CompactResult;\n\n";
    // Every field by name, so a reviewer of the generated file sees which
    // number is which. Designated initialisers must follow declaration order.
    o << "inline constexpr rt::Parameters parameters{\n";
    o << "    .axes = {\n";
    for(std::size_t i=0;i<6;++i)
        o << "        rt::Axis{.point = " << vecText(p.axes[i].point) << ", .direction = " << vecText(p.axes[i].direction) << "}" << (i+1<6?",":"") << "\n";
    o << "    },\n";
    o << "    .wristCenterHome = " << vecText(p.wristCenterHome) << ",\n";
    o << "    .wristInTargetFrame = " << vecText(p.wristInTargetFrame) << ",\n";
    o << "    .targetFrameHomeRotation = " << matText(p.targetFrameHomeRotation) << ",\n";
    o << "    .planarX = " << vecText(p.planarX) << ",\n";
    o << "    .planarY = " << vecText(p.planarY) << ",\n";
    o << "    .link1 = " << number(p.link1) << ",\n";
    o << "    .link2 = " << number(p.link2) << ",\n";
    o << "    .link2HomeAngle = " << number(p.link2HomeAngle) << ",\n";
    o << "    .axis3Sign = " << number(p.axis3Sign) << ",\n";
    o << "    .wristBasis = " << matText(p.wristBasis) << ",\n";
    o << "    .axis6Sign = " << number(p.axis6Sign) << ",\n";
    o << "    .limits = {\n";
    for(std::size_t i=0;i<6;++i)
        o << "        rt::JointLimit{.enabled = " << (p.limits[i].enabled?"true":"false") << ", .lower = " << number(p.limits[i].lower) << ", .upper = " << number(p.limits[i].upper) << "}" << (i+1<6?",":"") << "\n";
    o << "    }\n};\n\n";
    o << "[[nodiscard]] inline Result solve(const Pose& target)\n{\n    return rt::solve(parameters, target);\n}\n\n";
    o << "// All solutions in compact form (per joint: its in-limit full turns); no\n// branch budget. rt::expand() gives what solve() returns.\n";
    o << "[[nodiscard]] inline CompactResult solveCompact(const Pose& target)\n{\n    return rt::solveCompact(parameters, target);\n}\n\n";
    o << "} // namespace " << ns << "\n";
    return o.str();
}

} // namespace

std::expected<std::string,CodeGenerationError>
CppAnalyticIkGenerator::generate(const KinematicChain& chain,
                                 const std::optional<FixedRigidTransform>& tcp,
                                 std::string_view namespaceName) const
{
    if(!validIdentifier(namespaceName))
        return fail(CodeGenerationErrorCode::InvalidNamespaceName,"namespace name is not a valid C++ identifier");
    const auto parameters=buildParameters(chain,tcp);
    if(!parameters)return std::unexpected(parameters.error());
    return emit(*parameters,namespaceName);
}

} // namespace kinemaforge::ik
