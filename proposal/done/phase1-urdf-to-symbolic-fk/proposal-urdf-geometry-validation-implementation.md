# Proposal: walidacja i normalizacja geometrii URDF — implementacja

## Prompt

> przechodź

Realizacja architektury zatwierdzonej w `proposal-urdf-geometry-validation-architecture.md` (werdykt `approve` po dwóch rundach review). Standardowy format repo: stan obecny + pełny kod zmian, do zatwierdzenia przed naniesieniem na źródła.

## Status weryfikacji — wdrożone

**Zatwierdzone i naniesione na źródła.** Kod wycięty automatycznie z tego dokumentu, więc to, co przeczytałeś, jest dokładnie tym, co trafiło do repo.

- pełny rebuild od zera: **czysty, zero ostrzeżeń**
- `ctest`: **117/117 zielonych** — dokładnie tyle, ile zapowiadał ten dokument (70 dotychczasowych + 47 nowych)
- dziewięć istniejących testów loadera przeszło **bez żadnej modyfikacji**, potwierdzając ocenę migracji z §16 proposalu architektonicznego

### Rozstrzygnięte ryzyko rezydualne

Dokument wymieniał trzy rzeczy, których nie umiałem rozstrzygnąć bez builda. Wszystkie zamknięte:

| Ryzyko | Wynik |
|---|---|
| `std::hypot` trójargumentowy na tym toolchainie | ✅ dostępny, kompiluje się |
| `LoadError` z enuma na strukturę — ukryte porównania `error == LoadError::...` | ✅ brak takich miejsc, kompilacja przeszła |
| **`std::from_chars` a literały subnormalne** | ✅ **akceptuje** — `NormalizesMultiComponentSubnormalAxis` i `NormalizesFullySubnormalAxis` przechodzą, czyli `"4.9406564584124654e-324"` parsuje się do `denorm_min`, a nie zwraca `result_out_of_range` |

Ostatni punkt był jedyną realną niewiadomą tego proposala — gdyby `from_chars` odrzucało denormalne, oba testy subnormalne failowałyby z `malformed_vector` zamiast sprawdzać normalizację, i trzeba by je przeformułować. Nie trzeba.

## Stan obecny

### `src/kinematics/robot_model_loader.hpp`

```cpp
#pragma once
#include <cstdint>
#include <expected>
#include <filesystem>
#include "common/diagnostic_bag.hpp"
#include "kinematics/robot_model.hpp"

namespace mt::kinematics {

enum class LoadError : std::uint8_t {
    file_not_found,
    parse_failure,
    unsupported_joint_type,
    incomplete_kinematic_chain,
    invalid_limits,
};

struct LoadResult {
    RobotModel    model;
    DiagnosticBag diagnostics;
};

[[nodiscard]] std::expected<LoadResult, LoadError>
load_urdf(std::filesystem::path const& urdf_path);

}  // namespace mt::kinematics

// REVIEWED: modern C++ (C++23-first); deviations justified inline.
```

### `src/kinematics/robot_model_loader.cpp` — fragmenty podlegające zmianie

```cpp
std::expected<JointType, LoadError> parse_joint_type(std::string_view s) noexcept {
    if (s == "revolute")  return JointType::revolute;
    if (s == "prismatic") return JointType::prismatic;
    if (s == "fixed")     return JointType::fixed;
    return std::unexpected(LoadError::unsupported_joint_type);
}

// Parses space-separated "x y z" from a null-terminated URDF attribute value.
Vec3 parse_xyz(char const* attr) noexcept {
    Vec3 v{};
    if (!attr || !*attr) return v;
    char const* p   = attr;
    char const* end = p + std::strlen(p);
    double* dst[3]  = {&v.x, &v.y, &v.z};
    for (int i = 0; i < 3; ++i) {
        while (p < end && *p == ' ') ++p;
        auto [next, ec] = std::from_chars(p, end, *dst[i]);
        if (ec != std::errc{}) break;
        p = next;
    }
    return v;
}
```

```cpp
    for (auto node : robot.children("joint")) {
        auto jtype = parse_joint_type(node.attribute("type").as_string());
        if (!jtype) return std::unexpected(LoadError::unsupported_joint_type);

        Joint j;
        j.name = node.attribute("name").as_string();
        j.type = *jtype;

        if (auto origin = node.child("origin"); origin) {
            j.origin_xyz = parse_xyz(origin.attribute("xyz").as_string());
            j.origin_rpy = parse_xyz(origin.attribute("rpy").as_string());
        }
        if (auto axis = node.child("axis"); axis)
            j.axis = parse_xyz(axis.attribute("xyz").as_string());

        j.parent_link = node.child("parent").attribute("link").as_string();
        j.child_link  = node.child("child").attribute("link").as_string();

        if (j.type != JointType::fixed) {
            auto lim = node.child("limit");
            if (!lim) return std::unexpected(LoadError::invalid_limits);
            j.limits.lower    = lim.attribute("lower").as_double();
            j.limits.upper    = lim.attribute("upper").as_double();
            j.limits.velocity = lim.attribute("velocity").as_double();
            j.limits.effort   = lim.attribute("effort").as_double();
            if (!std::isfinite(j.limits.lower) || !std::isfinite(j.limits.upper) ||
                j.limits.lower > j.limits.upper)
                return std::unexpected(LoadError::invalid_limits);
        }
        out.model.add_joint(std::move(j));
    }
```

Pozostałe zwroty błędu w `load_urdf` i `trace_chain`: `file_not_found`, `parse_failure`, `incomplete_kinematic_chain`.

### `src/kinematics/robot_model.hpp` — fragment

```cpp
struct Joint {
    std::string name;
    JointType   type        = JointType::revolute;
    Vec3        axis        = {0.0, 0.0, 1.0};  // unit vector in parent's local frame
    Vec3        origin_xyz  = {};               // [m]
    Vec3        origin_rpy  = {};               // [rad], roll-pitch-yaw
    std::string parent_link;
    std::string child_link;
    JointLimits limits;
};
```

### `src/ik_equations/UrdfModelLoader.cpp` — fragment

```cpp
std::string describe(mt::kinematics::LoadError error)
{
    using mt::kinematics::LoadError;
    switch (error)
    {
    case LoadError::file_not_found:             return "URDF file not found";
    case LoadError::parse_failure:              return "failed to parse URDF XML";
    case LoadError::unsupported_joint_type:     return "unsupported joint type in URDF";
    case LoadError::incomplete_kinematic_chain: return "URDF does not describe a complete kinematic chain";
    case LoadError::invalid_limits:             return "invalid or missing joint limits in URDF";
    }
    return "unknown URDF load error";
}

RobotDescription UrdfModelLoader::load(const std::filesystem::path& urdfPath) const
{
    auto result = mt::kinematics::load_urdf(urdfPath);
    if (!result)
    {
        throw std::runtime_error(
            "UrdfModelLoader: " + describe(result.error()) + " (" + urdfPath.string() + ")");
    }
    ...
}
```

## Co się zmienia

### 1. `src/kinematics/robot_model_loader.hpp` (nowa pełna treść)

```cpp
#pragma once
#include <cstdint>
#include <expected>
#include <filesystem>
#include <optional>
#include <string>
#include "common/diagnostic_bag.hpp"
#include "kinematics/robot_model.hpp"

namespace mt::kinematics {

enum class LoadErrorCode : std::uint8_t {
    file_not_found,
    parse_failure,
    unsupported_joint_type,
    incomplete_kinematic_chain,
    invalid_limits,
    malformed_vector,   // attribute present but not three finite numbers
    degenerate_axis,    // actuated joint whose axis has zero length
};

// Code plus diagnostic context. The code alone is not enough: "malformed
// vector" in a file with thirty joints tells the user nothing useful.
//
// rawValue is optional rather than a plain string because an empty
// attribute (xyz="") is itself a reportable value, and must stay
// distinguishable from "no raw value applies to this error".
struct LoadError {
    LoadErrorCode              code{};
    std::string                jointName;   // empty when not joint-specific
    std::string                attribute;   // e.g. "origin/xyz", "axis/xyz"
    std::optional<std::string> rawValue;    // raw attribute text, truncated
};

struct LoadResult {
    RobotModel    model;
    DiagnosticBag diagnostics;
};

// On success the geometry is canonical:
//   * every joint's origin_xyz and origin_rpy hold only finite values
//   * an actuated joint's axis is finite, non-zero and numerically
//     normalized — exactly +/-1 and 0 for axis-aligned input, 1.0 within a
//     few ULP otherwise
//   * a fixed joint's axis carries no meaning
[[nodiscard]] std::expected<LoadResult, LoadError>
load_urdf(std::filesystem::path const& urdf_path);

}  // namespace mt::kinematics

// REVIEWED: modern C++ (C++23-first); deviations justified inline.
```

### 2. `src/kinematics/robot_model_loader.cpp` (nowa pełna treść)

```cpp
#include "kinematics/robot_model_loader.hpp"
#include <pugixml.hpp>
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstring>
#include <optional>
#include <string_view>
#include <unordered_map>

namespace mt::kinematics {
namespace {

// Longest untrusted text carried into an error. A well-formed vector of
// three full-precision doubles fits in ~70 characters, so this leaves room
// to see the context of a mistake without copying an arbitrarily long
// attribute — or joint name — into an exception message.
constexpr std::size_t kMaxDiagnosticLength = 256;

std::string truncated(std::string_view text) {
    if (text.size() <= kMaxDiagnosticLength) return std::string{text};
    return std::string{text.substr(0, kMaxDiagnosticLength)} + "...";
}

// rawValue is passed as an optional so that an empty attribute (xyz="")
// stays distinguishable from an error that carries no raw value at all.
LoadError makeError(LoadErrorCode code,
                    std::string_view jointName = {},
                    std::string_view attribute = {},
                    std::optional<std::string_view> rawValue = std::nullopt) {
    LoadError error;
    error.code      = code;
    error.jointName = truncated(jointName);   // joint names come from the
    error.attribute = attribute;              // same untrusted XML
    if (rawValue) error.rawValue = truncated(*rawValue);
    return error;
}

// Both parsers stay noexcept: they return plain values and never build a
// LoadError, so no allocation happens on their failure path. Context is
// attached by the caller, which is not noexcept.
std::optional<JointType> parse_joint_type(std::string_view s) noexcept {
    if (s == "revolute")  return JointType::revolute;
    if (s == "prismatic") return JointType::prismatic;
    if (s == "fixed")     return JointType::fixed;
    return std::nullopt;
}

constexpr bool is_space(char c) noexcept {
    return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

// Accepts exactly three finite numbers separated by whitespace:
//
//     whitespace* number whitespace+ number whitespace+ number whitespace*
//
// Rejects a wrong count, trailing garbage, non-finite values ("nan", "inf")
// and out-of-range literals ("1e400"). std::from_chars refuses a leading
// '+', so one is skipped by hand — but at most one, and it may not be
// followed by another sign, otherwise "+-1" would slip through as -1.
std::optional<Vec3> parse_vector3(std::string_view text) noexcept {
    if (text.empty()) return std::nullopt;   // present but empty attribute

    char const* p   = text.data();
    char const* end = p + text.size();

    double components[3]{};
    for (int i = 0; i < 3; ++i) {
        // Numbers must be separated by whitespace: "1-2 3" is not a vector.
        if (i > 0 && (p == end || !is_space(*p))) return std::nullopt;
        while (p < end && is_space(*p)) ++p;
        if (p == end) return std::nullopt;

        if (*p == '+') {
            ++p;
            if (p == end || *p == '+' || *p == '-') return std::nullopt;
        }

        auto const [next, ec] = std::from_chars(p, end, components[i]);
        if (ec != std::errc{})             return std::nullopt;
        if (!std::isfinite(components[i])) return std::nullopt;
        p = next;
    }

    while (p < end && is_space(*p)) ++p;
    if (p != end) return std::nullopt;

    return Vec3{components[0], components[1], components[2]};
}

// Scaling by the largest component first is required, not an optimisation:
// std::hypot alone loses the answer deep in the subnormal range, where the
// true length is not representable. For [denorm_min, denorm_min, 0] it
// returns denorm_min, and dividing by it yields [1, 1, 0] — length sqrt(2).
//
// After scaling, one component is exactly +/-1, so the second length lies
// in [1, sqrt(3)]: far from both ends of the double range.
std::optional<Vec3> normalize_axis(Vec3 axis) noexcept {
    double const scale = std::max({std::abs(axis.x), std::abs(axis.y), std::abs(axis.z)});
    if (scale == 0.0) return std::nullopt;

    double const sx = axis.x / scale;
    double const sy = axis.y / scale;
    double const sz = axis.z / scale;
    double const norm = std::hypot(sx, sy, sz);

    return Vec3{sx / norm, sy / norm, sz / norm};
}

// Traces a linear kinematic chain (parent->child). Collects actuated joint indices.
// Emits Info for each skipped fixed joint. Emits Warning if chain < 6 actuated joints.
std::expected<std::vector<std::size_t>, LoadError>
trace_chain(std::span<Joint const> joints, mt::DiagnosticBag& diag) {
    std::unordered_map<std::string, std::size_t> parent_to_idx;
    std::unordered_map<std::string, bool>        is_child;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        parent_to_idx[joints[i].parent_link] = i;
        is_child[joints[i].child_link]       = true;
    }

    // Root link: appears as a parent but never as a child of any joint.
    std::string current;
    for (auto const& [parent, _] : parent_to_idx) {
        if (!is_child.contains(parent)) { current = parent; break; }
    }
    if (current.empty())
        return std::unexpected(makeError(LoadErrorCode::incomplete_kinematic_chain));

    std::vector<std::size_t> chain;
    while (parent_to_idx.contains(current)) {
        auto const  idx = parent_to_idx.at(current);
        auto const& j   = joints[idx];
        if (j.type == JointType::fixed) {
            diag.add(mt::DiagnosticSeverity::Info,
                     mt::DiagnosticCode::Kinematics_InvalidRobotModel,
                     {}, "fixed joint skipped in kinematic chain: " + j.name);
        } else {
            chain.push_back(idx);
        }
        current = j.child_link;
    }

    if (chain.empty())
        return std::unexpected(makeError(LoadErrorCode::incomplete_kinematic_chain));

    if (chain.size() < 6) {
        diag.add(mt::DiagnosticSeverity::Warning,
                 mt::DiagnosticCode::Kinematics_InvalidRobotModel,
                 {}, "kinematic chain has " + std::to_string(chain.size()) +
                     " actuated joints (expected >=6)");
    }
    return chain;
}

}  // namespace

[[nodiscard]] std::expected<LoadResult, LoadError>
load_urdf(std::filesystem::path const& urdf_path) {
    if (!std::filesystem::exists(urdf_path))
        return std::unexpected(makeError(LoadErrorCode::file_not_found));

    pugi::xml_document doc;
    if (!doc.load_file(urdf_path.c_str()))
        return std::unexpected(makeError(LoadErrorCode::parse_failure));

    auto robot = doc.child("robot");
    if (!robot) return std::unexpected(makeError(LoadErrorCode::parse_failure));

    LoadResult out;
    out.model.set_name(robot.attribute("name").as_string());

    for (auto node : robot.children("link"))
        out.model.add_link(Link{.name = node.attribute("name").as_string()});

    for (auto node : robot.children("joint")) {
        Joint j;
        j.name = node.attribute("name").as_string();

        auto jtype = parse_joint_type(node.attribute("type").as_string());
        if (!jtype)
            return std::unexpected(makeError(LoadErrorCode::unsupported_joint_type,
                                             j.name, "type",
                                             node.attribute("type").as_string()));
        j.type = *jtype;

        // A missing element or a missing attribute means "use the default".
        // An attribute that is present but unparsable is an error — including
        // an empty one, which pugixml cannot distinguish from a missing
        // attribute through as_string(), hence the explicit truthiness test.
        if (auto origin = node.child("origin"); origin) {
            if (auto attribute = origin.attribute("xyz")) {
                auto parsed = parse_vector3(attribute.value());
                if (!parsed)
                    return std::unexpected(makeError(LoadErrorCode::malformed_vector,
                                                     j.name, "origin/xyz",
                                                     attribute.value()));
                j.origin_xyz = *parsed;
            }
            if (auto attribute = origin.attribute("rpy")) {
                auto parsed = parse_vector3(attribute.value());
                if (!parsed)
                    return std::unexpected(makeError(LoadErrorCode::malformed_vector,
                                                     j.name, "origin/rpy",
                                                     attribute.value()));
                j.origin_rpy = *parsed;
            }
        }

        // URDF defaults <axis> to (1,0,0). Syntax is validated for every
        // joint type; only the semantics (non-degenerate) are skipped for
        // fixed joints, whose axis takes no part in the kinematics.
        std::string_view rawAxis;
        j.axis = Vec3{1.0, 0.0, 0.0};
        if (auto axis = node.child("axis"); axis) {
            if (auto attribute = axis.attribute("xyz")) {
                rawAxis = attribute.value();
                auto parsed = parse_vector3(rawAxis);
                if (!parsed)
                    return std::unexpected(makeError(LoadErrorCode::malformed_vector,
                                                     j.name, "axis/xyz", rawAxis));
                j.axis = *parsed;
            }
        }
        if (j.type != JointType::fixed) {
            auto normalized = normalize_axis(j.axis);
            if (!normalized)
                return std::unexpected(makeError(LoadErrorCode::degenerate_axis,
                                                 j.name, "axis/xyz", rawAxis));
            j.axis = *normalized;
        }

        j.parent_link = node.child("parent").attribute("link").as_string();
        j.child_link  = node.child("child").attribute("link").as_string();

        if (j.type != JointType::fixed) {
            auto lim = node.child("limit");
            if (!lim)
                return std::unexpected(makeError(LoadErrorCode::invalid_limits, j.name, "limit"));
            j.limits.lower    = lim.attribute("lower").as_double();
            j.limits.upper    = lim.attribute("upper").as_double();
            j.limits.velocity = lim.attribute("velocity").as_double();
            j.limits.effort   = lim.attribute("effort").as_double();
            if (!std::isfinite(j.limits.lower) || !std::isfinite(j.limits.upper) ||
                j.limits.lower > j.limits.upper)
                return std::unexpected(makeError(LoadErrorCode::invalid_limits, j.name, "limit"));
        }
        out.model.add_joint(std::move(j));
    }

    auto chain = trace_chain(out.model.joints(), out.diagnostics);
    if (!chain) return std::unexpected(chain.error());
    out.model.set_chain(std::move(*chain));

    return out;
}

}  // namespace mt::kinematics
```

### 3. `src/kinematics/robot_model.hpp` — zmiana jednej linii

```cpp
struct Joint {
    std::string name;
    JointType   type        = JointType::revolute;
    Vec3        axis        = {1.0, 0.0, 0.0};  // Unit vector in the joint frame.
    Vec3        origin_xyz  = {};               // [m]
    Vec3        origin_rpy  = {};               // [rad], roll-pitch-yaw
    std::string parent_link;
    std::string child_link;
    JointLimits limits;
};
```

Dwie poprawki w jednej linii:

- **wartość** `{0,0,1}` → `{1,0,0}` — zgodnie ze specyfikacją URDF (*„axis defaults to (1,0,0)"*). Loader i tak nadaje ją jawnie (§2), więc to spójność, nie mechanizm;
- **komentarz** „in parent's local frame" → „in the joint frame" — oś URDF jest wyrażona **po** zastosowaniu `origin`, i to jest dokładnie powód, dla którego `JointTransformBuilder` buduje `T_origin · Rotation(axis, q)` bez obracania osi przez `origin.rpy`.

### 4. `src/ik_equations/UrdfModelLoader.cpp` — zmiana `describe()`

```cpp
namespace {

std::string describeCode(mt::kinematics::LoadErrorCode code)
{
    using Code = mt::kinematics::LoadErrorCode;
    switch (code)
    {
    case Code::file_not_found:             return "URDF file not found";
    case Code::parse_failure:              return "failed to parse URDF XML";
    case Code::unsupported_joint_type:     return "unsupported joint type in URDF";
    case Code::incomplete_kinematic_chain: return "URDF does not describe a complete kinematic chain";
    case Code::invalid_limits:             return "invalid or missing joint limits in URDF";
    case Code::malformed_vector:           return "malformed vector attribute";
    case Code::degenerate_axis:            return "actuated joint has a zero-length axis";
    }
    return "unknown URDF load error";
}

// The lower layer returns data, not prose; the message is assembled here.
std::string describe(const mt::kinematics::LoadError& error)
{
    std::string message = describeCode(error.code);
    if (!error.jointName.empty())
        message += " in joint '" + error.jointName + "'";
    if (!error.attribute.empty())
        message += ", attribute '" + error.attribute + "'";
    // Printed whenever a raw value applies, including when it is empty —
    // xyz="" is exactly the case a reader needs to see quoted.
    if (error.rawValue)
        message += ": \"" + *error.rawValue + "\"";
    return message;
}

} // namespace
```

Reszta pliku bez zmian — `UrdfModelLoader::load` nadal rzuca `std::runtime_error` z tym komunikatem, więc **publiczne API warstwy `ik` się nie zmienia**. Zmienia się wyłącznie treść komunikatu, np.:

```
UrdfModelLoader: malformed vector attribute in joint 'joint_4',
attribute 'axis/xyz': "1 abc 3" (data/urdf/broken.urdf)
```

### 5. `src/ik_equations/model/RobotDescription.hpp` — komentarz kontraktu

```cpp
#pragma once

#include "ik_equations/model/UrdfJoint.hpp"

#include <string>
#include <vector>

namespace kinemaforge::ik {

// Instances produced by UrdfModelLoader carry canonical geometry:
//   * origin.translation and origin.rpy hold only finite values
//   * an actuated joint's axis is finite, non-zero and numerically
//     normalized; it is expressed in the joint frame, not the parent's
//   * a fixed joint's axis carries no meaning
//
// This is a property of the loader, not of the type: RobotDescription is a
// plain aggregate and a hand-built one guarantees nothing.
//
// "Numerically normalized" is not a bitwise promise: axis-aligned input
// normalizes exactly, an arbitrary axis lands within a few ULP of length 1.
struct RobotDescription
{
    std::string name;

    std::vector<UrdfLink> links;
    std::vector<UrdfJoint> joints;
};

} // namespace kinemaforge::ik
```

### 6. `tests/test_urdf_model_loader.cpp` — dopisane testy

Istniejące dziewięć testów **bez zmian** — wszystkie powinny pozostać zielone (§16 proposalu architektonicznego: osie w obu robotach mają normę dokładnie `1.0`, więc normalizacja ich nie ruszy).

Nowe elementy dopisywane na górze pliku:

```cpp
#include "kinematics/robot_model_loader.hpp"   // dla testów kodu błędu

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

// Writes a URDF to a unique temporary file and removes it on destruction.
// Keeping the malformed XML inline in each test is what makes those tests
// readable; a directory of invalid_axis_07.urdf files would not be.
class TemporaryUrdf
{
public:
    explicit TemporaryUrdf(std::string_view contents)
        : path_(std::filesystem::temp_directory_path() /
                ("kinemaforge_test_" + uniqueSuffix() + ".urdf"))
    {
        std::ofstream out(path_, std::ios::binary);
        if (!out)
            throw std::runtime_error("cannot create temporary URDF: " + path_.string());

        out << contents;
        out.close();
        if (!out)
            throw std::runtime_error("cannot write temporary URDF: " + path_.string());
    }

    ~TemporaryUrdf()
    {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }

    TemporaryUrdf(const TemporaryUrdf&) = delete;
    TemporaryUrdf& operator=(const TemporaryUrdf&) = delete;

    const std::filesystem::path& path() const noexcept { return path_; }

private:
    // gtest_discover_tests runs each test as its own process, and ctest may
    // run those in parallel. A per-process counter starting at zero would
    // make several processes fight over the same file — one deleting
    // another's input mid-test. The random salt is drawn once per process.
    static std::string uniqueSuffix()
    {
        static const std::uint64_t salt = [] {
            std::random_device device;
            return (static_cast<std::uint64_t>(device()) << 32) ^ device();
        }();
        static std::atomic<unsigned> counter{0};
        return std::to_string(salt) + "_" + std::to_string(counter++);
    }

    std::filesystem::path path_;
};

// Minimal two-link robot; the caller supplies the joint element.
std::string urdfWithJoint(std::string_view jointXml)
{
    return std::string{R"(<?xml version="1.0"?><robot name="t">)"}
         + R"(<link name="base"/><link name="tool"/>)"
         + std::string{jointXml}
         + "</robot>";
}

std::string revoluteJoint(std::string_view originXml, std::string_view axisXml)
{
    return std::string{R"(<joint name="j" type="revolute">)"}
         + R"(<parent link="base"/><child link="tool"/>)"
         + std::string{originXml} + std::string{axisXml}
         + R"(<limit lower="-1" upper="1" effort="1" velocity="1"/>)"
         + "</joint>";
}

// Loads a URDF given inline; returns the single joint, or throws.
kinemaforge::ik::UrdfJoint loadSingleJoint(std::string_view urdf)
{
    const TemporaryUrdf file{urdf};
    kinemaforge::ik::UrdfModelLoader loader;
    const auto description = loader.load(file.path());
    return description.joints.at(0);
}

// Asserts on the actual reason, not merely "something threw". A bare
// EXPECT_THROW would go green for a file-not-found, a temp-file collision
// or a write failure just as happily as for the malformed input under test.
void expectRejectedWith(std::string_view urdf, mt::kinematics::LoadErrorCode expected)
{
    const TemporaryUrdf file{urdf};
    const auto result = mt::kinematics::load_urdf(file.path());

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, expected);
}

void expectMalformed(std::string_view urdf)
{
    expectRejectedWith(urdf, mt::kinematics::LoadErrorCode::malformed_vector);
}

// The smallest positive double. Written as a literal on purpose:
// std::to_string uses %f with six decimals and would render it "0.000000",
// i.e. a zero axis — the test would then exercise degenerate-axis
// rejection instead of subnormal normalization.
constexpr std::string_view kDenormMin = "4.9406564584124654e-324";

} // namespace
```

**Parsowanie wektorów** — pełna tabela patologii przez `origin/xyz`:

```cpp
TEST(UrdfModelLoaderTest, ParsesValidVector3)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1 2 3"/>)", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.translation.x, 1.0);
    EXPECT_DOUBLE_EQ(joint.origin.translation.y, 2.0);
    EXPECT_DOUBLE_EQ(joint.origin.translation.z, 3.0);
}

TEST(UrdfModelLoaderTest, AcceptsLeadingAndTrailingWhitespace)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="  1 2 3  "/>)", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.translation.x, 1.0);
    EXPECT_DOUBLE_EQ(joint.origin.translation.z, 3.0);
}

TEST(UrdfModelLoaderTest, AcceptsCharacterReferenceTabSeparators)
{
    // A literal tab would be normalized to a space by XML itself, so this
    // test would pass on the old parser too and guard nothing. A character
    // reference survives normalization and reaches the parser as a tab.
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1&#x9;2&#x9;3"/>)", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.translation.y, 2.0);
}

TEST(UrdfModelLoaderTest, AcceptsLeadingPlusSign)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="+1 +2 +3"/>)", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.translation.x, 1.0);
}

TEST(UrdfModelLoaderTest, AcceptsScientificNotation)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1.5e2 0 0"/>)", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.translation.x, 150.0);
}

TEST(UrdfModelLoaderTest, AcceptsNegativeZero)
{
    // Accepting the spelling is the contract; the sign bit is not, since
    // -0.0 == 0.0 and later stages must not depend on it.
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="-0 0 0"/>)", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.translation.x, 0.0);
}

TEST(UrdfModelLoaderTest, RejectsMalformedVectorText)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1 abc 3"/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsMissingVectorComponent)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1 2"/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsExtraVectorComponent)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1 2 3 4"/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsTrailingGarbage)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1 2 3 abc"/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsEmptyVectorAttribute)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz=""/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsOverflowingVectorComponent)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1e400 0 0"/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsNaNVectorComponent)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="nan 0 0"/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsPositiveInfinityVectorComponent)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="inf 0 0"/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsNegativeInfinityVectorComponent)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="-inf 0 0"/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsMultipleLeadingSigns)
{
    // Naively skipping '+' before from_chars would let this through as -1;
    // it is the only sign combination that slips past that mistake.
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="+-1 0 0"/>)", R"(<axis xyz="0 0 1"/>)")));
}
```

**Origin — wartości domyślne i radiany:**

```cpp
TEST(UrdfModelLoaderTest, DefaultsMissingOriginToZero)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint("", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.translation.x, 0.0);
    EXPECT_DOUBLE_EQ(joint.origin.rpy.z, 0.0);
}

TEST(UrdfModelLoaderTest, DefaultsMissingOriginXyzToZero)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint(R"(<origin rpy="0 0 1"/>)", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.translation.x, 0.0);
    EXPECT_DOUBLE_EQ(joint.origin.rpy.z, 1.0);
}

TEST(UrdfModelLoaderTest, DefaultsMissingOriginRpyToZero)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1 0 0"/>)", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.translation.x, 1.0);
    EXPECT_DOUBLE_EQ(joint.origin.rpy.x, 0.0);
}

TEST(UrdfModelLoaderTest, KeepsRpyInRadians)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint(R"(<origin rpy="3.14159 0 0"/>)", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.origin.rpy.x, 3.14159);
}
```

**Podłączenie parsera w pozostałych miejscach użycia** — `origin/xyz` jest już pokryte powyżej:

```cpp
TEST(UrdfModelLoaderTest, RejectsMalformedOriginRpy)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin rpy="1 abc 3"/>)", R"(<axis xyz="0 0 1"/>)")));
}

TEST(UrdfModelLoaderTest, RejectsMalformedAxis)
{
    expectMalformed(urdfWithJoint(
        revoluteJoint(R"(<origin xyz="0 0 0"/>)", R"(<axis xyz="1 abc 3"/>)")));
}
```

**Oś — wartości domyślne, degeneracja, `Fixed`:**

```cpp
TEST(UrdfModelLoaderTest, DefaultsMissingAxisToXForActuatedJoint)
{
    const auto joint = loadSingleJoint(urdfWithJoint(revoluteJoint("", "")));

    EXPECT_DOUBLE_EQ(joint.axis.x, 1.0);
    EXPECT_DOUBLE_EQ(joint.axis.y, 0.0);
    EXPECT_DOUBLE_EQ(joint.axis.z, 0.0);
}

TEST(UrdfModelLoaderTest, DefaultsEmptyAxisElementToXForActuatedJoint)
{
    // <axis/> is a missing attribute, not a bad value: same default as no
    // element at all.
    const auto joint = loadSingleJoint(urdfWithJoint(revoluteJoint("", "<axis/>")));

    EXPECT_DOUBLE_EQ(joint.axis.x, 1.0);
}

TEST(UrdfModelLoaderTest, RejectsEmptyAxisAttribute)
{
    // Present but empty is malformed, unlike a missing attribute.
    expectMalformed(urdfWithJoint(revoluteJoint("", R"(<axis xyz=""/>)")));
}

TEST(UrdfModelLoaderTest, RejectsZeroAxisForRevoluteJoint)
{
    expectRejectedWith(urdfWithJoint(revoluteJoint("", R"(<axis xyz="0 0 0"/>)")),
                       mt::kinematics::LoadErrorCode::degenerate_axis);
}

TEST(UrdfModelLoaderTest, RejectsZeroAxisForPrismaticJoint)
{
    const std::string joint =
        std::string{R"(<joint name="j" type="prismatic">)"}
        + R"(<parent link="base"/><child link="tool"/>)"
        + R"(<axis xyz="0 0 0"/>)"
        + R"(<limit lower="-1" upper="1" effort="1" velocity="1"/>)"
        + "</joint>";
    expectRejectedWith(urdfWithJoint(joint),
                       mt::kinematics::LoadErrorCode::degenerate_axis);
}

TEST(UrdfModelLoaderTest, IgnoresZeroAxisForFixedJoint)
{
    // A fixed joint's axis takes no part in the kinematics, so a degenerate
    // value is not an error.
    //
    // The chain needs an actuated joint as well: trace_chain skips fixed
    // joints and rejects an empty chain, so a robot made of one fixed joint
    // would fail with incomplete_kinematic_chain and never reach the axis
    // check this test is about.
    const std::string urdf =
        R"(<?xml version="1.0"?><robot name="t">)"
        R"(<link name="base"/><link name="middle"/><link name="tool"/>)"
        R"(<joint name="fixed_j" type="fixed">)"
        R"(<parent link="base"/><child link="middle"/>)"
        R"(<axis xyz="0 0 0"/>)"
        R"(</joint>)"
        R"(<joint name="active_j" type="revolute">)"
        R"(<parent link="middle"/><child link="tool"/>)"
        R"(<axis xyz="0 0 1"/>)"
        R"(<limit lower="-1" upper="1" effort="1" velocity="1"/>)"
        R"(</joint></robot>)";

    const TemporaryUrdf file{urdf};
    const auto result = mt::kinematics::load_urdf(file.path());

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->model.joints().size(), 2u);

    // The degenerate axis is kept verbatim, not normalized or rejected.
    const auto& fixed = result->model.joints()[0];
    ASSERT_EQ(fixed.type, mt::kinematics::JointType::fixed);
    EXPECT_DOUBLE_EQ(fixed.axis.x, 0.0);
    EXPECT_DOUBLE_EQ(fixed.axis.y, 0.0);
    EXPECT_DOUBLE_EQ(fixed.axis.z, 0.0);
}

TEST(UrdfModelLoaderTest, RejectsMalformedAxisForFixedJoint)
{
    // Semantics are skipped for fixed joints; syntax never is.
    const std::string joint =
        std::string{R"(<joint name="j" type="fixed">)"}
        + R"(<parent link="base"/><child link="tool"/>)"
        + R"(<axis xyz="1 abc 3"/>)"
        + "</joint>";
    expectMalformed(urdfWithJoint(joint));
}
```

**Normalizacja:**

```cpp
TEST(UrdfModelLoaderTest, NormalizesNonUnitXAxis)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint("", R"(<axis xyz="2 0 0"/>)")));

    EXPECT_DOUBLE_EQ(joint.axis.x, 1.0);
    EXPECT_DOUBLE_EQ(joint.axis.y, 0.0);
}

TEST(UrdfModelLoaderTest, NormalizesNonUnitYAxis)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint("", R"(<axis xyz="0 3 0"/>)")));

    EXPECT_DOUBLE_EQ(joint.axis.y, 1.0);
}

TEST(UrdfModelLoaderTest, NormalizesNonUnitZAxis)
{
    // Axis-aligned input normalizes exactly: hypot returns |component| and
    // dividing a value by itself gives 1.0 bit for bit.
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint("", R"(<axis xyz="0 0 5"/>)")));

    EXPECT_DOUBLE_EQ(joint.axis.z, 1.0);
    EXPECT_DOUBLE_EQ(joint.axis.x, 0.0);
}

TEST(UrdfModelLoaderTest, NormalizesNegativeAxis)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint("", R"(<axis xyz="0 0 -5"/>)")));

    EXPECT_DOUBLE_EQ(joint.axis.z, -1.0);
}

TEST(UrdfModelLoaderTest, KeepsAlreadyUnitAxisBitExact)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint("", R"(<axis xyz="0 0 1"/>)")));

    EXPECT_DOUBLE_EQ(joint.axis.z, 1.0);
}

TEST(UrdfModelLoaderTest, NormalizesArbitraryAxis)
{
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint("", R"(<axis xyz="1 2 3"/>)")));

    // All three components, not just the length: a wrong direction of the
    // right length would otherwise pass.
    const double scale = 1.0 / std::sqrt(14.0);
    EXPECT_NEAR(joint.axis.x, 1.0 * scale, 1e-15);
    EXPECT_NEAR(joint.axis.y, 2.0 * scale, 1e-15);
    EXPECT_NEAR(joint.axis.z, 3.0 * scale, 1e-15);
    EXPECT_NEAR(std::hypot(joint.axis.x, joint.axis.y, joint.axis.z), 1.0, 1e-15);
}

TEST(UrdfModelLoaderTest, NormalizesVeryLargeAxis)
{
    // sqrt(x*x + ...) would overflow to inf here and yield a zero axis.
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint("", R"(<axis xyz="1e200 0 0"/>)")));

    EXPECT_DOUBLE_EQ(joint.axis.x, 1.0);
}

TEST(UrdfModelLoaderTest, NormalizesVerySmallAxis)
{
    // sqrt(x*x + ...) would underflow to zero here and divide by it.
    const auto joint = loadSingleJoint(urdfWithJoint(
        revoluteJoint("", R"(<axis xyz="1e-200 0 0"/>)")));

    EXPECT_DOUBLE_EQ(joint.axis.x, 1.0);
}

TEST(UrdfModelLoaderTest, NormalizesMultiComponentSubnormalAxis)
{
    // std::hypot alone returns denorm_min here, giving [1, 1, 0] — length
    // sqrt(2). Only scaling by the largest component first survives this.
    // A single-component test cannot catch it: that value divides by itself.
    const std::string axis = std::string{R"(<axis xyz=")"}
        + std::string{kDenormMin} + " " + std::string{kDenormMin} + R"( 0"/>)";

    const auto joint = loadSingleJoint(urdfWithJoint(revoluteJoint("", axis)));

    const double expected = 1.0 / std::sqrt(2.0);
    EXPECT_NEAR(joint.axis.x, expected, 1e-15);
    EXPECT_NEAR(joint.axis.y, expected, 1e-15);
    EXPECT_DOUBLE_EQ(joint.axis.z, 0.0);
    EXPECT_NEAR(std::hypot(joint.axis.x, joint.axis.y, joint.axis.z), 1.0, 1e-15);
}

TEST(UrdfModelLoaderTest, NormalizesFullySubnormalAxis)
{
    const std::string value{kDenormMin};
    const std::string axis = std::string{R"(<axis xyz=")"}
        + value + " " + value + " " + value + R"("/>)";

    const auto joint = loadSingleJoint(urdfWithJoint(revoluteJoint("", axis)));

    // Length alone would pass for any unit vector; the direction matters.
    const double expected = 1.0 / std::sqrt(3.0);
    EXPECT_NEAR(joint.axis.x, expected, 1e-15);
    EXPECT_NEAR(joint.axis.y, expected, 1e-15);
    EXPECT_NEAR(joint.axis.z, expected, 1e-15);
}
```

**Regresja na rzeczywistych robotach i `Continuous`:**

```cpp
TEST(UrdfModelLoaderTest, KeepsKr640GeometryValid)
{
    kinemaforge::ik::UrdfModelLoader loader;
    const auto description = loader.load(urdfPath("kr640.urdf"));

    // Expected axes per joint, not just unit length: a loader that turned
    // every axis into [1,0,0] would pass a length-only check.
    struct Expected { const char* name; double x, y, z; };
    const Expected expected[] = {
        {"joint_a1", 0.0, 0.0, 1.0},
        {"joint_a2", 0.0, 1.0, 0.0},
        {"joint_a3", 0.0, 1.0, 0.0},
        {"joint_a4", 1.0, 0.0, 0.0},
        {"joint_a5", 0.0, 1.0, 0.0},
        {"joint_a6", 1.0, 0.0, 0.0},
    };

    for (const auto& e : expected)
    {
        SCOPED_TRACE(e.name);
        const auto it = std::ranges::find(description.joints, e.name,
                                          &kinemaforge::ik::UrdfJoint::name);
        ASSERT_NE(it, description.joints.end());
        EXPECT_DOUBLE_EQ(it->axis.x, e.x);
        EXPECT_DOUBLE_EQ(it->axis.y, e.y);
        EXPECT_DOUBLE_EQ(it->axis.z, e.z);
    }
}

TEST(UrdfModelLoaderTest, KeepsKr4GeometryValid)
{
    kinemaforge::ik::UrdfModelLoader loader;
    const auto description = loader.load(urdfPath("kr4_r600.urdf"));

    // Every actuated joint in this robot spins about local Z.
    for (const auto& joint : description.joints)
    {
        if (joint.type == kinemaforge::ik::JointType::Fixed)
            continue;
        SCOPED_TRACE(joint.name);
        EXPECT_DOUBLE_EQ(joint.axis.x, 0.0);
        EXPECT_DOUBLE_EQ(joint.axis.y, 0.0);
        EXPECT_DOUBLE_EQ(joint.axis.z, 1.0);
    }
}

TEST(UrdfModelLoaderTest, ReturnsMalformedVectorContext)
{
    const TemporaryUrdf file{urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1 abc 3"/>)", R"(<axis xyz="0 0 1"/>)"))};
    const auto result = mt::kinematics::load_urdf(file.path());

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, mt::kinematics::LoadErrorCode::malformed_vector);
    EXPECT_EQ(result.error().jointName, "j");
    EXPECT_EQ(result.error().attribute, "origin/xyz");
    ASSERT_TRUE(result.error().rawValue.has_value());
    EXPECT_EQ(*result.error().rawValue, "1 abc 3");
}

TEST(UrdfModelLoaderTest, ReturnsEmptyRawValueAsPresentButEmpty)
{
    // xyz="" is a reportable value, not an absent one — the optional must
    // be engaged so the message can quote it.
    const TemporaryUrdf file{urdfWithJoint(
        revoluteJoint(R"(<origin xyz=""/>)", R"(<axis xyz="0 0 1"/>)"))};
    const auto result = mt::kinematics::load_urdf(file.path());

    ASSERT_FALSE(result.has_value());
    ASSERT_TRUE(result.error().rawValue.has_value());
    EXPECT_TRUE(result.error().rawValue->empty());
}

TEST(UrdfModelLoaderTest, ReturnsDegenerateAxisContext)
{
    const TemporaryUrdf file{urdfWithJoint(revoluteJoint("", R"(<axis xyz="0 0 0"/>)"))};
    const auto result = mt::kinematics::load_urdf(file.path());

    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, mt::kinematics::LoadErrorCode::degenerate_axis);
    EXPECT_EQ(result.error().jointName, "j");
    EXPECT_EQ(result.error().attribute, "axis/xyz");
    ASSERT_TRUE(result.error().rawValue.has_value());
    EXPECT_EQ(*result.error().rawValue, "0 0 0");
}

TEST(UrdfModelLoaderTest, TruncatesOverlongRawValue)
{
    const std::string overlong(400, '9');
    const std::string origin = R"(<origin xyz=")" + overlong + R"("/>)";
    const TemporaryUrdf file{urdfWithJoint(revoluteJoint(origin, R"(<axis xyz="0 0 1"/>)"))};
    const auto result = mt::kinematics::load_urdf(file.path());

    ASSERT_FALSE(result.has_value());
    ASSERT_TRUE(result.error().rawValue.has_value());
    EXPECT_EQ(result.error().rawValue->size(), 256u + 3u);   // 256 znaków + "..."
    EXPECT_TRUE(result.error().rawValue->ends_with("..."));
}

TEST(UrdfModelLoaderTest, IncludesLoadErrorContextInException)
{
    // The ik layer keeps throwing, but the message must now carry the
    // context the lower layer collected.
    const TemporaryUrdf file{urdfWithJoint(
        revoluteJoint(R"(<origin xyz="1 abc 3"/>)", R"(<axis xyz="0 0 1"/>)"))};
    kinemaforge::ik::UrdfModelLoader loader;

    try
    {
        loader.load(file.path());
        FAIL() << "expected UrdfModelLoader::load to throw";
    }
    catch (const std::runtime_error& error)
    {
        const std::string message = error.what();
        EXPECT_NE(message.find("joint 'j'"), std::string::npos) << message;
        EXPECT_NE(message.find("origin/xyz"), std::string::npos) << message;
        EXPECT_NE(message.find("1 abc 3"), std::string::npos) << message;
    }
}

TEST(UrdfModelLoaderTest, ReportsContinuousAsUnsupportedUntilImplemented)
{
    // TODO: remove together with the proposal adding Continuous support.
    // This pins a known gap as deliberate, so it cannot quietly be taken
    // for intended behaviour.
    const std::string joint =
        std::string{R"(<joint name="j" type="continuous">)"}
        + R"(<parent link="base"/><child link="tool"/>)"
        + R"(<axis xyz="0 0 1"/><limit effort="1" velocity="1"/>)"
        + "</joint>";
    expectRejectedWith(urdfWithJoint(joint),
                       mt::kinematics::LoadErrorCode::unsupported_joint_type);
}
```

### 7. CMake — bez zmian

Nie powstają nowe jednostki translacji ani nowe pliki testowe. `CMakeLists.txt` i `tests/CMakeLists.txt` pozostają nietknięte.

## Uwagi implementacyjne

**Audyt `noexcept` (§12.2a proposalu architektonicznego) — rozwiązany przez konstrukcję.** `LoadError` zaczyna zawierać `std::string`, więc jego budowa może alokować. Zamiast zdejmować `noexcept` z parserów, **zmieniłem ich typ zwracany**: `parse_joint_type` i `parse_vector3` zwracają teraz `std::optional`, nie `std::expected<..., LoadError>`. Nie budują `LoadError`, więc pozostają `noexcept` bez ryzyka `std::terminate` na `std::bad_alloc`. Kontekst dokłada `load_urdf`, które `noexcept` nie ma.

**`trace_chain` zmienia typ błędu**, bo zwraca `std::expected<..., LoadError>` — dostaje `makeError(...)` bez kontekstu jointu (błąd dotyczy całego łańcucha, nie pojedynczego jointu).

**Kolejność w pętli jointów uległa zmianie**: `j.name` jest ustawiane **przed** parsowaniem typu, żeby komunikat o nieobsługiwanym typie mógł podać nazwę jointu. Wcześniej typ był parsowany pierwszy.

**`rawAxis` jako `std::string_view`** wskazuje na bufor pugixml, żywy przez cały czas trwania `load_urdf` — `makeError` kopiuje go do `std::string`, więc nie ma problemu z czasem życia.

**Domyślna oś nadawana bezwarunkowo** (`j.axis = Vec3{1,0,0}` przed sprawdzeniem atrybutu), a nie przez inicjalizator w strukturze. Dzięki temu jedno źródło prawdy jest w parserze, a wartość w `robot_model.hpp` jest wyłącznie spójnościowa.

## Zgodność z zatwierdzoną architekturą

| Decyzja | Gdzie w kodzie |
|---|---|
| dokładnie trzy liczby, białe znaki między nimi | `parse_vector3`, pętla z warunkiem `i > 0 && !is_space(*p)` |
| co najwyżej jeden znak liczby | `parse_vector3`, blok `if (*p == '+')` z kontrolą następnego znaku |
| odrzucanie `nan`/`inf`/przepełnienia | `!std::isfinite(...)` oraz `ec != std::errc{}` |
| odrzucanie śmieci po trzeciej wartości | końcowe `if (p != end) return std::nullopt` |
| brak atrybutu ≠ pusty atrybut | `if (auto attribute = origin.attribute("xyz"))` zamiast `as_string()` |
| `<axis/>` → wartość domyślna | brak wewnętrznego `if` ustawiającego `j.axis`, więc zostaje `{1,0,0}` |
| normalizacja: skalowanie, potem `hypot` | `normalize_axis` |
| odrzucanie przy `scale == 0.0` | `normalize_axis`, pierwszy `if` |
| brak snappingu | brak jakiegokolwiek progu w `normalize_axis` |
| domyślna oś `[1,0,0]` w parserze | `j.axis = Vec3{1.0, 0.0, 0.0};` przed odczytem atrybutu |
| `Fixed`: składnia tak, semantyka nie | `parse_vector3` wołane zawsze; `normalize_axis` tylko dla `type != fixed` |
| dwa nowe kody błędu | `malformed_vector`, `degenerate_axis` |
| kontekst w strukturze, przycięty `rawValue` **i `jointName`** | `LoadError`, `makeError`, `truncated`, `kMaxDiagnosticLength` |
| `rawValue` obecny-ale-pusty odróżnialny od braku | `std::optional<std::string>` w `LoadError` |
| tekst składany w warstwie `ik` | `describeCode` + `describe` w `UrdfModelLoader.cpp` |
| oś w joint frame | komentarz w `robot_model.hpp` |
| kontrakt kanoniczny udokumentowany | komentarze w `robot_model_loader.hpp` i `RobotDescription.hpp` |

## Jak to zweryfikuję po zatwierdzeniu

```powershell
cmake --build build
ctest --test-dir build --output-on-failure
```

Oczekiwany wynik: **70 dotychczasowych + 47 nowych = 117 zielonych**, przy czym dziewięć istniejących testów loadera nie wymaga żadnej modyfikacji (§16 proposalu architektonicznego — osie w obu robotach mają normę dokładnie `1.0`).

Liczba policzona z treści tego dokumentu (`grep -c "^TEST(UrdfModelLoaderTest,"`), nie oszacowana — poprzednia wersja podawała 38 i była błędna.

Trzy rzeczy, które mogą wyjść dopiero przy budowaniu i o których uprzedzam:

1. **`std::hypot` trójargumentowy** wymaga C++17 — projekt jest na C++23, więc powinno być dostępne, ale nie sprawdziłem tego na tym toolchainie.
2. **Zmiana `LoadError` z enuma na strukturę** dotyka `UrdfModelLoader.cpp`; jeśli gdzieś jest porównanie `error == LoadError::...`, przestanie się kompilować. Przejrzałem plik i takich porównań nie ma, ale kompilator jest tu wiarygodniejszy ode mnie.
3. **`std::from_chars` a literały subnormalne** — rozstrzygnięte przy wdrożeniu: akceptuje. Patrz „Status weryfikacji" na górze dokumentu.

*(Poprzednia wersja używała tu `std::to_string(denorm_min)`, co daje `"0.000000"` — czyli oś zerową. Testy sprawdzałyby wtedy odrzucanie osi zdegenerowanej zamiast normalizacji subnormalnej. Naprawione literałem `kDenormMin`.)*

## Do zatwierdzenia

Czekam na ok przed naniesieniem na pliki źródłowe. Bez commita.
