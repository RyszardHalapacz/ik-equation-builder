#pragma once
#include <cstdint>
#include <type_traits>

namespace mt {

struct SourceLocation {
    uint32_t line   = 0;
    uint32_t column = 0;

    constexpr bool operator==(SourceLocation const&) const noexcept = default;
    constexpr bool operator!=(SourceLocation const&) const noexcept = default;
};

struct SourceRange {
    SourceLocation begin;
    SourceLocation end;

    constexpr bool operator==(SourceRange const&) const noexcept = default;
};

static_assert(!std::is_polymorphic_v<SourceLocation>, "Virtual dispatch is forbidden");
static_assert(!std::is_polymorphic_v<SourceRange>,    "Virtual dispatch is forbidden");

} // namespace mt
