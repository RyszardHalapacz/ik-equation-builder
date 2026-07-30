#pragma once
#include "common/diagnostic.hpp"
#include <vector>
#include <type_traits>

namespace mt {

class DiagnosticBag {
public:
    void add(Diagnostic diag) {
        if (diag.severity == DiagnosticSeverity::Error) {
            m_has_errors = true;
        }
        m_diagnostics.push_back(std::move(diag));
    }

    void add(DiagnosticSeverity severity, DiagnosticCode code,
             SourceRange location, std::string message) {
        add(Diagnostic{severity, code, location, std::move(message)});
    }

    void merge(DiagnosticBag other) {
        if (other.m_has_errors) { m_has_errors = true; }
        for (auto& d : other.m_diagnostics) {
            m_diagnostics.push_back(std::move(d));
        }
    }

    [[nodiscard]] bool        hasErrors() const noexcept { return m_has_errors; }
    [[nodiscard]] bool        empty()     const noexcept { return m_diagnostics.empty(); }
    [[nodiscard]] std::size_t size()      const noexcept { return m_diagnostics.size(); }

    [[nodiscard]] std::vector<Diagnostic> const& diagnostics() const noexcept {
        return m_diagnostics;
    }

    auto begin() const noexcept { return m_diagnostics.begin(); }
    auto end()   const noexcept { return m_diagnostics.end(); }

private:
    std::vector<Diagnostic> m_diagnostics;
    bool                    m_has_errors = false;
};

static_assert(!std::is_polymorphic_v<DiagnosticBag>, "Virtual dispatch is forbidden");

} // namespace mt
