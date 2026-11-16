#include "ik_equations/UrdfModelLoader.hpp"
#include "ik_equations/builders/KinematicChainBuilder.hpp"
#include "ik_equations/codegen/CppAnalyticIkGenerator.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <locale>
#include <optional>
#include <stdexcept>
#include <string>

namespace ik = kinemaforge::ik;

namespace {
struct CommaDecimal : std::numpunct<char>
{
    char do_decimal_point() const override { return ','; }
};

void generate(const std::filesystem::path& directory, const std::filesystem::path& urdf,
              const char* name, const std::optional<ik::FixedRigidTransform>& tcp = std::nullopt)
{
    const auto model = ik::UrdfModelLoader{}.load(urdf);
    const auto chain = ik::KinematicChainBuilder{}.build(model, "base_link", "tool0");
    if (!chain)
        throw std::runtime_error("cannot build test fixture chain");
    const auto text = ik::CppAnalyticIkGenerator{}.generate(*chain, tcp, name);
    if (!text)
        throw std::runtime_error(text.error().message);
    std::ofstream output(directory / (std::string(name) + ".hpp"));
    output << *text;
    output.close();
    if (!output)
        throw std::runtime_error("cannot write generated test fixture");
}
} // namespace

int main(int argc, char** argv)
{
    if (argc != 2)
        return 1;
    try
    {
        // Exercise locale independence in code that CMake will actually compile.
        std::locale::global(std::locale(std::locale::classic(), new CommaDecimal));
        const std::filesystem::path directory{argv[1]};
        std::filesystem::create_directories(directory);
        const std::filesystem::path robots{KINEMAFORGE_URDF_DATA_DIR};
        const std::filesystem::path testRobots{KINEMAFORGE_TEST_DATA_DIR};
        generate(directory, robots / "kr640.urdf", "kr640_fresh");
        generate(directory, robots / "kr4_r600.urdf", "kr4_fresh");
        generate(directory, robots / "kr640.urdf", "kr640_tcp_fresh",
                 ik::FixedRigidTransform{{0.03, -0.04, 0.12}, {0.2, -0.1, 0.3}});
        generate(directory, testRobots / "equal_links_6r.urdf", "equal_links_fresh");
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
