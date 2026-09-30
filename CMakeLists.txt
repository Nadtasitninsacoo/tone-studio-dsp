cmake_minimum_required(VERSION 3.22)

project(tone-studio-dsp VERSION 1.0.11 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Static MSVC runtime, so the installed engine runs on a machine that has never had Visual
# Studio or the VC++ redistributable. With the default (/MD) the binary needs MSVCP140.dll and
# VCRUNTIME140*.dll, which a clean Windows install does not ship — the engine would fail to
# start on a customer's PC with a DLL error before printing a single line.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# Enable Folder grouping in IDEs
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Cache variables for JUCE setup
set(JUCE_BUILD_HELPER_TOOLS OFF CACHE INTERNAL "")
set(JUCE_COPY_PLUGIN_AFTER_BUILD OFF CACHE INTERNAL "")

# Include FetchContent to download JUCE
include(FetchContent)
FetchContent_Declare(
    JUCE
    GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
    GIT_TAG 7.0.12
)
FetchContent_MakeAvailable(JUCE)

# Add subdirectories
add_subdirectory(engine)
add_subdirectory(app)
