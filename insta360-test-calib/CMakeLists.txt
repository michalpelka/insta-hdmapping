cmake_minimum_required(VERSION 3.20)
project(calib_app CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Not pinned to specific COMPONENTS: OpenCV 4.7 folded "aruco" into
# "objdetect" and OpenCV 5.0 split "calib3d" into "calib"/"geometry"/"stereo",
# so a fixed component list fails find_package on one CI runner or the other
# depending on which OpenCV they get (see ArucoCompat.h for the matching
# source-level split). Un-scoped find_package instead pulls in whatever
# modules that install actually has and links them all via ${OpenCV_LIBS}.
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# raylib is installed system-wide but only as a static archive
# (/usr/local/lib/libraylib.a, no .so) — use the static variant of its
# cmake package so find_package looks for libraylib.a instead of libraylib.so.
set(raylib_USE_STATIC_LIBS ON)
find_package(raylib 5.5 REQUIRED)

include(FetchContent)

# ── Dear ImGui ───────────────────────────────────────────────────────────────
# Populated (not FetchContent_MakeAvailable'd) rather than built via its own
# CMakeLists: imgui doesn't ship one, so this is the standard way to vendor it.
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.92.8
    GIT_SHALLOW    TRUE
)
FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
    FetchContent_Populate(imgui)
    add_library(imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp
    )
    target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR})
    target_compile_definitions(imgui PUBLIC IMGUI_DISABLE_OBSOLETE_FUNCTIONS)
endif()

# ── rlImGui (raylib <-> ImGui glue) ──────────────────────────────────────────
# Also hand-populated: its own CMakeLists would FetchContent its own raylib,
# which would fight the system raylib found above.
FetchContent_Declare(
    rlimgui
    GIT_REPOSITORY https://github.com/raylib-extras/rlImGui.git
    GIT_TAG        main
    GIT_SHALLOW    TRUE
)
FetchContent_GetProperties(rlimgui)
if(NOT rlimgui_POPULATED)
    FetchContent_Populate(rlimgui)
    add_library(rlimgui STATIC ${rlimgui_SOURCE_DIR}/rlImGui.cpp)
    target_include_directories(rlimgui PUBLIC ${rlimgui_SOURCE_DIR})
    target_link_libraries(rlimgui PUBLIC raylib imgui)
endif()

add_executable(calib_app
    src/main.cpp
    src/MeiCamera.cpp
    src/Verification.cpp
)

target_include_directories(calib_app PRIVATE src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(calib_app PRIVATE
    raylib
    imgui
    rlimgui
    yaml-cpp::yaml-cpp
    ${OpenCV_LIBS}
)

if(UNIX AND NOT APPLE)
    # The static raylib build bundles GLFW internally but its cmake package
    # does not carry the transitive system libs GLFW itself needs (the X11/GLX
    # backend it was compiled against) — link them explicitly.
    target_link_libraries(calib_app PRIVATE GL m pthread dl rt X11)
endif()

if(MSVC)
    target_compile_options(calib_app PRIVATE /W4)
else()
    target_compile_options(calib_app PRIVATE -Wall -Wextra)
endif()
