cmake_minimum_required(VERSION 3.12)
project(ROBSTRIDE_CONTROL_UPDATE VERSION 1.0.0 LANGUAGES CXX)

# ─── C++ standard ────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ─── compiler flags ──────────────────────────────────────────────────
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -DDEBUG")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -DNDEBUG")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pthread")

# ─── include directories ──────────────────────────────────────────────
include_directories(include)

# ─── existing targets ────────────────────────────────────────────────
add_executable(robstride-mit-position src/position_control.cpp)
add_executable(mit-multi-motor-position src/multi_motor_position.cpp)
add_executable(single_motor_test examples/hello_motor.cpp src/can_interfaces.cpp)
add_executable(multi_motor_functionality examples/multi_motor_functionality.cpp src/can_interfaces.cpp)
add_executable(sinusoidal_actuator_test examples/sinusoidal_actuator_test.cpp src/can_interfaces.cpp)
target_link_libraries(multi_motor_functionality pthread)
target_link_libraries(sinusoidal_actuator_test pthread)
add_executable(my_Test src/my_Test.cpp)
# ─── Dear ImGui GUI target ───────────────────────────────────────────
# Use local imgui source if available, otherwise fetch
if(EXISTS "${CMAKE_BINARY_DIR}/_deps/imgui-src")
    set(imgui_SOURCE_DIR "${CMAKE_BINARY_DIR}/_deps/imgui-src")
else()
    include(FetchContent)
    FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG        v1.91.9
    )
    FetchContent_MakeAvailable(imgui)
endif()

find_package(SDL2 REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(motor_gui
    examples/motor_gui.cpp
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(motor_gui PRIVATE
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${SDL2_INCLUDE_DIRS}
)

target_link_libraries(motor_gui PRIVATE
    ${SDL2_LIBRARIES}
    OpenGL::GL
    dl
    pthread
)

# ─── LibTorch ────────────────────────────────────────────────────────
# expects libtorch/ folder sitting next to CMakeLists.txt
# override with: cmake .. -DCMAKE_PREFIX_PATH=/path/to/libtorch
set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/libtorch")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

# ─── PPO training target ──────────────────────────────────────────────
add_executable(ppo_train
    src/ppo.cpp
    src/can_interfaces.cpp
)

target_include_directories(ppo_train PRIVATE
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/libtorch/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libtorch/include/torch/csrc/api/include
)

target_link_libraries(ppo_train PRIVATE
    ${TORCH_LIBRARIES}
    pthread
)

set_property(TARGET ppo_train PROPERTY CXX_STANDARD 17)

# ─── TD3 training target ──────────────────────────────────────────────
add_executable(td3_train
    src/td3_train.cpp
    src/can_interfaces.cpp
)

target_include_directories(td3_train PRIVATE
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/libtorch/include
    ${CMAKE_CURRENT_SOURCE_DIR}/libtorch/include/torch/csrc/api/include
)

target_link_libraries(td3_train PRIVATE
    ${TORCH_LIBRARIES}
    pthread
)

set_property(TARGET td3_train PROPERTY CXX_STANDARD 17)

# ─── link libraries (existing targets) ───────────────────────────────
target_link_libraries(robstride-mit-position pthread)
target_link_libraries(mit-multi-motor-position)
target_link_libraries(single_motor_test)

# ─── installation ────────────────────────────────────────────────────
install(TARGETS
    robstride-mit-position
    mit-multi-motor-position
    single_motor_test
    sinusoidal_actuator_test
    ppo_train
    td3_train
    my_Test
    RUNTIME DESTINATION bin
)

# ─── testing ─────────────────────────────────────────────────────────
enable_testing()
add_test(NAME motor_connection_test
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_motor.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

# ─── custom targets ───────────────────────────────────────────────────
add_custom_target(setup-can
    COMMAND sudo ip link set can0 type can bitrate 1000000
    COMMAND sudo ip link set up can0
    COMMENT "Setting up CAN interface"
)

add_custom_target(install-deps
    COMMAND sudo apt-get update
    COMMAND sudo apt-get install -y build-essential can-utils cmake
    COMMENT "Installing dependencies"
)

# ─── package configuration ────────────────────────────────────────────
set(CPACK_PACKAGE_NAME "robstride-control")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "tianrking Motor Control Library")
set(CPACK_PACKAGE_VENDOR "tianrking")
set(CPACK_GENERATOR "DEB;RPM")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.17), libgcc1 (>= 1:4.2), libstdc++6 (>= 5.2)")
include(CPack)