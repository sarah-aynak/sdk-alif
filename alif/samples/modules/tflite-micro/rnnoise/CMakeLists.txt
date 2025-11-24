# Copyright (C) 2025 Alif Semiconductor - All Rights Reserved.
# Use, distribution and modification of this code is permitted under the
# terms stated in the Alif Semiconductor Software License Agreement
#
# You should have received a copy of the Alif Semiconductor Software
# License Agreement with this file. If not, please write to:
# contact@alifsemi.com, or visit: https://alifsemi.com/license
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(alif_rnnoise)

# Silence "changed in GCC 7.1" warnigns from C++ headers
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-psabi")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-psabi")

set(PLATFORM_DRIVERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/hal/source/platform/zephyr)

add_subdirectory(src/math)
add_subdirectory(src/application/api/common)
add_subdirectory(src/application/api/use_case/rnnoise)

target_include_directories(app PRIVATE
    src/application/main/include
    src/application/api/use_case/rnnoise/include
    src/use_case/alif_rnnoise/include
    src/generated/rnnoise

)

target_compile_definitions(app PRIVATE ACTIVATION_BUF_SZ=CONFIG_ACTIVATION_BUF_SZ)

if (CONFIG_USE_APP_MENU)
target_compile_definitions(app PRIVATE USE_APP_MENU)
endif()

target_sources(app PRIVATE
    src/application/main/Main.cc
    src/use_case/alif_rnnoise/src/MainLoop.cc
    src/use_case/alif_rnnoise/src/UseCaseHandler.cc
    )

if (CONFIG_ARM_ETHOS_U55_128)
target_sources(app PRIVATE src/generated/rnnoise/rnnoise_model.cc)
endif()
target_link_libraries(app PRIVATE
    arm_math
    common_api
)
zephyr_link_libraries(rnnoise_api)
