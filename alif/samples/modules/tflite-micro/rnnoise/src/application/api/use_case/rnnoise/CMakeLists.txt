#########################################################
#            RNNoise API library                        #
#########################################################
cmake_minimum_required(VERSION 3.21.0)
# Declare a Zephyr library named rnnoise_api
zephyr_library_named(rnnoise_api)

# Add sources
zephyr_library_sources(
    src/RNNoiseProcessing.cpp
    src/RNNoiseModel.cpp
    ${WEST_TOPDIR}/optional/modules/lib/tflite-micro/third_party_static/kissfft/kiss_fft.c
)

# Add include directories
zephyr_library_include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../generated/rnnoise
    ${WEST_TOPDIR}/optional/modules/lib/tflite-micro/third_party_static/kissfft
)

# Link dependencies
zephyr_library_link_libraries(PUBLIC common_api PRIVATE zephyr_interface)

message(STATUS "*******************************************************")
message(STATUS "Library                                : rnnoise_api")
message(STATUS "CMAKE_SYSTEM_PROCESSOR                 : ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "*******************************************************")
