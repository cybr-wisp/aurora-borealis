cmake_minimum_required(VERSION 3.24)

project(aurora_borealis_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 CONFIG REQUIRED)
find_package(Protobuf REQUIRED)
find_package(GTest REQUIRED)
find_package(Threads REQUIRED)

protobuf_generate_cpp(
    PROTO_SRCS
    PROTO_HDRS
    proto/observation.proto
)

add_library(
    aurora_proto
    ${PROTO_SRCS}
    ${PROTO_HDRS}
)

target_link_libraries(
    aurora_proto
    PUBLIC
        protobuf::libprotobuf
)

target_include_directories(
    aurora_proto
    PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
)

add_library(
    aurora_core

    src/geometry/wgs84.cpp
    src/geometry/enu.cpp
    src/geometry/conversions.cpp

    src/estimation/process_model.cpp
    src/estimation/measurement_model.cpp
    src/estimation/ekf.cpp
    src/estimation/ukf.cpp
    src/estimation/consistency.cpp
    src/estimation/maneuver_detector.cpp

    src/ingestion/validator.cpp
    src/ingestion/udp_receiver.cpp
)

target_include_directories(
    aurora_core
    PUBLIC
        src
        ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(
    aurora_core
    PUBLIC
        Eigen3::Eigen
        aurora_proto
        Threads::Threads
)

if(WIN32)
    target_link_libraries(
        aurora_core
        PUBLIC
            ws2_32
    )
endif()

if(MSVC)
    target_compile_options(
        aurora_core
        PRIVATE
            /W4
            /permissive-
            /EHsc
    )
else()
    target_compile_options(
        aurora_core
        PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wshadow
    )
endif()

include(CTest)

if(BUILD_TESTING)
    add_executable(
        aurora_tests

        tests/test_coordinate_transforms.cpp
        tests/test_queue.cpp
        tests/test_ekf_1d.cpp
        tests/test_ukf_1d.cpp
        tests/test_nees.cpp
        tests/test_maneuver_detector.cpp
        tests/test_protobuf_roundtrip.cpp
        tests/test_validator.cpp
    )

    target_link_libraries(
        aurora_tests
        PRIVATE
            aurora_core
            GTest::gtest_main
    )

    include(GoogleTest)
    gtest_discover_tests(aurora_tests)
endif()

add_executable(
    aurora_engine
    src/main.cpp
)

target_link_libraries(
    aurora_engine
    PRIVATE
        aurora_core
)

add_executable(
    bench_ingestion
    benchmarks/bench_ingestion.cpp
)

target_link_libraries(
    bench_ingestion
    PRIVATE
        aurora_core
)

