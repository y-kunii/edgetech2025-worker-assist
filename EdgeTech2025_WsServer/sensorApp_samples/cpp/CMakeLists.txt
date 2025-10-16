cmake_minimum_required(VERSION 3.10)
project(SensorApp)

# C++17を使用
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# コンパイラフラグ
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")

# デバッグビルド用フラグ
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -DDEBUG")

# 必要なパッケージを検索
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

# Boost
find_package(Boost REQUIRED COMPONENTS system thread)

# JsonCpp
pkg_check_modules(JSONCPP jsoncpp)
if(NOT JSONCPP_FOUND)
    find_package(jsoncpp REQUIRED)
    set(JSONCPP_LIBRARIES jsoncpp_lib)
endif()

# OpenCV（オプション）
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
    message(STATUS "OpenCV found: ${OpenCV_VERSION}")
    add_definitions(-DUSE_OPENCV)
else()
    message(STATUS "OpenCV not found - camera features disabled")
endif()

# WiringPi（Raspberry Pi用、オプション）
find_library(WIRINGPI_LIBRARY wiringpi)
if(WIRINGPI_LIBRARY)
    message(STATUS "WiringPi found: ${WIRINGPI_LIBRARY}")
    add_definitions(-DUSE_WIRINGPI)
else()
    message(STATUS "WiringPi not found - GPIO features disabled")
endif()

# インクルードディレクトリ
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
)

if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()

# ソースファイル
set(SOURCES
    sensor_app.cpp
)

# 実行ファイル作成
add_executable(sensor_app ${SOURCES})

# ライブラリリンク
target_link_libraries(sensor_app
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_LIBRARIES}
    ${JSONCPP_LIBRARIES}
)

if(OpenCV_FOUND)
    target_link_libraries(sensor_app ${OpenCV_LIBS})
endif()

if(WIRINGPI_LIBRARY)
    target_link_libraries(sensor_app ${WIRINGPI_LIBRARY})
endif()

# インストール
install(TARGETS sensor_app DESTINATION bin)

# CPack設定（パッケージ作成用）
set(CPACK_PACKAGE_NAME "sensor-app")
set(CPACK_PACKAGE_VERSION "1.0.0")
set(CPACK_PACKAGE_DESCRIPTION "Manufacturing Sensor Data Collection App")
set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Manufacturing Team")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libjsoncpp1, libboost-system1.71.0, libboost-thread1.71.0")

include(CPack)