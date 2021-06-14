cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(conditional_tests)

target_sources(app PRIVATE
    src/main.cpp
)

target_link_libraries(app
	PRIVATE
        Mcu::MinimalLibCpp
        Mcu::Warnings
        Mcu::Common
)
