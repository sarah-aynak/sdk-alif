# Required minimum CMake version
cmake_minimum_required(VERSION 3.20.0)

# Find Zephyr (uses $ZEPHYR_BASE from your environment)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Name of your project
project(my_littlefs_project)

# Add your application source files
target_sources(app PRIVATE src/main.c)
