################################################################################
### Template code. Do not modify                                               #
                                                                               #
cmake_minimum_required (VERSION 3.0.0)                                         #
# Aside from cmake_minimum_required, this must be the first two lines          #
# of the file                                                                  #
file(TO_CMAKE_PATH $ENV{PROPWARE_PATH} PROPWARE_PATH)                          #
include(${PROPWARE_PATH}/CMakePropellerHeader.cmake)                           #
################################################################################

set(BOARD QUICKSTART)
set(MODEL cmm)
set(COMMON_FLAGS "-Os")

project(grbl C CXX ASM COGC COGCXX ECOGC ECOGCXX)

add_executable(${PROJECT_NAME}
    main

    config
    coolant_control
    cpu_map
    defaults
    eeprom
    gcode
    limits
    motion_control
    nuts_bolts
    planner
    print
    probe
    protocol
    report
    serial
    settings
    spindle_control
    stepper
    system)

################################################################################
### Template code. Do not modify                                               #
                                                                               #
# This must be the last line of the file                                       #
include(${PROPWARE_PATH}/CMakePropellerFooter.cmake)                           #
################################################################################
