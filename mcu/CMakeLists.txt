cmake_minimum_required (VERSION 2.8.11)
PROJECT(ovenc C CXX)
INCLUDE(cmake_configuration.cmake)

ADD_DEFINITIONS(-std=c++11)
ADD_DEFINITIONS(-Wall -Wextra)
ADD_DEFINITIONS(-fshort-enums)

INCLUDE_DIRECTORIES(.)

INCLUDE_DIRECTORIES("lufa_config/")
INCLUDE_DIRECTORIES(${LUFA_INCLUDE_DIRECTORIES})
ADD_SUBDIRECTORY("lufa_config")

FILE(GLOB LUFA_USB_C
    ${LUFA_LOCATION}/LUFA/Drivers/USB/Class/Device/
    ${LUFA_LOCATION}/LUFA/Drivers/USB/Class/Device/*)
FILE(GLOB LUFA_USB_C_CORE
    ${LUFA_LOCATION}/LUFA/Drivers/USB/Core/
    ${LUFA_LOCATION}/LUFA/Drivers/USB/Core/*.c)
FILE(GLOB_RECURSE LUFA_USB_C_PLATFORM
    ${LUFA_LOCATION}/LUFA/Drivers/USB/Core/${MCU_ARCH}/
    ${LUFA_LOCATION}/LUFA/Drivers/USB/Core/${MCU_ARCH}/*.c)

FILE(GLOB LUFA_PERIPHERAL
    ${LUFA_LOCATION}/LUFA/Drivers/Peripheral/${MCU_ARCH}/
    ${LUFA_LOCATION}/LUFA/Drivers/Peripheral/${MCU_ARCH}/*.c)

ADD_EXECUTABLE(ovenc
    main_mcu.cpp
    thermo_max6675.hh thermo_max6675.cpp
    ${LUFA_USB_C}
    ${LUFA_USB_C_CORE}
    ${LUFA_USB_C_PLATFORM}
    ${LUFA_PERIPHERAL}
    )

TARGET_LINK_LIBRARIES(ovenc
    LufaConfig
    )
