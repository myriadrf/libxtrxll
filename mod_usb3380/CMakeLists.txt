# USB3380 module for LL
include_directories(..)
include_directories(${LIBUSB3380_INCLUDE_DIRS})

if(XTRXLL_STATIC)
    if(CMAKE_SYSTEM_NAME MATCHES "Linux")
        add_definitions(-fPIC)
    endif()
    add_definitions(-DXTRXLL_STATIC)
endif()

add_library(xtrxll_libusb3380 ${PLUGIN_TYPE} xtrxll_libusb3380.c)
target_include_directories(xtrxll_libusb3380 PUBLIC ${LIBUSB_1_INCLUDE_DIRS})
if(NOT XTRXLL_STATIC)
    target_link_libraries(xtrxll_libusb3380 xtrxll ${LIBUSB3380_LIBRARIES})
    set_target_properties(xtrxll_libusb3380 PROPERTIES VERSION ${LIBVER} SOVERSION ${MAJOR_VERSION})

    install(TARGETS xtrxll_libusb3380 DESTINATION ${XTRXLL_LIBRARY_DIR})
else()
    target_link_libraries(xtrxll_libusb3380 ${LIBUSB3380_LIBRARIES})
endif()


########################################################################
# Install udev rules
########################################################################
option(INSTALL_UDEV_RULES "Install udev rules for xtrx" OFF)
if (INSTALL_UDEV_RULES)
    set(UDEV_RULES_PATH
        "/etc/udev/rules.d"
        CACHE STRING
        "Target directory for udev rule installation. Ensure you have permissions to write to this directory."
        )
    install (
        FILES udev/50-xtrx-usb3380.rules
        DESTINATION ${UDEV_RULES_PATH}
        COMPONENT "xtrx-usb"
        )
else (INSTALL_UDEV_RULES)
    message (STATUS "Udev rules not being installed, install them with -DINSTALL_UDEV_RULES=ON")
endif (INSTALL_UDEV_RULES)

