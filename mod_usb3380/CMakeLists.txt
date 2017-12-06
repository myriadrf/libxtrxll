# USB3380 module for LL
include_directories(..)
include_directories(${LIBUSB3380_INCLUDE_DIR})


add_library(xtrxll_libusb3380 SHARED xtrxll_libusb3380.c)
target_include_directories(xtrxll_libusb3380 PUBLIC ${LIBUSB_1_INCLUDE_DIRS})
target_link_libraries(xtrxll_libusb3380 xtrxll ${LIBUSB3380_LIBRARIES})
set_target_properties(xtrxll_libusb3380 PROPERTIES VERSION ${LIBVER} SOVERSION ${MAJOR_VERSION})


install(TARGETS xtrxll_libusb3380 DESTINATION ${XTRXLL_LIBRARY_DIR})

install(FILES udev/50-xtrx-usb3380.rules DESTINATION "../etc/udev/rules.d" COMPONENT xtrx-usb)
