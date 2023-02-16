project(libxtrxll C)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
cmake_minimum_required(VERSION 2.8)

# Set the version information here
set(MAJOR_VERSION 0)
set(API_COMPAT    0)
set(MINOR_VERSION 1)
set(MAINT_VERSION 2)

set(LIBVER  "${MAJOR_VERSION}.${API_COMPAT}.${MINOR_VERSION}")

# determine target architecture
execute_process(COMMAND ${CMAKE_C_COMPILER} -print-multiarch OUTPUT_VARIABLE CC_ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)
if(CC_ARCH MATCHES "i386")
	message(STATUS "Building for i386")
	set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "i386")
	set(ARCH "x86")
elseif(CC_ARCH MATCHES "x86_64")
	message(STATUS "Building for x86_64")
	set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
	set(ARCH "x86")
elseif(CC_ARCH MATCHES "aarch64")
	message(STATUS "Building for aarch64")
	set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
	set(ARCH "arm")
elseif(CC_ARCH MATCHES "arm.*eabihf")
	message(STATUS "Building for armhf")
	set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "armhf")
	set(ARCH "arm")
else()
	message(WARNING "Unknown target architecture ${CC_ARCH}, forcing to FORCE_ARCH=${FORCE_ARCH} variable if it was provided")
	set(ARCH ${FORCE_ARCH})
endif()

set(CROSS_COMPILE_INCLUDE_PATH "/usr/${CC_ARCH}/include")
set(CROSS_COMPILE_LIB_PATH     "/usr/${CC_ARCH}/lib")


if(CMAKE_SYSTEM_NAME MATCHES "Linux")
	set(SYSTEM_LIBS rt dl pthread)
elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
	set(SYSTEM_LIBS "")
	if(CMAKE_COMPILER_IS_GNUCC)
		set(CMAKE_SHARED_LINKER_FLAGS " -Wl,--kill-at -static-libgcc ")
	endif(CMAKE_COMPILER_IS_GNUCC)
elseif(CMAKE_SYSTEM_NAME MATCHES "Darwin")
	set(SYSTEM_LIBS rt dl pthread)
else()
	message(WARNING "Unknown system type is ${CMAKE_SYSTEM_NAME}")
endif()

set(XTRXLL_LIBRARY_DIR      lib${LIB_SUFFIX})
set(XTRXLL_INCLUDE_DIR      include)
set(XTRXLL_UTILS_DIR        bin)

CONFIGURE_FILE(
	${CMAKE_CURRENT_SOURCE_DIR}/libxtrxll.pc.in
	${CMAKE_CURRENT_BINARY_DIR}/libxtrxll.pc
	@ONLY)

INSTALL(
	FILES ${CMAKE_CURRENT_BINARY_DIR}/libxtrxll.pc
	DESTINATION ${XTRXLL_LIBRARY_DIR}/pkgconfig)

# Report all undfined symbols in shared libraries
if(CMAKE_COMPILER_IS_GNUCC)
	add_definitions("-Wl,-z,defs")
endif(CMAKE_COMPILER_IS_GNUCC)


add_definitions(-Wall -g)

option(XTRXLL_STATIC "XTRXLL Static plugin linkage" ON)
if(XTRXLL_STATIC)
	set(PLUGIN_TYPE STATIC)
else(XTRXLL_STATIC)
	set(PLUGIN_TYPE SHARED)
endif(XTRXLL_STATIC)

###############################################################################
# protocol specific drivers
###############################################################################
option(ENABLE_PCIE    "Enable PCIe Low-level XTRX driver"    ON)
option(ENABLE_USB3380 "Enable USB3380 Low-level XTRX driver" ON)

find_package(libusb3380)
if(NOT LIBUSB3380_FOUND)
	if(ENABLE_USB3380)
		message(WARNING "No USB3380 support, install libusb3380 to enable it")
		set(ENABLE_USB3380 OFF)
	endif(ENABLE_USB3380)
endif(NOT LIBUSB3380_FOUND)

if(ENABLE_PCIE)
	add_subdirectory(mod_pcie)
	message(STATUS "xtrxll: PCIe support enabled")
endif(ENABLE_PCIE)

if(ENABLE_USB3380)
	add_subdirectory(mod_usb3380)
	message(STATUS "xtrxll: USB3380 support enabled")
endif(ENABLE_USB3380)

###############################################################################
# common files that shares among protocol specific drivers
###############################################################################
if(ENABLE_PCIE)
	add_definitions(-DENABLE_PCIE)
endif(ENABLE_PCIE)
if(ENABLE_USB3380)
	add_definitions(-DENABLE_USB3380)
endif(ENABLE_USB3380)
if(XTRXLL_STATIC)
	add_definitions(-DXTRXLL_STATIC)
endif(XTRXLL_STATIC)

add_library(xtrxll SHARED
	xtrxll_log.c xtrxll_port.c xtrxll_base.c xtrxll_base_pcie.c
	xtrxll_mmcm.c xtrxll_flash.c xtrxll_api.c)
target_link_libraries(xtrxll ${SYSTEM_LIBS})
if(XTRXLL_STATIC AND ENABLE_PCIE)
	target_link_libraries(xtrxll xtrxll_pcie)
endif()
if(XTRXLL_STATIC AND ENABLE_USB3380)
	target_link_libraries(xtrxll xtrxll_libusb3380)
endif()
set_target_properties(xtrxll PROPERTIES VERSION ${LIBVER} SOVERSION ${MAJOR_VERSION})


add_subdirectory(tests)
install(TARGETS xtrxll DESTINATION ${XTRXLL_LIBRARY_DIR})


########################################################################
# install headers
########################################################################
install(FILES
	xtrxll_api.h
	xtrxll_port.h
	xtrxll_log.h
	xtrxll_mmcm.h
	xtrxll_flash.h
	DESTINATION ${XTRXLL_INCLUDE_DIR})

########################################################################
# package generator
########################################################################
if(NOT CPACK_GENERATOR)
	set(CPACK_GENERATOR DEB)
endif()
set(CPACK_PACKAGE_NAME                "libxtrxll")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "XTRX Low-level API library")
set(CPACK_PACKAGE_VENDOR              "Fairwaves, Inc.")
set(CPACK_PACKAGE_CONTACT             "http://fairwaves.co/wp/contact-us/")
set(CPACK_PACKAGE_VERSION             ${LIBVER}-${MAINT_VERSION})
set(CPACK_DEBIAN_PACKAGE_DEPENDS      "libc6 (>= 2.17)")
if(ENABLE_USB3380)
	set(CPACK_DEBIAN_PACKAGE_DEPENDS      "${CPACK_DEBIAN_PACKAGE_DEPENDS}, libusb3380 (>= 0.0.1)")
endif(ENABLE_USB3380)

set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
set(CPACK_SET_DESTDIR               "")
set(CPACK_PACKAGING_INSTALL_PREFIX  "/usr")
set(CPACK_PACKAGE_CONTACT           "Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>")

include(CPack) #include last
