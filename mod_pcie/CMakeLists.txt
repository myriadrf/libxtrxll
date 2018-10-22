# PCIe module for LL

include_directories(..)

if(XTRXLL_STATIC)
    if(CMAKE_SYSTEM_NAME MATCHES "Linux")
        add_definitions(-fPIC)
    endif()
    add_definitions(-DXTRXLL_STATIC)
endif()

add_library(xtrxll_pcie ${PLUGIN_TYPE} xtrxll_pcie_linux.c)
if(NOT XTRXLL_STATIC)
    target_link_libraries(xtrxll_pcie xtrxll)
    set_target_properties(xtrxll_pcie PROPERTIES VERSION ${LIBVER} SOVERSION ${MAJOR_VERSION})

    install(TARGETS xtrxll_pcie DESTINATION ${XTRXLL_LIBRARY_DIR})
endif()
