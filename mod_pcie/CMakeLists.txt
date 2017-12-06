# PCIe module for LL

include_directories(..)

add_library(xtrxll_pcie SHARED xtrxll_pcie_linux.c)
target_link_libraries(xtrxll_pcie xtrxll)
set_target_properties(xtrxll_pcie PROPERTIES VERSION ${LIBVER} SOVERSION ${MAJOR_VERSION})


install(TARGETS xtrxll_pcie DESTINATION ${XTRXLL_LIBRARY_DIR})
