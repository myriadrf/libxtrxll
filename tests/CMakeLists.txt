# Low-level tests
include_directories(..)

add_executable(test_xtrxll test_xtrxll.c)
target_link_libraries(test_xtrxll xtrxll ${SYSTEM_LIBS})

add_executable(test_xtrxflash test_xtrxflash.c)
target_link_libraries(test_xtrxflash xtrxll ${SYSTEM_LIBS})

install(TARGETS test_xtrxll test_xtrxflash DESTINATION ${XTRXLL_UTILS_DIR})
