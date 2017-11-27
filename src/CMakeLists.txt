set(_srcs mikata.cpp kinematics.cpp )

set(test_srcs test.cpp ${_srcs})
set(kinetest_srcs kinematics_test.cpp ${_srcs})
set(unit_src unit_test.cpp ${_srcs})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../libdynamixel/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../libarm/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../libdynamixel/libaqua/include)

find_package(Threads)

set(ADDITIONAL_LIBRARY arm dynamixel)
# add_library(${PROJECT_NAME} SHARED ${_srcs})
# target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} ${ADDITIONAL_LIBRARY})

add_executable(${PROJECT_NAME}_test ${test_srcs})
target_link_libraries(${PROJECT_NAME}_test ${CMAKE_THREAD_LIBS_INIT} ${ADDITIONAL_LIBRARY})

add_executable(kinematics_test ${kinetest_srcs})
target_link_libraries(kinematics_test ${CMAKE_THREAD_LIBS_INIT} ${ADDITIONAL_LIBRARY})

#if (BUILD_TEST)
add_executable(kinematics_test ${kinetest_srcs})
target_link_libraries(kinematics_test ${CMAKE_THREAD_LIBS_INIT} ${ADDITIONAL_LIBRARY})
#endif (BUilD_TEST)

if(WIN32)
 set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:\"LIBCMT.LIB\"")
endif()

# add_library(${PROJECT_NAME}Static STATIC ${_srcs})
# target_link_libraries(${PROJECT_NAME}Static ${CMAKE_THREAD_LIBS_INIT})

# set_property(TARGET ${PROJECT_NAME}Static APPEND PROPERTY COMPILE_DEFINITIONS EXPORT_${PROJECT_NAME}_STATIC)
