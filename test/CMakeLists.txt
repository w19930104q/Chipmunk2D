include_directories(${chipmunk_SOURCE_DIR}/include)

if(TARGET chipmunk_static)
	set(chipmunk_test_library chipmunk_static)
else()
	set(chipmunk_test_library chipmunk)
endif()

add_executable(sleep_components_test sleep_components.c)
target_link_libraries(sleep_components_test ${chipmunk_test_library})
if(NOT MSVC)
	target_link_libraries(sleep_components_test m)
endif()

add_test(NAME sleep_components COMMAND sleep_components_test)
# The regression scenario used to hang forever; bound the run time.
set_tests_properties(sleep_components PROPERTIES TIMEOUT 120)
