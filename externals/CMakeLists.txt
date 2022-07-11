#Eigen
add_subdirectory(unsupported)

#json
add_library(json INTERFACE)
target_include_directories(json INTERFACE json/)

#cnpy
add_library(cnpy SHARED IMPORTED GLOBAL)
set_target_properties(cnpy PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/cnpy/libcnpy.so")

#blocsc_filter
add_library(blocsc_filter SHARED IMPORTED GLOBAL)
set_target_properties(blocsc_filter PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/blocsc_filter/libblocsc_filter.so")

