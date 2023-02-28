include_directories("../../common/include/")

add_library(N100IMU SHARED
	N100IMU_serial)
	
add_executable(N100IMU_test N100IMU_test.cpp)

target_link_libraries(N100IMU_test N100IMU)
