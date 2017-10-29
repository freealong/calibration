cmake_minimum_required(VERSION 3.0)
project(calibration)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV 3.0.0 REQUIRED)
find_package(VISP REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(camera_calibration camera_calibration.cpp)
target_link_libraries(camera_calibration ${OpenCV_LIBRARIES})

add_executable(imagelist_creator imagelist_creator.cpp)
target_link_libraries(imagelist_creator ${OpenCV_LIBRARIES})

add_executable(hand_eye hand_eye.cpp)
target_link_libraries(hand_eye ${OpenCV_LIBRARIES} ${VISP_LIBRARIES})

add_executable(test_hand_eye test_hand_eye.cpp)
target_link_libraries(test_hand_eye ${OpenCV_LIBRARIES} ${VISP_LIBRARIES})

add_executable(generate_c2o generate_c2o.cpp)
target_link_libraries(generate_c2o ${OpenCV_LIBRARIES})

add_executable(detect_board_charuco detect_board_charuco.cpp)
target_link_libraries(detect_board_charuco ${OpenCV_LIBRARIES})

add_executable(create_marker create_marker.cpp)
target_link_libraries(create_marker ${OpenCV_LIBRARIES})

add_executable(detect_markers detect_markers.cpp)
target_link_libraries(detect_markers ${OpenCV_LIBRARIES})
