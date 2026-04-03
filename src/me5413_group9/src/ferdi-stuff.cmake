add_executable(show_img ${CMAKE_CURRENT_LIST_DIR}/show_img.cpp) # ${COMMON_SOURCES}
target_link_libraries(show_img ${catkin_LIBRARIES} ${OpenCV_LIBS})