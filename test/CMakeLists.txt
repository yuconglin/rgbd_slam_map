add_executable( run_tracking run_tracking.cpp )
target_link_libraries( run_tracking myslam )

add_executable( run_tracking_localmapping run_tracking_localmapping.cpp )
target_link_libraries( run_tracking_localmapping myslam )