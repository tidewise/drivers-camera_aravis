rock_library(camera_aravis
	SOURCES CameraAravis.cpp
	HEADERS CameraAravis.hpp
	DEPS_PKGCONFIG camera_interface aravis-0.4 frame_helper
	)

target_link_libraries(camera_aravis pthread glog)

rock_executable(camera_aravis_test CameraAravisTest.cpp
	DEPS camera_aravis
	DEPS_PKGCONFIG opencv camera_interface)

TARGET_LINK_LIBRARIES(camera_aravis_test pthread)

rock_executable(aravis-test-player player.cpp videolog.cpp
	DEPS_PKGCONFIG aravis-0.4 opencv camera_interface)

rock_executable(aravis-test-converter videolog.cpp converter.cpp
	DEPS_PKGCONFIG aravis-0.4 opencv camera_interface)

rock_executable(aravis-test-recorder recorder.cpp
	DEPS_PKGCONFIG aravis-0.4 opencv camera_interface)

rock_executable(aravis-test-pid pid.cpp
	DEPS_PKGCONFIG aravis-0.4 opencv motor_controller camera_interface)

rock_executable(aravis-test-better-exposure betterExposure.cpp 
	DEPS_PKGCONFIG aravis-0.4 opencv camera_interface)
