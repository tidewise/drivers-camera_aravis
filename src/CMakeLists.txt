rock_library(camera_aravis
	SOURCES CameraAravis.cpp
	HEADERS CameraAravis.hpp
	DEPS_PKGCONFIG camera_interface aravis-0.6 frame_helper
	)

target_link_libraries(camera_aravis pthread)
rock_executable(camera_aravis_test CameraAravisTest.cpp
	DEPS camera_aravis
	DEPS_PKGCONFIG opencv camera_interface)
TARGET_LINK_LIBRARIES(camera_aravis_test pthread)
