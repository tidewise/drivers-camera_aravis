rock_library(camera_aravis
	SOURCES CameraAravis.cpp
	HEADERS CameraAravis.hpp
	DEPS_PKGCONFIG camera_interface aravis-0.4
	)

rock_executable(camera_aravis_test CameraAravisTest.cpp
	DEPS camera_aravis
	DEPS_PKGCONFIG opencv)

rock_executable(aravis-test-player player.cpp
	DEPS_PKGCONFIG aravis-0.4 opencv)

rock_executable(aravis-test-recorder recorder.cpp
	DEPS_PKGCONFIG aravis-0.4 opencv)
