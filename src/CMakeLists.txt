#rock_library(camera_aravis
#    SOURCES Dummy.cpp
#    HEADERS Dummy.hpp)

rock_executable(aravis-test-player player.cpp
	DEPS_PKGCONFIG aravis-0.4 opencv)

rock_executable(aravis-test-recorder recorder.cpp
	DEPS_PKGCONFIG aravis-0.4 opencv)
