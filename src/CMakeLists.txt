rock_library(camera_aravis
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(camera_aravis_bin Main.cpp
    DEPS camera_aravis)

