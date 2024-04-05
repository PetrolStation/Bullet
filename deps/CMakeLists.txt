# disable bullet 3 examples and extras
set(BUILD_BULLET2_DEMOS OFF CACHE BOOL "Build the Bullet 2 demos" FORCE)
set(BUILD_CPU_DEMOS OFF CACHE BOOL "Build the CPU demos" FORCE)
set(BUILD_OPENGL3_DEMOS OFF CACHE BOOL "Build the OpenGL 3 demos" FORCE)
set(BUILD_EXTRAS OFF CACHE BOOL "Build Extras" FORCE)
set(BUILD_UNIT_TESTS OFF CACHE BOOL "Build Unit Tests" FORCE)


add_subdirectory(bullet3)

