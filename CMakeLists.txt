cmake_minimum_required(VERSION 3.16)

PA_NEW(Bullet)

PA_INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/deps/bullet3/src
)

PA_LINK_LIBRARIES(
    BulletDynamics BulletCollision LinearMath Bullet3Common
)

File(GLOB_RECURSE SOURCE_FILES src/*.cpp )
File(GLOB_RECURSE HEADER_FILES src/*.h   )

pa_add_source_files(
    ${SOURCE_FILES}
    ${HEADER_FILES}
)

add_subdirectory(deps)
