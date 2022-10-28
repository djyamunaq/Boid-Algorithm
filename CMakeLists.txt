cmake_minimum_required(VERSION 2.8.12)
project (ParticleSystem)

find_package (Eigen3 3.4 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT)

set (CMAKE_SHARED_LINKER_FLAGS "-pthread")
set (CMAKE_CXX_FLAGS "-g")

add_library(Boid ${CMAKE_SOURCE_DIR}/src/Boid.cpp)
target_include_directories(Boid PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(Boid PUBLIC Eigen3::Eigen)

add_library(Quadtree ${CMAKE_SOURCE_DIR}/src/Quadtree.cpp)
target_include_directories(Quadtree PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(Quadtree PUBLIC Eigen3::Eigen Boid)

add_library(Flock ${CMAKE_SOURCE_DIR}/src/Flock.cpp)
target_include_directories(Flock PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(Flock PUBLIC Eigen3::Eigen Boid Quadtree)

add_executable(example_boids ${CMAKE_SOURCE_DIR}/example_boids.cpp)
target_include_directories(example_boids PUBLIC ${CMAKE_SOURCE_DIR}/include ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIRS})
target_link_libraries(example_boids PUBLIC Flock Eigen3::Eigen ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

# add_executable(example_boids3D ${CMAKE_SOURCE_DIR}/example_boids3D.cpp)
# target_include_directories(example_boids3D PUBLIC ${CMAKE_SOURCE_DIR}/include ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIRS})
# target_link_libraries(example_boids3D PUBLIC Eigen3::Eigen ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

add_executable(example_gl ${CMAKE_SOURCE_DIR}/example_gl.cpp)
target_include_directories(example_gl PUBLIC ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIRS})
target_link_libraries(example_gl PUBLIC Eigen3::Eigen ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
