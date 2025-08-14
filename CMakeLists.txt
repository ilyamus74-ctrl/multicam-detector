cmake_minimum_required(VERSION 3.16)
project(multicam_yolovo_v1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Исполняемый файл
add_executable(multicam_yolovo_v1
  main.cc
  src/config_loader.cc
  src/v4l2_profiles.cc
  src/npu_worker_pool.cc
  src/tracker_sort.cc
  src/global_id_fuser.cc
  src/camera_worker.cc
  src/camera_manager.cc
  src/web_server.cc
)

# Инклюды (заголовки лежат в корне/ include/ src/)
target_include_directories(multicam_yolovo_v1 PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Потоки
find_package(Threads REQUIRED)
target_link_libraries(multicam_yolovo_v1 PRIVATE Threads::Threads)

# Чуть-чуть варнингов на GCC/Clang
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(multicam_yolovo_v1 PRIVATE -Wall -Wextra -Wpedantic)
endif()
