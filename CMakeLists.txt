cmake_minimum_required(VERSION 3.10)
project(SpiegelMessung)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

if(APPLE)
    set(CMAKE_PREFIX_PATH "/opt/homebrew/opt/qt@5")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-headerpad_max_install_names")
endif()

find_package(Qt5 COMPONENTS Widgets Gui Core REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(SpiegelMessung main.cpp)
target_link_libraries(SpiegelMessung Qt5::Widgets Qt5::Gui Qt5::Core ${OpenCV_LIBS})
