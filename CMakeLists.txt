cmake_minimum_required(VERSION 3.10)
project(SpiegelMessung)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# Pfade für Mac (lokal)
if(APPLE)
    set(CMAKE_PREFIX_PATH "/opt/homebrew/opt/qt@5")
endif()

# Auf Windows (GitHub) versuchen wir den Pfad dynamisch zu finden
if(WIN32)
    # Wir geben CMake ein paar Tipps, wo OpenCV liegen könnte
    set(OpenCV_DIR "C:/tools/opencv/build")
endif()

find_package(Qt5 COMPONENTS Widgets Gui Core REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(SpiegelMessung main.cpp)
target_link_libraries(SpiegelMessung Qt5::Widgets Qt5::Gui Qt5::Core ${OpenCV_LIBS})
