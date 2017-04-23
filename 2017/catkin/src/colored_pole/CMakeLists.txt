cmake_minimum_required(VERSION 2.8.3)
project(colored_pole)

add_definitions("--std=c++14")

execute_process(
  COMMAND grep -e depend ${PROJECT_SOURCE_DIR}/package.xml
  # COMMAND grep -v qt5
  # COMMAND grep -v qtbase5
  COMMAND sed -E "s/(.*)<(.*)>(.*)<\\/\\2>/\\3;/g"
  COMMAND tr -d "\n"
  OUTPUT_VARIABLE ${PROJECT_NAME}_PACKAGE_DEPENDS_LIST
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

file(GLOB ${PROJECT_NAME}_MESSAGES ${PROJECT_SOURCE_DIR}/msg/*.msg)
file(GLOB ${PROJECT_NAME}_SERVICES ${PROJECT_SOURCE_DIR}/srv/*.srv)
file(GLOB ${PROJECT_NAME}_SOURCES  ${PROJECT_SOURCE_DIR}/src/*.cpp)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_PACKAGE_DEPENDS_LIST})
# find_package(Qt5Widgets REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# add_message_files(
#   DIRECTORY ${PROJECT_SOURCE_DIR}/msg
#   FILES     ${${PROJECT_NAME}_MESSAGES}
# )
#
# add_service_files(
#   DIRECTORY ${PROJECT_SOURCE_DIR}/srv
#   FILES     ${${PROJECT_NAME}_SERVICES}
# )

catkin_package(
  CATKIN_DEPENDS ${${PROJECT_NAME}_PACKAGE_DEPENDS}
)

# qt5_wrap_cpp(${PROJECT_NAME}_QT5_MOCS
#   ${PROJECT_SOURCE_DIR}/include/turtlesim/turtle_frame.h
# )

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_SOURCES}
  # ${${PROJECT_NAME}_QT5_MOCS}
)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  # Qt5::Widgets
)

