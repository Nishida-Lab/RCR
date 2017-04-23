cmake_minimum_required(VERSION 2.8.3)
project(robocar_sim2d)

add_definitions("--std=c++14")

execute_process(
  COMMAND grep -e depend ${PROJECT_SOURCE_DIR}/package.xml
  COMMAND grep -v qt5
  COMMAND sed -E "s/(.*)<(.*)>(.*)<\\/\\2>/\\3;/g"
  COMMAND tr -d "\n"
  OUTPUT_VARIABLE ${PROJECT_NAME}_PACKAGE_DEPENDS_LIST
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

file(GLOB ${PROJECT_NAME}_MESSAGES ${PROJECT_SOURCE_DIR}/msg/*.msg)
file(GLOB ${PROJECT_NAME}_SERVICES ${PROJECT_SOURCE_DIR}/srv/*.srv)
file(GLOB ${PROJECT_NAME}_SOURCES  ${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME}_node.cpp)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_PACKAGE_DEPENDS_LIST})
find_package(Qt5Widgets REQUIRED)

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

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_SOURCES}
)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

