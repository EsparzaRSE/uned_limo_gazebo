cmake_minimum_required(VERSION 3.8)
project(uned_limo_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/limo_nav2/include
)

set(TARGET_DEPENDS
  rclcpp
  rclcpp_components
  rclcpp_action
  nav2_msgs
  behaviortree_cpp_v3
  yaml-cpp
  tf2 
  tf2_geometry_msgs
)

add_library(circuito_vuelta_component SHARED limo_nav2/src/circuito_vuelta.cpp limo_nav2/src/navigation_behaviors.cpp)
ament_target_dependencies(circuito_vuelta_component ${TARGET_DEPENDS})
rclcpp_components_register_nodes(circuito_vuelta_component "uned_limo_gazebo::CircuitoVuelta")
target_link_libraries(circuito_vuelta_component ${YAML_CPP_LIBRARIES})

install(DIRECTORY 
  limo_bringup 
  limo_description 
  limo_nav2 
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  circuito_vuelta_component
	ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Ejecutable en vez de componente

# Crea el ejecutable e instala el nodo
#add_executable(circuito_vuelta limo_nav2/src/circuito_vuelta.cpp limo_nav2/src/navigation_behaviors.cpp) 
#ament_target_dependencies(circuito_vuelta ${TARGET_DEPENDS})
#target_link_libraries(circuito_vuelta_component ${YAML_CPP_LIBRARIES})

#install(TARGETS
#  circuito_vuelta
#	DESTINATION lib/${PROJECT_NAME}
#)

ament_package()
