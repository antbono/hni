cmake_minimum_required(VERSION 3.8)
project(hni_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hni_interfaces REQUIRED)
find_package(nao_led_interfaces REQUIRED)
find_package(nao_lola_sensor_msgs REQUIRED)
find_package(nao_lola_command_msgs REQUIRED)
find_package(nao_pos_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)


# Include Cpp "include" directory
include_directories(include)


#################### JOINTS_PLAY_ACTION_SERVER ###################
add_library(joints_play_server SHARED
  src/joints_play_action_server.cpp
)
target_include_directories(joints_play_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(joints_play_server
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL"
)

set(JOINTS_PLAY_SERVER_INCLUDE_DEPENDS
  "hni_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nao_lola_command_msgs")
ament_target_dependencies(joints_play_server ${JOINTS_PLAY_SERVER_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${JOINTS_PLAY_SERVER_INCLUDE_DEPENDS})

rclcpp_components_register_node( joints_play_server 
  PLUGIN "hni_joints_play_action_server::JointsPlayActionServer" 
    EXECUTABLE joints_play_action_server
)

# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS joints_play_server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)


#################### JOINTS_PLAY_ACTION_CLIENT #####################
add_library(joints_play_client SHARED
  src/joints_play_action_client.cpp
)
target_include_directories(joints_play_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(joints_play_client
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL"
)

set(JOINTS_PLAY_CLIENT_INCLUDE_DEPENDS
  "hni_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")

ament_target_dependencies(joints_play_client ${JOINTS_PLAY_CLIENT_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${JOINTS_PLAY_CLIENT_INCLUDE_DEPENDS})

rclcpp_components_register_node(joints_play_client 
  PLUGIN "hni_joints_play_action_client::JointsPlayActionClient" 
    EXECUTABLE joints_play_action_client
)
# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS joints_play_client
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)


#################### HEAD_TRACK_SERVER ###########################
add_library(head_track_server SHARED
  src/head_track_action_server.cpp
)
target_include_directories(head_track_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(head_track_server
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL"
)

set(HEAD_TRACK_SERVER_INCLUDE_DEPENDS
  ament_index_cpp
  rclcpp
  rclcpp_action
  rclcpp_components
  hni_interfaces
  nao_lola_sensor_msgs
  nao_lola_command_msgs
  std_msgs)

ament_target_dependencies(head_track_server ${HEAD_TRACK_SERVER_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${HEAD_TRACK_SERVER_INCLUDE_DEPENDS})

rclcpp_components_register_node(head_track_server 
  PLUGIN "hni_head_track_action_server::HeadTrackActionServer" 
  EXECUTABLE head_track_action_server
)
# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS head_track_server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)


#################### HEAD_TRACK_SERVER2 ###########################
add_library(head_track_server2 SHARED
  src/head_track_action_server2.cpp
)
target_include_directories(head_track_server2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(head_track_server2
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL"
)

set(HEAD_TRACK_SERVER_INCLUDE_DEPENDS
  ament_index_cpp
  rclcpp
  rclcpp_action
  rclcpp_components
  hni_interfaces
  nao_lola_sensor_msgs
  nao_lola_command_msgs
  std_msgs)

ament_target_dependencies(head_track_server2 ${HEAD_TRACK_SERVER_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${HEAD_TRACK_SERVER_INCLUDE_DEPENDS})

rclcpp_components_register_node(head_track_server2 
  PLUGIN "hni_head_track_action_server2::HeadTrackActionServer2" 
  EXECUTABLE head_track_action_server2
)
# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS head_track_server2
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)


#################### HEAD_TRACK_CLIENT ###########################
add_library(head_track_client SHARED
  src/head_track_action_client.cpp
)
target_include_directories(head_track_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(head_track_client
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL"
)
ament_target_dependencies(head_track_client
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "nao_lola_command_msgs"
  "nao_lola_sensor_msgs"
  "hni_interfaces"
)
rclcpp_components_register_node(head_track_client 
  PLUGIN "hni_head_track_cpp::HeadTrackActionClient" 
  EXECUTABLE head_track_action_client
)
install(TARGETS
  head_track_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


#################### CHAT ACTION SERVER ###########################
add_library(chat_server SHARED
  src/chat_action_server.cpp)

target_include_directories(chat_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(chat_server
  PLUGIN "hni_chat_action_server::ChatActionServer" 
  EXECUTABLE chat_action_server
)

target_compile_definitions(chat_server
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")


set(CHAT_SERVER_INCLUDE_DEPENDS
  "hni_interfaces"
  "nao_led_interfaces"
  "nao_lola_command_msgs"
  "nao_lola_sensor_msgs"  
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_srvs")

target_link_libraries(chat_server joints_play_client gstt_client gtts_client chat_serv_client)
ament_target_dependencies(chat_server ${CHAT_SERVER_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${CHAT_SERVER_INCLUDE_DEPENDS})

install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS chat_server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

#################### CHAT ACTION CLIENT ###########################
add_library(chat_client SHARED
  src/chat_action_client.cpp)

target_include_directories(chat_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(chat_client
  PLUGIN "hni_chat_action_client::ChatActionClient" 
  EXECUTABLE chat_action_client
)

#target_compile_definitions(chat_client
#  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")

set(CHAT_CLIENT_INCLUDE_DEPENDS
  "hni_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_srvs")

ament_target_dependencies(chat_client ${CHAT_SERVER_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${CHAT_SERVER_INCLUDE_DEPENDS})

install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS chat_client
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)


#################### GSTT SERVICE CLIENT ###########################

add_library(gstt_client SHARED
  src/gstt_service_client.cpp)

target_include_directories(gstt_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(gstt_client
  PLUGIN "hni_gstt_service_client::GsttServiceClient" 
  EXECUTABLE gstt_service_client
)


set(GSTT_CLIENT_INCLUDE_DEPENDS
  "rclcpp"
  "rclcpp_components"
  "std_srvs")

ament_target_dependencies(gstt_client ${GSTT_CLIENT_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${GSTT_CLIENT_INCLUDE_DEPENDS})

install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS gstt_client
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)


#################### GTTS SERVICE CLIENT ###########################

add_library(gtts_client SHARED
  src/gtts_service_client.cpp)

target_include_directories(gtts_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(gtts_client
  PLUGIN "hni_gtts_service_client::GttsServiceClient" 
  EXECUTABLE gtts_service_client
)

set(GTTS_CLIENT_INCLUDE_DEPENDS
  "hni_interfaces"
  "rclcpp"
  "rclcpp_components"
  )

ament_target_dependencies(gtts_client ${GTTS_CLIENT_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${GTTS_CLIENT_INCLUDE_DEPENDS})

install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS gtts_client
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

#################### CHAT SERVICE CLIENT ###########################

add_library(chat_serv_client SHARED
  src/chat_service_client.cpp)

target_include_directories(chat_serv_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_node(chat_serv_client
  PLUGIN "hni_chat_service_client::ChatServiceClient" 
  EXECUTABLE chat_service_client
)

set(CHAT_CLIENT_INCLUDE_DEPENDS
  "hni_interfaces"
  "rclcpp"
  "rclcpp_components"
  )

ament_target_dependencies(chat_serv_client ${CHAT_CLIENT_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${CHAT_CLIENT_INCLUDE_DEPENDS})

install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS chat_serv_client
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

  # Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


#################### TEST ###############################
if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# comment the line when a copyright and license is added to all source files
set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
