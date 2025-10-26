cmake_minimum_required(VERSION 3.5)
project(psyche_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/AudioPCM.msg"
  "msg/AudioChunk.msg"
  "msg/AudioSegment.msg"
  "msg/Conversation.msg"
  "msg/TranscriptEvent.msg"
  "msg/TranscriptPartial.msg"
  "msg/TranscriptSegment.msg"
  "msg/TranscriptWhole.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
