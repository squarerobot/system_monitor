cmake_minimum_required(VERSION 2.8.3)
project(system_monitor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation)

add_message_files(
   FILES
   CoreTemp.msg
   CoreUsage.msg
   CPUTemperatureStatus.msg
   CPUUsageStatus.msg
   Diagnostic.msg
   DiagnosticCPUTemperature.msg
   DiagnosticCPUUsage.msg
   DiagnosticHDD.msg
   DiagnosticMEM.msg
   DiagnosticNET.msg
   Disk.msg
   HDDStatus.msg
   Interface.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs  
   #monitor # Or other packages containing msgs
)

catkin_package()
