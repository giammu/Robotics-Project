cmake_minimum_required(VERSION 2.8.3)
project(custom_messages)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation) ##quando creo un custom message, qui  è richiesta la message geneeration

add_message_files( ##qui aggiungo il file del custom message
 FILES
 Num.msg

 )
 
generate_messages( ##qui ho le dependecies che servono per generare i custom messages
   DEPENDENCIES
  std_msgs
 )


## Declare a catkin package
catkin_package( CATKIN_DEPENDS message_runtime) ##qui voglio specificare message runtime




## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(pub_custom src/pub.cpp)
add_dependencies(pub_custom custom_messages_generate_messages_cpp) ##aggiungo questo per specificare il custom message
target_link_libraries(pub_custom ${catkin_LIBRARIES})

add_executable(sub_custom src/sub.cpp)
target_link_libraries(sub_custom ${catkin_LIBRARIES})
add_dependencies(sub_custom custom_messages_generate_messages_cpp) ##aggiungo questo per specificare il custom message

