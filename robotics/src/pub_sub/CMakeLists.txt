cmake_minimum_required(VERSION 2.8.3)
project(pub_sub) ## specifico il nome del package che ho creato

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs) ##specifico i package che mi servono, stiamo usando std_msgs

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS}) 

add_executable(pub src/pub.cpp) ## il nostro publisher si chiama pub, e specifico anche il nome del file cpp
target_link_libraries(pub ${catkin_LIBRARIES}) ##includiamo le librerie di catkin perchè non stiamo includendo nessuna libreria più complessa

add_executable(sub src/sub.cpp) ##il nostro subscriber è sub, ecc... come prima
target_link_libraries(sub ${catkin_LIBRARIES})

