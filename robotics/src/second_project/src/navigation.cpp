#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>

typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char **argv) {
    return 0;
}