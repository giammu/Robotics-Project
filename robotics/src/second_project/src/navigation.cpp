#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool loadWaypoints(const std::string &file_name, std::vector<std::vector<double>> &waypoints) {
    std::ifstream file(file_name);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", file_name.c_str());
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> waypoint;

        while (std::getline(ss, value, ',')) {
            waypoint.push_back(std::stod(value));
        }

        if (waypoint.size() == 3) { // Ensure correct number of values per line
            waypoints.push_back(waypoint);
        } else {
            ROS_WARN("Skipping invalid waypoint line: %s", line.c_str());
        }
    }

    file.close();
    return true;
}

void sendGoal(const std::vector<double> &waypoint, MoveBaseClient &ac) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = waypoint[0];
    goal.target_pose.pose.position.y = waypoint[1];
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(waypoint[2]);

    ac.sendGoal(goal);
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Successfully reached waypoint (X: %.2f, Y: %.2f, YAW: %.2f)", waypoint[0], waypoint[1], waypoint[2]);
    } else {
        ROS_ERROR("Failed to reach waypoint (X: %.2f, Y: %.2f, YAW: %.2f)", waypoint[0], waypoint[1], waypoint[2]);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh;

    ROS_INFO("--\n\n Navigation node started \n\n--");

    std::string file_name;
    nh.getParam("navigation/waypoints", file_name);

    std::vector<std::vector<double>> waypoints;
    if (!loadWaypoints(file_name, waypoints)) {
        ROS_ERROR("No waypoints loaded. Exiting.");
        return -1;
    }

    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    for (const auto &waypoint : waypoints) {
        ROS_INFO("Sending waypoint (X: %.2f, Y: %.2f, YAW: %.2f)", waypoint[0], waypoint[1], waypoint[2]);
        sendGoal(waypoint, ac);
    }

    ROS_INFO("All waypoints processed. Navigation complete.");
    return 0;
}
