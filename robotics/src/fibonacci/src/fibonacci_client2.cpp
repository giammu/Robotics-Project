#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>

typedef actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> Client;

void doneCb(const actionlib::SimpleClientGoalState& state,
            const actionlib_tutorials::FibonacciResultConstPtr& result) {       //questa callback viene chiamata alla fine: e dice....
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    std::stringstream ss;
    for (auto value : result->sequence) {
        ss << value << " ";
    }
    ROS_INFO("Result: %s", ss.str().c_str());
}

void activeCb() {
    ROS_INFO("Goal just went active");
}

void feedbackCb(const actionlib_tutorials::FibonacciFeedbackConstPtr& feedback) {       //questa viene chiamata ogni volta che fa la computation...
    ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
}

void preemptTimerCallback(const ros::TimerEvent&, Client* client) {             //quando il timer termina, questa callback viene chiamata, e serve per terminare il server
    if (client->getState() == actionlib::SimpleClientGoalState::ACTIVE ||
        client->getState() == actionlib::SimpleClientGoalState::PENDING) {
        ROS_INFO("Preempting the current goal due to timeout.");
        client->cancelGoal();
    }
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "test_fibonacci");
    ros::NodeHandle nh;

    Client client("fibonacci", true);           //qui abbiamo "Client" perchè prima l'ha definito con la typedef
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    actionlib_tutorials::FibonacciGoal goal;
    int order = 10; // Default order
    double duration = 5.0; // Default duration in seconds
    nh.param("order", order, 10); // Retrieve order if specified in parameters
    nh.param("duration", duration, 5.0); // Retrieve duration if specified in parameters

    goal.order = order;
    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);             //ogni callback viene chiamata in un momento diverso delle mie actions

    // Setup a timer to preempt the goal after the specified duration
    ros::Timer timer = nh.createTimer(ros::Duration(duration), boost::bind(preemptTimerCallback, _1, &client), true);           // il timer viene chiamato 10 secondi dopo l'inizio della execution

    ros::Rate loop_rate(1);

    while (ros::ok()){
	      ROS_INFO("doing other processing");
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}

