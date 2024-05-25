#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h>

typedef actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> Client;

void doneCb(const actionlib::SimpleClientGoalState& state,
            const actionlib_tutorials::FibonacciResultConstPtr& result) {       //questa callback viene chiamata alla fine della computazione del server
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    std::stringstream ss;
    for (auto value : result->sequence) {
        ss << value << " ";
    }
    ROS_INFO("Result: %s", ss.str().c_str());
}

void activeCb() { //callback che si triggera quando inizio, non ha arguments
    ROS_INFO("Goal just went active");
}

void feedbackCb(const actionlib_tutorials::FibonacciFeedbackConstPtr& feedback) {       //questa viene chiamata ogni volta che il server fa la computation, e stampa il feedback fornito dal server
    ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
}

void preemptTimerCallback(const ros::TimerEvent&, Client* client) {             //quando il timer termina, questa callback viene chiamata, e serve per terminare il server
    if (client->getState() == actionlib::SimpleClientGoalState::ACTIVE ||       //controllo lo stato del server: non ha senso spegnerlo se è già spento
        client->getState() == actionlib::SimpleClientGoalState::PENDING) {
        ROS_INFO("Preempting the current goal due to timeout.");
        client->cancelGoal();
    }
}

int main (int argc, char **argv) {
    ros::init(argc, argv, "test_fibonacci");
    ros::NodeHandle nh;

    Client client("fibonacci", true);           //qui abbiamo "Client" perchè sopra l'ha definito con la typedef
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    actionlib_tutorials::FibonacciGoal goal;
    int order = 10; // Default order
    double duration = 5.0; // Default duration in seconds
    nh.param("order", order, 10); // Retrieve order if specified in parameters
    nh.param("duration", duration, 5.0); // Retrieve duration if specified in parameters

    goal.order = order;
    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);             //ogni callback viene chiamata in un momento diverso durante l'esecuzione delle mie actions

    // Setup a timer to preempt the goal after the specified duration
    ros::Timer timer = nh.createTimer(ros::Duration(duration), boost::bind(preemptTimerCallback, _1, &client), true);  // il timer viene chiamato 10 secondi dopo l'inizio della execution, e ferma l'execution, bind serve per cambiare la callback che riceviamo perchè vogliamo più di 1 arguments: _1 è il primo argument ed è placeholder, il secondo argument è il client

    ros::Rate loop_rate(1);

    while (ros::ok()){     //questo è un loop che serve per mostrare che è possibile ancora fare la computazione in contemporanea della action
	      ROS_INFO("doing other processing"); //simula una computazione
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}

