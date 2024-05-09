#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/FibonacciAction.h> //includo anche la mia custom action

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  // create the action client
  actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  actionlib_tutorials::FibonacciGoal goal;
  int order =10;
  double duration =1.0;           //quanto voglio aspettare prima di non essere più interessato nella execution
  ros::param::get("order",order);
  ros::param::get("duration",duration);
  goal.order = order;             //settiamo l'order field (che avevamo anche dentro il file .action)
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(duration));       //usiamo action come se fosse un service, se durante la duration tutto funziona bene waitforresult returna e il bool viene settato a true


  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    
    actionlib_tutorials::FibonacciResultConstPtr  result = ac.getResult();
    for (int i=0; i<result->sequence.size();i++){
       ROS_INFO ("%d ", result->sequence[i]);
    }

  }
  else{
  	ROS_INFO("Action did not finish before the time out.");
	  //ac.cancelGoal ();                                       //devo chiamare manualmente cancelGoal per spegnere il server??
  }
    
  //exit
  return 0;
}
