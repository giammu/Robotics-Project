#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h> //includiamo il generic ros action server
#include <actionlib_tutorials/FibonacciAction.h> //e anche quello generato dal .action file nel folder action

class FibonacciAction
{
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  actionlib_tutorials::FibonacciFeedback feedback_;
  actionlib_tutorials::FibonacciResult result_;

public:

  FibonacciAction(std::string name) :
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false), //bind serve per collegarlo alla callback, lo inizializzo a false così non parte automaticamente, e poi
    action_name_(name)
  {
    as_.start();
  }

  ~FibonacciAction(void)
  {
  }

  void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1); //simulate compute time
    bool success = true;

    // clear and set first two values
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, we simulate compute time
      r.sleep();
    }

    if(success)           //se non c'è mai stato lo scenario in cui abbiamo ricevuto la preempted request, allora ha finito correttamente
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);          //pubblica il risultato sulla ros network, e setta lo stato come succeded 
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci");

  FibonacciAction fibonacci("fibonacci");
  ros::spin();

  return 0;
}
