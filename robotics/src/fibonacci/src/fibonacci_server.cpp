#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h> //includiamo il generic ros action server
#include <actionlib_tutorials/FibonacciAction.h> //e anche quello generato dal .action file nel folder action

class FibonacciAction
{
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_; //il nostro actionlib server: specifico la mia implementazione che è la fibonacci action
  std::string action_name_;
  // create messages that are used to published feedback/result
  actionlib_tutorials::FibonacciFeedback feedback_; //definisco il feedback message
  actionlib_tutorials::FibonacciResult result_;//               e il result message (sono custom messages che per coincidenza vengono usati anche dentro al server)

public:

  FibonacciAction(std::string name) :
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false), //inizializzo il server, boost serve per collegarlo alla callback, lo inizializzo a false così non parte automaticamente
    action_name_(name)
  {
    as_.start(); //starto manualmente il server
  }

  ~FibonacciAction(void)
  {
  }

  void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr &goal) //il processing avviene dentro alla callback: riceve il goal
  {
    // helper variables
    ros::Rate r(1); //simulate compute time: vogliamo simulare che la computazione impiega del tempo
    bool success = true;

    // clear and set first two values
    feedback_.sequence.clear();           //feedback è il messaggio che vogliamo inviare periodicamente
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]); //order è un field del goal

    // start executing the action
    for(int i=1; i<=goal->order; i++)           //qui inizia la computazione
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())       //prima di computare faccio un controllo: 1)se il client chiede al server di fermarsi, 2) e se ros non è ok
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();   //setto il server come preempted
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);         //voglio periodicamente aggiornare lo stato
      // this sleep is not necessary, we simulate compute time
      r.sleep();
    }

    if(success)           //se non c'è mai stato lo scenario in cui abbiamo ricevuto la preempted request, allora ha finito correttamente
    {
      result_.sequence = feedback_.sequence; //result è un campo del messaggio
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);          //Fa 2 cose: pubblica il risultato sulla ros network, e setta lo stato del server come succeded 
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci");

  FibonacciAction fibonacci("fibonacci"); //chiamo la fibonacci-action
  ros::spin();

  return 0;
}
