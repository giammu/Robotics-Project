#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
  // Initialize the ROS node
  ros::init(argc, argv, "world_to_frleg_listener");

  // Create a ROS node handle
  ros::NodeHandle node;

  // Create a TransformListener object that will listen to tf data
  tf::TransformListener listener;   //simile a quello che avevo per il publishing (TransformBroadcaster)

  // Set the rate at which we want to check the transformation
  ros::Rate rate(10.0); //per questo esempio vogliamo prendere le informazioni a 10hz -> usiamo il while(ok) loop

  while (node.ok()){
    tf::StampedTransform transform; //creiamo questo per salvare l'informazione
          //devo fare 2 cose per avere la transformation
    try{
      // Wait for up to 1 second for the transform to become available
      listener.waitForTransform("/world", "/FRleg", ros::Time(0), ros::Duration(1.0)); //[prima cosa]: chiamo waitForTransform sul listener: è una chiamata bloccante che aspetta che la transformation sia disponibile, specifico che voglio bloccare per 1 secondo prima di tirare l'exception (devo controllare che il publisher non sia più lento del listener es. publisher a 10hz); poi settiamo la trasformazione tra world e frleg (posso quindi fare una trasformazine tra elementi che appartengono al tfTree e non per forza elementi che sono vicini). Usiamo ros Time 0: per dire quando la transformation deve essere pubblicata (il timestamp)

      // Look up the transformation from "world" to "FRleg"
      listener.lookupTransform("/world", "/FRleg", ros::Time(0), transform); //[seconda cosa]: se arriva la transformation, posso proseguire alla lookupTransform: gli passo i due elementi, il timestamp al quale voglio pubblicare la transformation, e che voglio salvarlo dentro alla transformation 
    }
    catch (tf::TransformException &ex) { //questa è l'exception che viene tirata, di solito quando il waitForTransform termina dopo 1 secondo 
      // If there is an exception print the error message
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }


    // Print transformation (only pose, but you can get the orientation)
    ROS_INFO("Translation: x=%f, y=%f, z=%f", //qui posso stampare i diversi valori
              transform.getOrigin().x(),  
              transform.getOrigin().y(),
              transform.getOrigin().z());

    // Sleep 
    rate.sleep();
  }

  return 0;
}

