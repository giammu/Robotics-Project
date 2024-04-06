        //questo esempio mostra come si pubblica TF

#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include <tf/transform_broadcaster.h>

class tf_sub_pub{
public:
  	tf_sub_pub(){
  	sub = n.subscribe("/turtle1/pose", 1000, &tf_sub_pub::callback, this); //we subscribe to the position of the turtle e settiamo la callback
}

//tutto ciò che è in relazione col TF accade dentro alla callback
void callback(const turtlesim::Pose::ConstPtr& msg){ //pubblica sul topic Pose (è uno standard rostopic per la position), e viene ripubblicato indietro come un TF
  tf::Transform transform; //creiamo la trasformazione che vogliamo pubblicare: è un message di tipo TF
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0) ); //ha solo 2 fields: pubblichiamo la x e la y, poi 0 perchè non abbiamo z, con questo settiamo il primo elemento della transform: la position
  tf::Quaternion q; //creo il quaternion
  q.setRPY(0, 0, msg->theta); //setto roll, pitch che sono entrambi 0 perchè la tartaruga vive in un ambiente 2d, y è l'orientamento della turtle ed è l'unico valore che cambia
  transform.setRotation(q); //settiamo l'orientamento: gli passiamo il quaternion, con questo abbiamo settato il secondo field 
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "turtle")); //pubblichiamo la transformation: usiamo il transform broadcaster creato prima, però sendTransform non vuole uno stadard tf ma un StampedTransform: specifichiamo che la position del robot rispetto alla starting position ha questo valore ad un determinato istante di tempo -> StampedTransform aggiunge il timeStamp information. Gli passo la trasform(position e orientation) + il tempo + i due sistemi di riferimento: la turtle si trova in una certa posizione rispetto a world
}

private:
  ros::NodeHandle n;  //creo il nodehandle
  tf::TransformBroadcaster br; //questo è la versione specifica del tf of the publisher
  ros::Subscriber sub; //abbiamo un subscriber perchè vogliamo ricevere la posizione della turtle
};


int main(int argc, char **argv){
 ros::init(argc, argv, "subscribe_and_publish");
 tf_sub_pub my_tf_sub_bub;
 ros::spin();
 return 0;
}

