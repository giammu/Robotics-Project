#include "ros/ros.h"
#include "service/AddTwoInts.h"


int main(int argc, char **argv) // se voglio creare il servizio, devo chiamare un client
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<service::AddTwoInts>("add_two_ints"); //definiamo il client, il tipo di service è add_two_ints
  service::AddTwoInts srv; //creiamo il message
  srv.request.a = atoll(argv[1]); //settiamo i fields
  srv.request.b = atoll(argv[2]); //settiamo i fields
  if (client.call(srv)) //passiamo alla funzione call del client il nostro messaggio
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints"); //se non riesce a fare la computazione, semplicemente pubblica l'errore
    return 1;
  }

  return 0;
}
