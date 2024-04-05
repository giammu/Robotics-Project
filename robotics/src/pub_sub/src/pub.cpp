#include "ros/ros.h" //includiamo ros perchè lavoriamo con ros
#include "std_msgs/String.h" //includiamo la standard message string



int main(int argc, char **argv){ //lo standard main del c++ 

	ros::init(argc, argv, "talker"); //inizializza il nodo con init command e specifico il nome che voglio dare al nodo con un nome significativo "talker", devo dare nomi diversi ai diversi nodi
	//ros::init_options::AnonymousName);  //anonymousname lo starta con un nome formato da numeri random e quindi posso startarlo quante volte voglio
	ros::NodeHandle n; //creo il nodehandle

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1); //creo il publisher object: gli do un nome e chiamo il node handle n e chiamo la advertise function: vogliamo pubblicare uno string message, il topic è chatter, il buffer ha dimensione 1 (metterlo 1 è una good practice)
	//il buffer serve se ricevo più dati di quanti ne riesco a computare, però in quel caso significa che sto computando dei dati vecchi
	ros::Rate loop_rate(10); //il loop ha una frequenza di 10hz (ros vuole che facciamo così i loops)

	int count = 0; //qui non serve a niente

  	while (ros::ok()){ //ok è lo stesso di true, ma è meglio perchè fa dei check

	    	std_msgs::String msg; //creiamo il messaggio

                msg.data = "hello world!"; //se cerco su google std_msg, vedo che ha 1 solo field: popoliamo questo campo con la stringa

    		ROS_INFO("%s", msg.data.c_str()); //debugging messages inside the code: uso la funzione ROS_INFO al posto di cout perchè così il messaggio va anche dentro a ros, ci sono altre funzioni di ros
			//(ci sono anche ROS_ERROR, ROS_DEBUG)
    		chatter_pub.publish(msg); //pubblichiamo effettivamente i nostri data (gli passiamo il messaggio da pubblicare),

    		ros::spinOnce(); //spinOnce serve per: ros controlla tutte le callback nel codice. in questo caso non fa molto, ma in codici complessi ci sono delle actions da performare

    		loop_rate.sleep(); //faccio lo sleep per un certo tempo -> mi garantisce che il loop vada esattamente a 10hz come definito prima
    		++count;
  	}


  	return 0;
}
