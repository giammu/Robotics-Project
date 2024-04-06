#include <ros/ros.h>

#include <dynamic_reconfigure/server.h> //includo la dynamic reconfigure perchè è quello che stiamo facendo 
#include <parameter_test/parametersConfig.h> // nome_package/"stringWroteInsideTheConfigurationFile" combined with "Config.h"

void callback(parameter_test::parametersConfig &config, uint32_t level) { //la callback function ha 2 parametri: config=la lista di tutti i values, level= cioè quale parametro cambio (quello che prima abbiamo definito con 0,1,2,3,4...). uso uint32_t perchè solo 1 parmetro può cambiare alla volta, e quindi è sufficiente
  ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
            config.int_param, config.double_param, //posso accedere e stampare ogni valore
            config.str_param.c_str(), 
            config.bool_param?"True":"False", 
            config.size);
            
            ROS_INFO ("%d",level); //se voglio vedere quale sta cambiando mi basta vedere il level
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "param_second"); //inizializzo il ros node

  dynamic_reconfigure::Server<parameter_test::parametersConfig> server; //prima creo il sever: il tipo del server deve essere parameter_test::parametersConfig, cioè nome del package::nome del config file
  dynamic_reconfigure::Server<parameter_test::parametersConfig>::CallbackType f; //qua creo la callback

  f = boost::bind(&callback, _1, _2); //then i use boost to bind the callback che ho appena definito (riga 20) con la funzione che si occupa di fare la callback (riga 6)
  server.setCallback(f); //gli passo la callback che ho appena creato

  ROS_INFO("Spinning node");
  ros::spin(); //il nodo continua ad andare in attesa che succeda qualcosa sulla dynamic reconfigure, triggerando la callback
  return 0;
}
