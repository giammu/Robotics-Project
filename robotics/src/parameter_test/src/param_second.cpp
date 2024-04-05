#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <parameter_test/parametersConfig.h>

void callback(parameter_test::parametersConfig &config, uint32_t level) { //la callback function ah 2 parametri: config ... e level: cioè quale parametro cambio
  ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
            config.int_param, config.double_param, 
            config.str_param.c_str(), 
            config.bool_param?"True":"False", 
            config.size);
            
            ROS_INFO ("%d",level);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "param_second"); //inizializzo il ros node

  dynamic_reconfigure::Server<parameter_test::parametersConfig> server; 
  dynamic_reconfigure::Server<parameter_test::parametersConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin(); //il nodo continua ad andare in attesa che succeda qualcosa, chiamando la callback
  return 0;
}
