/*
Descrizione Terzo nodo: lidar data visualization: abbiamo 2 odometry e dobbiamo capire quale dei due è meglio

Il nodo si iscrive a /os_cloud_node/points e cambia il reference frame, che è definito nell'header
Il valore è regolato da un dynamic reconfigure callback che permette di cambiarlo dinamicamente tra wheel_odom o gps_odom
Alla fine pubblica sul topic /pointcloud_remapped 

Il nodo deve permettere all'utente di selezionare da rqt_reconfigure a quale tf si dovrebbe connettere il lidar

(Così facendo da rviz dovrei vedere la laserscan centrata sul tf dell'encoder o sul tf del gps odometry)

*/

#include "sensor_msgs/PointCloud2.h"
#include "string.h"
#include <sstream>

#include "ros/ros.h"
#include <dynamic_reconfigure/server.h> //includo la dynamic reconfigure 
#include <first_project/parametersConfig.h> // nome_package/"stringWroteInsideTheConfigurationFile" combined with "Config.h"


class LidarRemap
{
    private:
        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher pub;
        dynamic_reconfigure::Server<first_project::parametersConfig> server;
        sensor_msgs::PointCloud2 message;
        first_project::parametersConfig configurazione;

    public: LidarRemap(){
            sub = n.subscribe("/os_cloud_node/points", 1, &LidarRemap::callback, this);
            pub = n.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1);

            dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f;
            f = boost::bind(&LidarRemap::reconfigureCallback, this, _1, _2);
            server.setCallback(f);
        }

        void callback(const sensor_msgs::PointCloud2& msg)
        {
            message = msg;

            message.header.stamp = ros::Time::now(); //corregge l'errore in rviz dei messaggi scartati perchè troppo vecchi

            //ROS_INFO("Old frame: %s", msg.header.frame_id.c_str());

            message.header.frame_id = configurazione.header;

            //ROS_INFO("New frame: %s", message.header.frame_id.c_str());

            pub.publish(message);
        }

        void reconfigureCallback(first_project::parametersConfig &config, uint32_t level)
        {
            //ROS_INFO("New Header: %s, level %d", config.header.c_str(), level);

            configurazione.header = config.header;
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_remap");
    LidarRemap my_lidar_remap;
    ros::spin();
    return 0;
}
