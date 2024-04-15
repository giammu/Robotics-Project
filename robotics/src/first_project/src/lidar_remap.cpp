/*
Terzo nodo: lidar data visualization: abbiamo 2 odometry e dobbiamo capire quale dei due è meglio

Il nodo si iscrive a /os_cloud_node/points e cambia il reference frame, che è definito nell'header
Il valore è regolato da un dynamic reconfigure callback che permette di cambiarlo dinamicamente tra wheel_odom o gps_odom
Alla fine pubblica sul topic /pointcloud_remapped 

Il nodo deve permettere all'utente di selezionare da rqt_reconfigure a quale tf si dovrebbe connettere il lidar

(Così facendo da rviz dovrei vedere la laserscan centrata sul tf dell'encoder o sul tf del gps odometry)

*/

int main(int argc, char **argv){
    
}
