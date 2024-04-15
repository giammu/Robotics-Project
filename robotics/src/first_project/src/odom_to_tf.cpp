/*
 Secondo nodo: odom_to_tf: si iscrive a odometry e pubblica su tf (è estremamente simile a quello visto durante il lab 5: cambio il nome/tipo dei topic)

Si iscrive all'odometry
    type: nav_msgs/Odometry
    topic name: input_odom (questo è un nome generico, bisogna usare il remapping per iscriversi al vero topic)

Prende come input input_odom e ha come parameters i due reference frame del tf (root_frame and child_frame)

Il nodo parte dal launch file, con topic remapping per l'input e pubblica parametri con il tf_broadcaster
Il launch file deve creare 2 istanze del nodo [N.B. devo scrivere un nodo generico]: 1) pubblica l'odometry dell'encoder, 2) pubblica l'odometry dal gps
Entrambi hanno lo stesso root = world, ma i child frame sono rispettivamente 1) wheel_odom 2)gps_odom (partono dalla stessa posizione ma probabilmente hanno posizioni diverse al passare del tempo)

Quello che devo fare: scrivere il nodo, metterlo nel launch file, configurarlo per partire 2 volte, iscriversi al correct odometry, publish the correct tf using the parameters to set the root and child frame
*/ 

int main(int argc, char **argv){
    
}