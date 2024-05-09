
Abbiamo creato un file di configurazione per rviz, di modo che lanciando rviz con il launch.launch non è necessario ogni volta dover aggiungere TF, /pointcloud_remapped e settare fixed frame: world.

Inoltre nel launch.launch abbiamo aggiunto anche il nodo rqt_reconfigure, appunto per non doverlo aprire manualmente ogni volta.
Nel launch file non abbiamo aggiunto il comando per aprire il file bag perchè la posizione del file bag potrebbe cambiare, dato che non si trova all'interno della cartella del progetto.

Nel primo nodo gps_to_odom abbiamo ruotato le coordinate di 130° perchè così facendo i dati pubblicati su /gps_odom sono simili a quelli di /wheel_odom, almeno per i primi secondi.
Il calcolo dei gradi è stato fatto in questo modo:
-leggendo da /odom le posizioni x e y e calcolando l'angolo alpha=atan(y/x)
-calcolando le posizioni x e y in ENU lette da /fix e calcolando l'angolo beta=atan(y/x)
-infine calcolando delta=alpha-beta abbiamo ottenuto la differenza di angolo tra le due odometrie

Sulle nostre macchine il progetto sembra funzionare correttamente

