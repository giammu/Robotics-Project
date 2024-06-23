# in questo file definisco le caratteristiche del robot: in particolare i sensori

define kinect ranger
(
  sensor
  (
    pose [ 0.0 0.0 0.0 0.0 ]           
    size [0.0 0.0 0.0 ]
    range  [0 6.5]
    fov 360.0                              #qua definiamo le caratteristiche del laserscanner
    samples 4320            #l'importante è che il numero di samples sia 360*qualcosa
  )
  # generic model properties
  color "black"
  size [ 0.0 0.0 0.0 ]
)

define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]                #dove inizia il robot
  
  localization "odom"                     #tipologia di 


  odom_error [0.0 0.0 0.0 0.0 ]                       #questi sono i parameters che generano la gaussian noise con la varianza (l'errore)

  size [ 0.612 0.580 0.0 ]                  #dimensione del robot
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "omni"                      #tipologia di robot
  color "grey"
  
  kinect()                           #integro il sensore dentro il robot
)
