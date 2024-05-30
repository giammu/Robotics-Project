# in questo file definisco le caratteristiche del robot: in particolare i sensori

define kinect ranger
(
  sensor
  (
    pose [ -0.1 0.0 -0.11 0.0 ]           
    size [0.1 0.1 0.1 ]
    range  [0 6.5]
    fov 120.0                              #qua definiamo le caratteristiche del laserscanner
    samples 640
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]                #dove inizia il robot
  
  localization "odom"                     #tipologia di 


  odom_error [0.2 0.2 0.0 0.3 ]                       #questi sono i parameters che generano la gaussian noise con la varianza (l'errore)

  size [ 0.2552 0.2552 0.40 ]                  #dimensione del robot
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"                      #tipologia di robot: differential drive
  color "grey"
  
  kinect()                           #integro il sensore dentro il robot
)
