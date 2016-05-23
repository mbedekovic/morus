# morus
ROS package for controlling PWM outputs of Pixhawk controller for ESC of brushless motors

Upotreba:

  - spojiti Pixhawk na napajanje preko BEC-a sa kabla glavnog napajanja 
  - osigurati da iskopčana +5 V linija servo priključaka
  - spojiti TELEM2 port preko FTDI/USB konvertera na računalo
  - poktrenuti roslaunch mavros px4.launch fcu_url:=/dev/ttyUSBx:921600 (USB0 ili USB1, provjeriti)
  - prije pokretanja offb_noda omogućit arming (prearming state) sa prekidačem tako da pritisnete i držite prekidač
    sve dok ne počne brže titrati (tri titraja pa pauza)
  -pokrenuti rosrun morus offb_node
    Kada kontroler uđe u ARMED stanje svijetlo na kontroleru i na prekdaču stalno svijetli 
  - po završetku rada OBAVEZNO pritisnuti i držati prekidač sve dok kontroler ne onemogući servo izlaze
    (tada prekidač pulsira kao i u prearm stanju)
  - nakon toga je sigurno ugasiti offb_node.

-----------------------------------------------------------------------------------------------------------
Node se pretplaćuje na topic:
  - motors
  - tip poruke: std_msgs/Float32MultiArray 
Node objavljue topic:
  - axrs
  - tip poruke: sensor_msgs/Imu
    - pri čemu je: orientation.x = roll kut | 
                  orientation.y = pitch kut | 
                   orientation.z = yaw kut | 
                  orientation.w = -2  | 
  
