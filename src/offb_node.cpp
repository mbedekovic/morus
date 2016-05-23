#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float32MultiArray.h>

double motor_lijevo = -1.0;
double motor_desno = -1.0;

sensor_msgs::Imu forward_attitude;
void attitude_callback(const sensor_msgs::Imu::ConstPtr& msg){
    forward_attitude = *msg;
    //Preform transforamtion from Quaternion to RPY angles

    //this creates the quaternion
    tf::Quaternion q(forward_attitude.orientation.x,
                     forward_attitude.orientation.y,
                     forward_attitude.orientation.z,
                     forward_attitude.orientation.w);
    //Construct rotation matrix from quaternion
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    //Get the roll, pitch, yaw angles from orientation matrix
    m.getRPY(roll, pitch, yaw);
    forward_attitude.orientation.x = roll;
    forward_attitude.orientation.y = pitch;
    forward_attitude.orientation.z = yaw;
    forward_attitude.orientation.w = -2.0;
}

void motors_callback(const std_msgs::Float32MultiArray msg){
    //std_msgs::Float32MultiArray msg = *m;
    motor_lijevo = 2*msg.data[0]-1;
    motor_desno = 2*msg.data[1]-1;

    //Safety checking of output range
    if (motor_lijevo > 1.0) motor_lijevo = 1.0;
    if (motor_lijevo < -1.0) motor_lijevo = -1.0;
    if (motor_desno > 1.0) motor_desno = 1.0;
    if (motor_desno < -1.0) motor_desno = -1.0;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

/*void shutdownHandler(int sig){

    ros::ServiceClient setModeClient = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    //Service client for arming
    ros::ServiceClient armingClient = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    //Signal it's time to stop
    breakTheMainLoop = true;
    //Disarm the outputs
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;
    armingClient.call(arm_cmd);
    ROS_INFO("DISARMING");
    //Switch to manual mode
    mavros_msgs::SetMode setManualMode;
    setManualMode.request.custom_mode = "MANUAL";                                       TODO: Prebaci sve ovo u klasu da se moze hendlat shuttdown request
    setModeClient.call(setManualMode);
    ROS_INFO("SWITCH TO MANUAL");
    ROS_INFO("SHUTTING DOWN THE NODE");

    //Call the shutdown procedure
    ros::shutdown();
}*/

mavros_msgs::ActuatorControl formControlMessage(double moto1, double moto2){
    uint8_t groupMix = 0;
    mavros_msgs::ActuatorControl m;
    m.header.stamp = ros::Time::now();
    m.group_mix = groupMix;
    m.controls[0] = moto1;
    m.controls[1] = moto2;
    m.controls[3] = 0.0;
    m.controls[4] = 0.0;
    m.controls[5] = 0.0;
    m.controls[6] = 0.0;
    m.controls[7] = 0.0;
    return m;
}




int main(int argc, char **argv){
    //ros::init(argc,argv,"offb_node",ros::init_options::NoSigintHandler);
    ros::init(argc,argv,"offb_node");
    ros::NodeHandle nh;
    //Signal shutdown handler
    //signal(SIGINT, shutdownHandler);
    //Subscriebe to estimated attitude messages /mavros/imu/data
    ros::Subscriber attitude = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data",10, attitude_callback);
    //Publisher to forward the recieved message at a rate of 50 Hz in the loop
    ros::Publisher attitudePublisher = nh.advertise<sensor_msgs::Imu>("axrs",10);
    //Service client for mode change
    ros::ServiceClient setModeClient = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    //Service client for arming
    ros::ServiceClient armingClient = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    //Publisher for direct PWM output control
    ros::Publisher actuatorControlPublisher = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control",1);
    //Subscrieber to FMU state topic
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    //Subscrieber to motors topic that creates motor control inputs
    ros::Subscriber motorsSub = nh.subscribe<std_msgs::Float32MultiArray>("motors",10,motors_callback);


    ros::Rate rate(50.0);
    ros::Time started = ros::Time::now();
    mavros_msgs::ActuatorControl ESCcontrol;
    double motor_1;
    double motor_2;



    while(ros::ok()){
    ros::spinOnce();
    //Set the timestamp on the outgoing message
    forward_attitude.header.stamp = ros::Time::now();
    //Send the message
    attitudePublisher.publish(forward_attitude);

    nh.param<double>("motor_1",motor_1,-1.0);
    nh.param<double>("motor_2",motor_2,-1.0);

    //Form a ESC control message and send it
    ESCcontrol = formControlMessage(motor_desno,motor_lijevo);  //<----- Promjeni na motro_lijevo i motor_desno
    actuatorControlPublisher.publish(ESCcontrol);


    //Enable offboard control mode after 3 seconds
    if ((current_state.mode != "OFFBOARD") &&
            (ros::Time::now()-started>ros::Duration(3.0))){
        mavros_msgs::SetMode setOffboardMode;
        setOffboardMode.request.custom_mode = "OFFBOARD";
        if(setModeClient.call(setOffboardMode) && setOffboardMode.response.success){
            ROS_INFO("OFFBOARD ENABLED");
            started = ros::Time::now(); //Save the time for arming wait period
        }
    }
    else {
        //Arm the wehicle after 3 seconds past setting the offboard mode
        if(!(current_state.armed) && (ros::Time::now()-started > ros::Duration(3.0))){
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;

            if(armingClient.call(arm_cmd) && arm_cmd.response.success){
                ROS_INFO("OUTPUT ARMED");
                started = ros::Time::now(); //To stop multiple attempts at arming the system rapidly
            }
        }
    }

    rate.sleep();
    }

  return 0;
}
