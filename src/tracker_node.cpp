#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <yolact_ros_msgs/area_and_center.h>
#include <tf/transform_datatypes.h>

#include <cmath>
//#include "pid_control.h"

#define PREDEFINED_TARGET_AREA (20000)
#define MAX_LON_RATE           (1.5)
#define MIN_LON_RATE           (-1.5)
#define MAX_LAT_RATE           (1)
#define MIN_LAT_RATE           (-1)

using namespace std;

mavros_msgs::State            current_state;
geometry_msgs::TwistStamped   vel; 
geometry_msgs::PoseStamped    pose;
mavros_msgs::PositionTarget   setpoint_raw;
mavros_msgs::AttitudeTarget   setpoint_att;
mavros_msgs::AttitudeTarget   target_att;
bool flag = 0;

class PIDImpl;
class PID
{
    public:

        PID( double dt, double max, double min, double Kp, double Kd, double Ki );

        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double setpoint, double pv );
        ~PID();

    private:
        PIDImpl *pimpl;
};


u_int64_t inline euclidean_norm_2d(u_int32_t a, u_int32_t b){ return hypot(a,b); }


PID lon_controller              = PID(0.1, MAX_LON_RATE, MIN_LON_RATE, 0.00025, 0.00001, 0);
PID lat_controller              = PID(0.1, MAX_LAT_RATE, MIN_LAT_RATE, 0.001, 0.00001, 0);
PID alt_controller              = PID(0.1, MIN_LAT_RATE, MIN_LAT_RATE, 0.0005, 0.00001, 0);

class PIDImpl
{
    public:
        PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
        ~PIDImpl();
        double calculate( double setpoint, double pv );

    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
};


PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki ){
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}

double PID::calculate( double setpoint, double pv ){
    return pimpl->calculate(setpoint,pv);
}

PID::~PID(){
    delete pimpl;
}

PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
}

double PIDImpl::calculate( double setpoint, double pv ){
    
    double error = setpoint - pv;

    double Pout = _Kp * error;

    _integral += error * _dt;
    double Iout = _Ki * _integral;

    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    double output = Pout + Iout + Dout;

    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}

void pose_stamped_cb(const geometry_msgs::PoseStamped::ConstPtr & msg){

    pose = *msg;

}

/*
void target_att_cb(const mavros_msgs::AttitudeTarget::ConstPtr & msg){

    target_att = *msg;

}
*/

void state_cb(const mavros_msgs::State::ConstPtr& msg){

    current_state = *msg;
}

void tracker_cb(const yolact_ros_msgs::area_and_center::ConstPtr & msg){

    flag = 1;
    u_int32_t current_area = msg->area;
    //u_int64_t current_norm = euclidean_norm_2d(msg->center_coordinate[0], msg->center_coordinate[1]);

    double lon_cntrl_out           = lon_controller.calculate(PREDEFINED_TARGET_AREA, current_area);
    float lat_cntrl_out            = lat_controller.calculate(320, msg->hoz_cc);
    float alt_cntrl_out            = alt_controller.calculate(240, msg->ver_cc); // not active for now

    //vel.twist.linear.x  += area_cntrl_out;
    //vel.twist.angular.z += center_cntrl_out;


    //setpoint_raw.header.stamp     = ros::Time::now();
    //setpoint_raw.header.frame_id  = "map";
    //setpoint_raw.type_mask        = 0b010111000011;
    //setpoint_raw.type_mask        = 2019;
    //setpoint_raw.coordinate_frame = 1;
    setpoint_raw.velocity.x       = lon_cntrl_out;
    setpoint_raw.velocity.y       = 0;
    setpoint_raw.velocity.z       = 0;

    setpoint_raw.position.z       = 3;
    setpoint_raw.yaw_rate         = lat_cntrl_out;      

    
/*    
    setpoint_att.header.stamp    = ros::Time::now();
    setpoint_att.header.frame_id = "map"; 
    setpoint_att.body_rate.x     = 0;
    setpoint_att.body_rate.y     = lon_cntrl_out;
    setpoint_att.body_rate.z     = lat_cntrl_out;
    setpoint_att.type_mask       = 128;
    setpoint_att.orientation.w   = target_att.orientation.w;
    setpoint_att.orientation.x   = target_att.orientation.x;
    setpoint_att.orientation.y   = target_att.orientation.y;
    setpoint_att.orientation.z   = target_att.orientation.z;     
    setpoint_att.thrust          = target_att.thrust;       

*/

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub    = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1, state_cb);

    ros::Subscriber area_and_center_sub = nh.subscribe<yolact_ros_msgs::area_and_center>
           ("/area_and_center", 1, tracker_cb);    
    
    ros::Subscriber pose_stamped = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, pose_stamped_cb);    

    //ros::Subscriber att_rate_sub  = nh.subscribe<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/target_attitude", 10, target_att_cb);

    ros::Publisher body_vel_pub  = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);

    //ros::Publisher att_rate_pub  = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);

    //ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
    //       ("mavros/setpoint_velocity/cmd_vel", 1);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //vel.header.stamp = ros::Time::now();
    //vel.header.frame_id = "map";

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    setpoint_raw.header.frame_id  = "map";
    setpoint_raw.type_mask        = 2019; // vx + vy + z + y_rate
    setpoint_raw.coordinate_frame = 1;

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        //vel.twist.angular.z = 0.1;
        //vel.twist.linear.y  = 0.4;
        //vel.twist.linear.z  = 0;

        if(flag){ body_vel_pub.publish(setpoint_raw);
                  ros::Duration(0.5).sleep();
                  flag = 0; }
	
        else{
         
        flag = 0;
        //setpoint_raw.header.stamp = ros::Time::now();
        //body_vel_pub.publish(setpoint_raw);
    	setpoint_raw.header.stamp     = ros::Time::now();
    	
    	setpoint_raw.velocity.x       = 0;
    	setpoint_raw.velocity.y       = 0;
    	setpoint_raw.velocity.z       = 0;

    	setpoint_raw.position.z       = 3;
    	setpoint_raw.yaw_rate         = 0;

        body_vel_pub.publish(setpoint_raw);

       }	

        //body_vel_pub.publish(setpoint_raw);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
