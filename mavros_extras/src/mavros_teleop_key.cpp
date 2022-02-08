// This keyboard tramsmitter works better with gazebo


/* // TODO : Ideas:
 1. I still need to give the vehicle the hover thrust,
 the best way to do this is from PX4 side and not here,
 I will need to make a new ROS topic, publish it here (publish attitude only) and
 subscribe to it from PX4 side. Then inside PX4 merge this message input (attitude only)
 with the hover thrust uORB message that is enternally in PX4 from the hovering module.
 and then give this setpoint to the PX4:
 set_attitude_target(...) which is in line 270 in setpoint_raw.cpp module
 (I need to figure things out)

 2. integrate this with the object detection module using mavros
 */

#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2/transform_datatypes.h"

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_DD 0x64
#define KEYCODE_S 0x73

#define RAD2DEG  180/3.14;
#define DEG2RAD  3.14/180;

struct eulers{
  float phi;
  float theta;
  float psi;
};

eulers angles;
float dcm[3];
float q[4];
tf2::Quaternion myQuaternion;
float return_threshold = 0.01;

class TeleopMavros
{
public:
  TeleopMavros();
  void keyLoop();
  void watchdog();

private:

  ros::NodeHandle nh_;
  double thrust_, rudder_, aileron_, elevator_, t_scale_, r_scale_, a_scale_, e_scale_;
  //float phi, theta, psi;
  mavros_msgs::AttitudeTarget att;
  mavros_msgs::AttitudeTarget back_att;
  boost::mutex publish_mutex_;
  ros::Time first_publish_;
  ros::Time last_publish_;

  ros::Publisher att_pub_;

};

TeleopMavros::TeleopMavros():
  thrust_(0),
  rudder_(0),
  aileron_(0),
  elevator_(0),

  t_scale_(1.0),
  r_scale_(2.0),
  a_scale_(2.0),
  e_scale_(2.0)
{
    back_att.thrust = 0.491;
    back_att.orientation.x = 0;
    back_att.orientation.y = 0;
    back_att.orientation.z = 0;
    back_att.orientation.w = 1;

    thrust_=rudder_=aileron_=elevator_=0;

  att_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_mavros");
  TeleopMavros teleop_mavros;
  ros::NodeHandle n;

  signal(SIGINT,quit);

  boost::thread my_thread(boost::bind(&TeleopMavros::keyLoop, &teleop_mavros));

  // TODO: check the timer and the hz
  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&TeleopMavros::watchdog, &teleop_mavros));

  ros::spin();
  my_thread.interrupt() ;
  my_thread.join() ;

  //teleop_mavros.keyLoop();

  return(0);
}

void modify_att(mavros_msgs::AttitudeTarget *att, double *elevator_, double *aileron_, double *rudder_)
{

  double ph,th,ps;
  tf2::Matrix3x3(myQuaternion).getRPY(ph,th,ps);

  angles.phi = (float) ph;
  angles.theta = (float) th;
  angles.psi = (float) ps;

  if(angles.theta > return_threshold){
    angles.theta -= 0.04;
    *aileron_ -= 0.04;
  } else if (angles.theta < -return_threshold){
    angles.theta += 0.04;
    *aileron_ += 0.04;
  }

  if(angles.phi > return_threshold){
    angles.phi -= 0.04;
    *elevator_ -= 0.04;
  } else if (angles.phi < -return_threshold){
    angles.phi += 0.04;
    *elevator_ += 0.04;
  }
/*
  if(angles.psi > return_threshold){
    angles.psi -= 0.04;
  } else if(angles.psi < -return_threshold){
    angles.psi += 0.04;
  }
 */
  myQuaternion.setRPY(angles.phi,angles.theta,angles.psi);
  myQuaternion.normalize();
  att->orientation.x = myQuaternion[0];
  att->orientation.y = myQuaternion[1];
  att->orientation.z = myQuaternion[2];
  att->orientation.w = myQuaternion[3];

}

void TeleopMavros::watchdog()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  if ((ros::Time::now() > last_publish_ + ros::Duration(0.15))){
  //&& (ros::Time::now() > first_publish_ + ros::Duration(0.30))){
      modify_att(&att, &elevator_, &aileron_, &rudder_);
      att_pub_.publish(att);
  }

}

void check_limits(double* thrust_, double* elevator_, double* aileron_, double* rudder_){

  if (*thrust_ > 0.1){
    *thrust_ = 0.1;
  } else if(*thrust_ < -0.1){
    *thrust_ = -0.1;
  }


  *elevator_ = *elevator_ * RAD2DEG;
  *aileron_  = *aileron_ * RAD2DEG;
  *rudder_   = *rudder_ * RAD2DEG;

  if (*elevator_ > 25){
    *elevator_ = 25;
  } else if(*elevator_ < -25){
    *elevator_ = -25;
  }

  if (*aileron_ > 25){
    *aileron_ = 25;
  } else if(*aileron_ < -25){
    *aileron_ = -25;
  }

  *elevator_ = *elevator_ * DEG2RAD;
  *aileron_  = *aileron_ * DEG2RAD;
  *rudder_   = *rudder_ * DEG2RAD;

/*
  if (*rudder_ > 45){
    *rudder_ = 45;
  } else if(*rudder_ < -45){
    *rudder_ = -45;
  }
 */
}

void TeleopMavros::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);

  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the drone.");
  puts("your options: up/down/left/right/w/s/a/d/q");

  int i= 0;

  while (true)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    { // report error
      perror("read():");
      exit(-1);
    }

    printf("value: 0x%02X\n", c);

    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("aileron LEFT");
        aileron_ -= 0.05;
        //dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("aileron RIGHT");
        aileron_ += 0.05;
        //dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("elevator UP");
        elevator_ += 0.01;
        //dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("elevator DOWN");
        elevator_ -= 0.01;
        //dirty = true;
        break;
      case KEYCODE_W:
        ROS_DEBUG("thrust more");
        thrust_ += 0.005;
        //dirty = true;
        break;
      case KEYCODE_S:
        ROS_DEBUG("thrust less");
        thrust_ -= 0.005;
        //dirty = true;
        break;
      case KEYCODE_A:
        ROS_DEBUG("rudder left");
        rudder_ -= 0.05;
        //dirty = true;
        break;
      case KEYCODE_DD:
        ROS_DEBUG("rudder right");
        rudder_ += 0.05;
        //dirty = true;
        break;
    }

    att.header.seq = 0;
    att.header.stamp = {0,0}; // {secs,nsecs}
    att.header.frame_id= "";
    att.type_mask = 0;// 7; // ignore body rates

    att.body_rate.x=att.body_rate.y=att.body_rate.z=0;

    check_limits(&thrust_, &elevator_, &aileron_, &rudder_);

    angles.phi = elevator_;
    angles.theta = aileron_;
    angles.psi = rudder_;

    myQuaternion.setRPY(angles.phi,angles.theta,angles.psi);
    myQuaternion.normalize();
    att.orientation.x = myQuaternion[0];
    att.orientation.y = myQuaternion[1];
    att.orientation.z = myQuaternion[2];
    att.orientation.w = myQuaternion[3];

    att.thrust = 0.70 + thrust_;

    boost::mutex::scoped_lock lock(publish_mutex_);
    last_publish_ = ros::Time::now();
    att_pub_.publish(att);

  }

  return;
}
