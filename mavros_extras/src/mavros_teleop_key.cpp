#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_DD 0x64
#define KEYCODE_S 0x73

struct eulers{
  float phi;
  float theta;
  float psi;
};

class TeleopMavros
{
public:
  TeleopMavros();
  void keyLoop();
  void watchdog();

private:

  ros::NodeHandle nh_;
  double thrust_, rudder_, aileron_, elevator_, t_scale_, r_scale_, a_scale_, e_scale_;
  float q[4];
  float phi, theta, psi;
  float return_threshold = 0.01;
  eulers angles;
  mavros_msgs::AttitudeTarget att;
  mavros_msgs::AttitudeTarget back_att; // TODO
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
  //nh_.param("scale_thrust", t_scale_, t_scale_);
  //nh_.param("scale_rudder", r_scale_, r_scale_);
  //nh_.param("scale_aileron", a_scale_, a_scale_);
  //nh_.param("scale_elevator", e_scale_, e_scale_);

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

void modify_att(mavros_msgs::AttitudeTarget *att){
  if (att->orientation.w > 0.1){
    att->orientation.w -= 0.02;
  } else if(att->orientation.w < -0.1){
    att->orientation.w += 0.02;
  }

/*   if(aileron_ > return_threshold){
    aileron_ -= 0.02;
  } else if (aileron_ < -return_threshold){
    aileron_ += 0.02;
  }
  if(elevator_ > return_threshold){
    elevator_ -= 0.02;
  } else if (elevator_ < -return_threshold){
    elevator_ += 0.02;
  }
  if(rudder_ > return_threshold){
    rudder_ -= 0.02;
  } else if(rudder_ < -return_threshold){
    rudder_ += 0.02;
  } */
}

void TeleopMavros::watchdog()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  if ((ros::Time::now() > last_publish_ + ros::Duration(0.15))){
  //&& (ros::Time::now() > first_publish_ + ros::Duration(0.30))){
      //modify_att(&att);
      att_pub_.publish(back_att); //TODO
  }

}

void euler_to_quat(float *q, eulers* eul){
  // "Quaternion from (body 3(psi)-2(theta)-1(phi) euler angles"

  float s1 = sinf(eul->psi/2);
  float c1 = cosf(eul->psi/2);
  float s2 = sinf(eul->theta/2);
  float c2 = cosf(eul->theta/2);
  float s3 = sinf(eul->phi/2);
  float c3 = cosf(eul->phi/2);

  q[0] = s1*s2*s3 + c1*c2*c3;
  q[1] = -s1*s2*c3 + s3*c1*c2;
  q[2] = s1*s3*c2 + s2*c1*c3;
  q[3] = s1*c2*c3 - s2*s3*c1;
}

void check_limits(mavros_msgs::AttitudeTarget *att, eulers *e, double* thrust){
  if (att->thrust > 0.9){
    att->thrust = 0.9;
    *thrust = 0.9;
  } else if(att->thrust < 0.1){
    att->thrust = 0.1;
    *thrust = 0.1;
  }

  if (e->phi > 45){
    e->phi = 45;
  } else if(e->phi < -45){
    e->phi = -45;
  }

  if (e->theta > 45){
    e->theta = 45;
  } else if(e->theta < -45){
    e->theta = -45;
  }

/*   if (e->psi > 180){
    e->psi = 180;
  } else if(e->psi < -180){
    e->psi = -180;
  } */

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

  thrust_=rudder_=aileron_=elevator_=0;
  int i= 0;

  while (true)
  {
    i++;
    printf("In the loop : %d \n", i);

    printf("1 \n");

    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    { // report error
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);

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
        thrust_ += 0.05;
        //dirty = true;
        break;
      case KEYCODE_S:
        ROS_DEBUG("thrust less");
        thrust_ -= 0.05;
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
    angles.phi = elevator_;
    angles.theta = aileron_;
    angles.psi = rudder_;
    att.body_rate.x=att.body_rate.y=att.body_rate.z=0;
    att.thrust = thrust_;

    check_limits(&att, &angles, &thrust_);

    euler_to_quat(&q[0],&angles);
    att.orientation.x = q[1];
    att.orientation.y = q[2];
    att.orientation.z = q[3];
    att.orientation.w = q[0];

    boost::mutex::scoped_lock lock(publish_mutex_);
    last_publish_ = ros::Time::now();
    att_pub_.publish(att);

  }

  return;
}

/* void publish(mavros_msgs::AttitudeTarget att){

} */
