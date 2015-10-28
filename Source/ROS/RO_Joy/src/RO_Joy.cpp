#include <ros/ros.h>
#include "RO_srv/Driver_setThrottle.h"
#include "RO_srv/Arm_setAngles.h"
#include "RO_srv/Arm_engage.h"
#include <sensor_msgs/Joy.h>

ros::ServiceClient driver_setThrottle;
ros::ServiceClient driver_setSteering;
ros::ServiceClient arm_setEngaged;
ros::ServiceClient arm_setAngles;
ros::Subscriber joy_sub;

bool timed_out = false;
bool should_engage = false;
float steering = 0.0f;
float throttle = 0.0f;

//Running values of angles
float t1 = 0.0f;
float t2 = 0.0f;
float t3 = 0.0f;

//State of gripper
float t4 = 0.0f;

//Max = m , Min  = n
float t1_m = 90.0f;
float t2_m = 110.0f;
float t3_m = 110.0f;
float t4_m = 0.0f;
float t1_n = -90.0f;
float t2_n = -110.0f;
float t3_n = -110.0f;
float t4_n = -44.0f;

//angular rates calibration (deg/s)
float d1 = 2.0f;
float d2 = 9.0f;
float d3 = 9.0f;

//stick deadband
float min_delta_s = 0.1f;

//Desired throttle values for arm joints
float s1 = 0.0f;
float s2 = 0.0f;
float s3 = 0.0f;
bool s4 = false;

ros::Timer timer;
ros::Time last_update;
ros::Duration timeout(3.0); //3 seconds

bool limitAngles(){
        bool passed = true;
        if(!(t1 >= t1_n && t1 <= t1_m)){
		if(t1 > t1_m){
			t1 = t1_m;
		}else{
			t1 = t1_n;
		}
                ROS_WARN("theta1 value request outside of limits");
                passed = false;
        }
        if(!(t2 >= t2_n && t2 <= t2_m)){
                if(t2 > t2_m){
			t2 = t2_m;
		}else{
			t2 = t2_n;
		}
		ROS_WARN("theta2 value request outside of limits");
                passed = false;
        }
        if(!(t3 >= t3_n && t3 <= t3_m)){
                if(t3 > t3_m){
			t3 = t3_m;
		}else{
			t3 = t3_n;
		}
		ROS_WARN("theta3 value request outside of limits");
                passed = false;
        }
        return passed;
}



void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	steering = joy->axes[0];
	throttle = joy->axes[1];
	
	s1 = joy->axes[3]*-1;
	s2 = joy->axes[4];
	s3 = (joy->axes[2] - joy->axes[5])/2.0f;
	if(joy->buttons[0] == 1){
		s4 = true;
		ROS_INFO("BTN A is true");
	}
	if(joy->buttons[1] == 1){
		s4 = false;
		ROS_INFO("BTN B is true");
	}
	ROS_INFO("Button State %f", s4);
	if(joy->buttons[2] == 1){
		t1 = 0.0f;
		t2 = 0.0f;
		t3 = 0.0f;
		ROS_INFO("BTN X is true");
	}
	if(joy->buttons[3] == 1){
		should_engage = true;
		ROS_INFO("Engage Arm Button Detected");
	}
	last_update = ros::Time::now();
	if(timed_out){
		timer.start();
		timed_out = false;
	}
}

void timerCallback(const ros::TimerEvent&){
	ros::Duration delta_t = ros::Time::now()-last_update;
	if(should_engage){
		RO_srv::Arm_engage eng;
		eng.request.state = true;
		arm_setEngaged.call(eng);
		should_engage = false;
	}
	if(delta_t > timeout){
		RO_srv::Driver_setThrottle srv;
		srv.request.throttle = 0.0f;
		driver_setThrottle.call(srv);
		srv.request.throttle = 0.0f;
		driver_setSteering.call(srv);
		ROS_WARN("RO_Joy: Timeout hit!");
		timed_out = true;
		timer.stop();
	}else{
		limitAngles();
		RO_srv::Driver_setThrottle srv;
		srv.request.throttle = throttle;
		driver_setThrottle.call(srv);
		srv.request.throttle = steering;
		driver_setSteering.call(srv);
		ROS_INFO("Throttle Set to %f", throttle);
		ROS_INFO("Steering Set to %f", steering);

		RO_srv::Arm_setAngles ang;
		t1 = t1+d1*s1;
		t2 = t2+d2*s2;
		t3 = t3+d3*s3;
		if(!s4){
			t4 = t4_m;
		}else{
			t4 = t4_n;
		}
				

		ang.request.theta0 = t1;
		ang.request.theta1 = t2;
		ang.request.theta2 = t3;
		ang.request.theta3 = t4;
		arm_setAngles.call(ang);
	}
}

int main(int argc, char **argv){
	ros::init(argc,argv,"RO_Joy");
	ros::NodeHandle n;
	arm_setAngles = n.serviceClient<RO_srv::Arm_setAngles>("Arm_setAngles");
	driver_setThrottle = n.serviceClient<RO_srv::Driver_setThrottle>("Driver_setThrottle");
	driver_setSteering = n.serviceClient<RO_srv::Driver_setThrottle>("Driver_setSteering");
	joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &joyCallback);
	arm_setEngaged = n.serviceClient<RO_srv::Arm_engage>("Arm_setEngaged");
	timer = n.createTimer(ros::Duration(0.1),timerCallback);
	ros::spin();
	return 0;
}
