#include <ros/ros.h>
#include "RO_srv/Arm_setAngles.h"
#include "RO_srv/Arm_home.h"
#include "RO_srv/Arm_engage.h"
#include "corobot_srvs/SetEngaged.h"
#include "corobot_srvs/SetPosition.h"

ros::ServiceClient servo_setEngaged;
ros::ServiceClient servo_setPosition;

bool armHomed = false;
bool armEngaged = false;
float theta0 = 1440.0f;
float theta1 = 1440.0f;
float theta2 = 1440.0f;
float theta3 = 1440.0f;

float theta0_min = -90.0f;
float theta0_max = 90.0f;
float theta1_min = -110.0f;
float theta1_max = 110.0f;
float theta2_min = -110.0f;
float theta2_max = 110.0f;
float theta3_min = -44.1f;
float theta3_max = 0.1f;

void home(){
	corobot_srvs::SetPosition srv;
	srv.request.index = 0;
        srv.request.position = 1440.0f;
        if(servo_setPosition.call(srv)){
                ROS_WARN("Servo Position Failed To Set");
        }srv.request.index = 1;
        srv.request.position = 1440.0f;
        if(servo_setPosition.call(srv)){
                ROS_WARN("Servo Position Failed To Set");
        }srv.request.index = 2;
        srv.request.position = 1440.0f;
        if(servo_setPosition.call(srv)){
                ROS_WARN("Servo Position Failed To Set");
        }srv.request.index = 3;
        srv.request.position = 1440.0f;
        if(servo_setPosition.call(srv)){
                ROS_WARN("Servo Position Failed To Set");
        }
	armHomed = true;
}

void engageArm(){
if(!armHomed){
 ROS_WARN("Arm not hommed when engaging automatically hommming");
 home();
}
	corobot_srvs::SetEngaged srv;
	srv.request.index = 0;
	srv.request.state = 1;
	servo_setEngaged.call(srv);
	srv.request.index = 1;
	servo_setEngaged.call(srv);
	srv.request.index = 2;
	servo_setEngaged.call(srv);
	srv.request.index = 3;
	servo_setEngaged.call(srv);
	armEngaged = true;

}

void disengageArm(){
	corobot_srvs::SetEngaged srv;
	srv.request.index = 0;
	srv.request.state = 0;
	servo_setEngaged.call(srv);
	srv.request.index = 1;
	servo_setEngaged.call(srv);
	srv.request.index = 2;
	servo_setEngaged.call(srv);
	srv.request.index = 3;
	servo_setEngaged.call(srv);
	armEngaged = false;
}

bool limitAngles(RO_srv::Arm_setAngles::Request &req){
	bool passed = true;
	if(!(req.theta0 >= theta0_min && req.theta0 <= theta0_max)){
		ROS_WARN("theta0 value request outside of limits");
		passed = false;
	}
	if(!(req.theta1 >= theta1_min && req.theta1 <= theta1_max)){
		ROS_WARN("theta1 value request outside of limits");
		passed = false;
	}
	if(!(req.theta2 >= theta2_min && req.theta2 <= theta2_max)){
		ROS_WARN("theta2 value request outside of limits");
		passed = false;
	}
	if(!(req.theta3 >= theta3_min && req.theta3 <= theta3_max)){
		ROS_WARN("theta3 value request outside of limits");
		passed = false;
	}
	return passed;	
}

bool setEngaged(RO_srv::Arm_engage::Request &req, RO_srv::Arm_engage::Response &res){
	if(armEngaged){
		ROS_INFO("Arm already engaged re-engaging?");
	}
	if(req.state){
		engageArm();
	}else{
		disengageArm();
	}	
	return false;
}

//This function should engage the arm and send it to zero position as safely as possible
bool homeArm(RO_srv::Arm_home::Request &req, RO_srv::Arm_home::Response &res){
	
	return false; //Not fully implemented yet
}

bool setAngles(RO_srv::Arm_setAngles::Request &req, RO_srv::Arm_setAngles::Response &res){
 	ROS_INFO("Arm_setAngles Called: %f %f %f %f",req.theta0, req.theta1,req.theta2,req.theta3);
	corobot_srvs::SetPosition srv;
	if(!armEngaged){
		ROS_WARN("Arm not engaged, automatically engaging");
		engageArm();
	}	
	if(limitAngles(req)){
	srv.request.index = 0;
        srv.request.position =-req.theta0*440.0f/90.0f+1440.0f;
        if(servo_setPosition.call(srv)){
                ROS_WARN("Servo Position Failed To Set");
        }
	
 	srv.request.index = 1;
        srv.request.position = req.theta1*440.0f/90.0f+1440.0f;
        if(servo_setPosition.call(srv)){
                ROS_WARN("Servo Position Failed To Set");
        }

 	srv.request.index = 2;
        srv.request.position = -req.theta2*440.0f/90.0f+1440.0f;
        if(servo_setPosition.call(srv)){
                ROS_WARN("Servo Position Failed To Set");
        }

	srv.request.index = 3;
	srv.request.position = req.theta3*440.0f/90.0f+1440.0f;
	if(servo_setPosition.call(srv)){
		ROS_WARN("Servo Position Failed To Set");
	}
	}
	return false; //Not fully implemlented yet
}


int main(int argc, char **argv){
	ros::init(argc,argv,"ArmDriver");	
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("Arm_setAngles",setAngles);
	ros::ServiceServer service2 = n.advertiseService("Arm_setEngaged",setEngaged);
	servo_setEngaged = n.serviceClient<corobot_srvs::SetEngaged>("phidgetServo_setEngaged");
	servo_setPosition = n.serviceClient<corobot_srvs::SetPosition>("phidgetServo_setPosition");
	ROS_INFO("Arm Driver Initialized");
	ros::spin();

	return 0;
}

