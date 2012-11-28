#include "ros/ros.h"
#include "joystick/haptic_polar.h"
//#include "haptic_interface/obstacle.h"
#include "joystick/GetXYZ.h"

int main(int argc, char * argv[]){
	
	ros::init(argc, argv, "haptic_interface");

	ros::NodeHandle n;

	/*Publiser for publishing polar feedback to the joystick*/
	ros::Publisher pub = n.advertise<joystick::haptic_polar>("haptic_polar", 1000);

	//ros::Subscriber obstacleListener = n.subscribe("");

	ros::Rate loop(1);

	int angle = 0;
	while(ros::ok()){
		joystick::haptic_polar msg;
		angle = rand()%360;
		msg.strength = 32767;
		msg.angle = angle;
		pub.publish(msg);
		//ros::spinOnce();
		loop.sleep();		
	}

	return 0;
}
