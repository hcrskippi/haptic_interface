#include "ros/ros.h"
#include "joystick/haptic_polar.h"
#include "haptic_interface/obstacle.h"
#include "joystick/GetXYZ.h"
#include <complex>
#include <math.h>

#define PI 3.14159265
#define SCALE_FACTOR 4096.0

const float def = (3.0*PI)/2.0;

ros::Publisher pub;

void generateFeedback(const haptic_interface::obstacle& d){
	float d1 = d.d1;
	float d2 = d.d2;
	float theta = def - (d1-d2);
	d1 = (1.0/d1);
	d2 = (1.0/d2);
	float R = d1+d2;
	std::complex<float> f(R*cos(theta), R*sin(theta));
	joystick::haptic_polar msg;
	msg.angle = (-1.0)*((180.0*arg(f))/PI);
	msg.strength = (int) (SCALE_FACTOR*abs(f));
	if(msg.strength > 32767){
		msg.strength = 32767;
	}
	msg.strength = 32767;
	pub.publish(msg);
}

int main(int argc, char * argv[]){
	
	ros::init(argc, argv, "haptic_interface");

	ros::NodeHandle n;

	/*Publiser for publishing polar feedback to the joystick*/
	pub = n.advertise<joystick::haptic_polar>("haptic_polar", 1000);

	ros::Subscriber obstacleListener = n.subscribe("l_s_d", 1000, generateFeedback);

	ros::spin();

	return 0;
}
