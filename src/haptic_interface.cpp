#include "ros/ros.h"
#include "joystick/haptic_polar.h"
#include "haptic_interface/obstacle.h"
#include "joystick/GetXYZ.h"

#define RATE 1

class Complex{
public: 
	Complex(float r = 0, float i = 0) : r(r), i(i){};
	~Complex(void){};
private:
	float r;
	float i; 
};

void generateFeedback(const haptic_interface::obstacle& d){
		
}

int main(int argc, char * argv[]){
	
	ros::init(argc, argv, "haptic_interface");

	ros::NodeHandle n;

	/*Publiser for publishing polar feedback to the joystick*/
	ros::Publisher pub = n.advertise<joystick::haptic_polar>("haptic_polar", 1000);

	ros::Subscriber obstacleListener = n.subscribe("l_s_d", 1000, generateFeedback);

	ros::Rate loop(RATE);

	int angle = 0;
	while(ros::ok()){
		joystick::haptic_polar msg;
		angle = rand()%360;
		msg.strength = 32767;
		msg.angle = angle;
		pub.publish(msg);
		loop.sleep();		
	}

	return 0;
}
