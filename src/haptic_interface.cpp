#include "ros/ros.h"
#include "joystick/haptic_polar.h"
#include "haptic_interface/obstacle.h"
#include "joystick/GetXYZ.h"
#include <complex>
#include <math.h>
#include "std_msgs/Float32.h"

#define PI 3.14159265
#define SCALE_FACTOR 2048.0
#define HAPTIC_POLAR_NODE "haptic_polar"
#define MAX_STRENGTH 32767
#define MAX_VAL 500000.0
#define MAX_TOLERANCE 5.0

const float def = (3.0*PI)/2.0;
const float delta = 15.0;

typedef enum EffectType{
	DEFAULT,
	SHAKE
}EffectType;

class Generator{
public:
	Generator(ros::NodeHandle& n) : laserLeft(MAX_VAL), laserRight(MAX_VAL){
		/*Publiser for publishing polar feedback to the joystick*/
		pub = n.advertise<joystick::haptic_polar>(HAPTIC_POLAR_NODE, 1000);
		effectType = DEFAULT;
		decision = 1;
	}
	
	~Generator(){};

	void publishMessage(joystick::haptic_polar& msg);

	void generateFeedback(void);

	void updateLaserLeft(std_msgs::Float32 l);

	void updateLaserRight(std_msgs::Float32 r); 

private:
	ros::Publisher pub;
	int decision;
	EffectType effectType;
	float laserLeft;
	float laserRight;
};


void Generator::publishMessage(joystick::haptic_polar& msg){
	decision = decision == 1 ? -1 : 1;
	pub.publish(msg);
}

void Generator::generateFeedback(void){
	float d1 = this->laserLeft;
	float d2 = this->laserRight;
	float theta = def - (std::min(d1, MAX_TOLERANCE)-std::min(d2, MAX_TOLERANCE));
	d1 = (1.0/(d1*d1));
	d2 = (1.0/(d2*d2));
	float R = d1+d2;
	std::complex<float> f(R*cos(theta), R*sin(theta));
	joystick::haptic_polar msg;
	msg.angle = (-1.0)*((180.0*arg(f))/PI);
	msg.strength = (int) (SCALE_FACTOR*abs(f));
	if(msg.strength > MAX_STRENGTH){
		msg.strength = MAX_STRENGTH;
	}

	if(msg.strength >= MAX_STRENGTH){
		this->effectType = SHAKE;	
	}else{
		this->effectType = DEFAULT; 
	}

	/*if(this->effectType == SHAKE){
		msg.angle = msg.angle + (*this).decision*delta;
	}*/

	this->publishMessage(msg);	
}

void Generator::updateLaserLeft(std_msgs::Float32 l){
	this->laserLeft = (float) l.data;
	this->generateFeedback();
}

void Generator::updateLaserRight(std_msgs::Float32 r){
	this->laserRight = (float) r.data;
	this->generateFeedback();
}

int main(int argc, char * argv[]){
	
	ros::init(argc, argv, "haptic_interface");

	ros::NodeHandle n;

	Generator g(n);

	ros::Subscriber leftListener = n.subscribe("step_detect_left", 1000, &Generator::updateLaserLeft, &g);
	
	ros::Subscriber rightListener = n.subscribe("step_detect_right", 1000, &Generator::updateLaserRight, &g);

	ros::spin();

	return 0;
}
