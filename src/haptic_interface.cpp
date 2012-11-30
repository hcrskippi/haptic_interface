#include "ros/ros.h"
#include "joystick/haptic_polar.h"
#include "haptic_interface/obstacle.h"
#include "joystick/GetXYZ.h"
#include <complex>
#include <math.h>

#define PI 3.14159265
#define SCALE_FACTOR 2048.0
#define HAPTIC_POLAR_NODE "haptic_polar"
#define MAX_STRENGTH 32767


const float def = (3.0*PI)/2.0;
const float delta = 15.0;

typedef enum EffectType{
	DEFAULT,
	SHAKE
}EffectType;

class Generator{
public:
	Generator(ros::NodeHandle& n){
		/*Publiser for publishing polar feedback to the joystick*/
		pub = n.advertise<joystick::haptic_polar>(HAPTIC_POLAR_NODE, 1000);
		effectType = DEFAULT;
		decision = 1;
	}
	
	~Generator(){};

	void publishMessage(joystick::haptic_polar& msg);

	void generateFeedback(const haptic_interface::obstacle& d);

private:
	ros::Publisher pub;
	int decision;
	EffectType effectType;
};


void Generator::publishMessage(joystick::haptic_polar& msg){
	decision = decision == 1 ? -1 : 1;
	pub.publish(msg);
}

void Generator::generateFeedback(const haptic_interface::obstacle& d){
	float d1 = d.d1;
	float d2 = d.d2;
	float theta = def - (d1-d2);
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

int main(int argc, char * argv[]){
	
	ros::init(argc, argv, "haptic_interface");

	ros::NodeHandle n;

	Generator g(n);

	ros::Subscriber obstacleListener = n.subscribe("l_s_d", 1000, &Generator::generateFeedback, &g);

	ros::spin();

	return 0;
}
