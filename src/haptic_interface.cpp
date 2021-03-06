#include "ros/ros.h"
#include "joystick/haptic_polar.h"
#include "haptic_interface/obstacle.h"
#include "joystick/GetXYZ.h"
#include <complex>
#include <math.h>
#include <limits>
#include "std_msgs/Float32.h"

#define PI 3.14159265
#define SCALE_FACTOR 8192.0
#define HAPTIC_POLAR_NODE "haptic_polar"
#define MAX_STRENGTH 32767
#define MAX_VAL 500000.0
#define SHAKE_COUNT 15

const float MAX_TOLERANCE = 5.0;
const float def = (3.0*PI)/2.0;
const float delta = 40.0;

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
		shakeEnabled = false;
	}
	
	~Generator(){};

	void publishMessage(joystick::haptic_polar& msg);

	void generateFeedback(void);

	void updateLaserLeft(std_msgs::Float32 l);

	void updateLaserRight(std_msgs::Float32 r); 

	void enableShake(void);
	
	void disableShake(void);

private:
	ros::Publisher pub;
	int decision;
	EffectType effectType;
	float laserLeft;
	float laserRight;
	bool shakeEnabled;
};


void Generator::publishMessage(joystick::haptic_polar& msg){
	decision = decision == 1 ? -1 : 1;
	pub.publish(msg);
}

void Generator::generateFeedback(void){
	float d1 = this->laserLeft == 0.0 ? std::numeric_limits<float>::min() : this->laserLeft;
	float d2 = this->laserRight == 0.0 ? std::numeric_limits<float>::min() : this->laserRight;
	float t_d1 = std::min(d1,MAX_TOLERANCE);
	float t_d2 = std::min(d2,MAX_TOLERANCE);
	float sum = t_d1 + t_d2;
	//float diff = (std::min(d1, MAX_TOLERANCE)-std::min(d2, MAX_TOLERANCE));
	//diff *= PI/20.0;
	float diff = 0.25*PI*((t_d1/sum) - (t_d2/sum));
	float theta = def - diff;
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

	if((this->effectType == SHAKE) && (this->shakeEnabled == true)){
		int temp = msg.angle;
		for(int i = 0; i < SHAKE_COUNT; ++i){
			msg.angle = temp + (*this).decision*delta;
			this->publishMessage(msg);
		}
	}else{
		this->publishMessage(msg);
	}	
}

void Generator::updateLaserLeft(std_msgs::Float32 l){
	this->laserLeft = (float) l.data;
	this->generateFeedback();
}

void Generator::updateLaserRight(std_msgs::Float32 r){
	this->laserRight = (float) r.data;
	this->generateFeedback();
}

void Generator::enableShake(void){
	this->shakeEnabled = true;
}

void Generator::disableShake(void){
	this->shakeEnabled = false;
}

int main(int argc, char * argv[]){
	
	ros::init(argc, argv, "haptic_interface");

	ros::NodeHandle n;

	Generator g(n);

//	g.enableShake();
//	ROS_INFO("shake enabled");

	ros::Subscriber leftListener = n.subscribe("step_detect_left", 1000, &Generator::updateLaserLeft, &g);
	
	ros::Subscriber rightListener = n.subscribe("step_detect_right", 1000, &Generator::updateLaserRight, &g);

	ros::spin();

	return 0;
}
