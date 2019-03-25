#include "motorClass.h"

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include "DualMC33926MotorShield.h"


// Robotzone motor
float gearRatio = 27;
float EncCntsPerRev = 48.0;

// create motor object
motorClass balMotor =  motorClass(3,4,8,gearRatio,EncCntsPerRev);
int controlType = 1; //0 = pos, 1 = vel

// gripper stuff
DualMC33926MotorShield md;
int gripperSpeed = 100;

void commandsCallback(const std_msgs::Float32MultiArray& commands)
{
    // motor command
    balMotor.setMotorVel(commands.data[0]);

    // gripper command...if +1, gripper will close. if -1, gripper will close. 
    md.setM1Speed(gripperSpeed*commands.data[1]);
}

// ros stuff
ros::NodeHandle arduino2Motor;

// subscriber
ros::Subscriber<std_msgs::Float32MultiArray> commands2ArduinoSub("commands2Arduino", &commandsCallback);

// publisher
std_msgs::Float32MultiArray ard2Control;
ros::Publisher ard2ControlPub("ard2Control", &ard2Control);

// encoder position offset
int encPosOffset = -999;

void setup () 
{ 
  // initialize ros
  arduino2Motor.initNode();
  
  // subscriber
  arduino2Motor.subscribe(commands2ArduinoSub);
  
  // publisher
  ard2Control.layout.dim[0].size = 2;
  ard2Control.data = (float *)malloc(sizeof(float)*2);
  ard2Control.data_length = 2;
  arduino2Motor.advertise(ard2ControlPub);

  // gripper
  md.init();
  
  delay(1000);

}


void loop ()
{
  // control motor
  balMotor.vel_closedLoopController();
 

  // publish encoder position and limit switches to ros
  ard2Control.data[0] = balMotor.encodercount;
  ard2Control.data[1] = balMotor.encodercount; // just in case we want something else

  ard2ControlPub.publish(&ard2Control);

  arduino2Motor.spinOnce();
}
