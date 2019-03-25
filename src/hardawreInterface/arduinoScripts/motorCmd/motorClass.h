#ifndef motorClass_h
#define motorClass_h


class motorClass{
public:
    motorClass(int pwmPin, int dirPin, int encPin, float gearRatio, float encCntsRev); 
    void setMotorVel(float vel);
    void setMotorPos(float pos);
    void clearEncoder(void);
    signed long readEnc(void);
    int vel_closedLoopController(void);
    int pos_closedLoopController(void);
    void logValues(void);
    void stopMotor(void);
    float errorPos = 0;
    void pos_on_off_controller(void);
    void log_on_off(void);
  
    float currentCommandv = 0;
    double MotorPos;
    double MotorVel = 0;
    double errorVel = 0;
    double desiredMotorVel = 0;
    signed long encodercount = 0;
    signed long encodercountPrev = 0;




private:
    //pins
    int _pwmPin;
    int _dirPin;
    int _encPin;
    float _gearRatio;
    float _encCntsRev;
    
    //timing
    unsigned long currentTime = 0;
    unsigned long prevTime = 0;
    float dt = 0; 

    //upkeep functions
    void storeOldVals(void);
    float motor_velocity_calc(void);
    float motor_position_calc(void);
    void calc_t(void);

    //Encoder
//    signed long encodercount = 0;

    //error vel
//    float errorVel = 0;
    float errorVelPrev = 0;
    float integratedVelError = 0;

    //velocity control efforts
    float pCommandv = 0;
    float dCommandv = 0;
    float iCommandv = 0;

    //error pos
    float errorPosPrev = 0;
    float integratedPosError = 0;

    //position control efforts
    float pCommandp = 0;
    float dCommandp = 0;
    float iCommandp = 0;
    float currentCommandp = 0;

    //Position Setpoint and state
    float desiredMotorPos = 0;
    //float MotorPos;

    //Position controller gains
    float Kpp = 5000.0;
    float Kdp = 0.0;
    float Kip = 0;//100000;
    
    //Velocity Setpoint and state
 //   float desiredMotorVel = 0;
//    float MotorVel = 0;

    //velocity controller gains
    float Kpv = 50;
    float Kdv = 0;
    float Kiv = 0.02;
//    float currentCommandv = 0;


    //velocity controller functions
    float vel_proportional_control(void);
    float vel_derivative_control(void);
    float vel_integral_control(void);

    //position controller functions
    float pos_proportional_control(void);
    float pos_derivative_control(void);
    float pos_integral_control(void);
    float tol = 0.07;

};


//Constants
const float MAX_PWM = 1000;
const float MIN_PWM = -1000;
const float on_off_tolerance = 0.07;

#endif
