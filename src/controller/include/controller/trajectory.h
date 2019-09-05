#ifndef _TRAJECTORY_RECORD_H_
#define _TRAJECTORY_RECORD_H_

class Trajectory {

public:
    uint32_t num;
    float x;
    float y;
    uint32_t status;
    float speed;

};

class Car{
public:
    float x;
    float y;
    float theta;
    float speed;
    float steeringWheelAngle;
    float throttle;
    float brake;


};


#endif 