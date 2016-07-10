///This file includes the Controllers can be used with ArduCopter framework
///At this time, there are 2 available controllers:
/// 1/Original PID Controller: The original controller of ArduPilot
///                             using PID controller to controller the rate of change of Roll, Pitch, Yaw
/// 2/New PID Controller: Instead of controlling the rate of change, this controller using 
///                            directly Roll, Pitch, Yaw with PID controller
#ifndef MULTI_CONTROLLER_H
#define MULTI_CONTROLLER_H
enum Controllers
{
    Original_PID_Controller = 0,
    New_PID_Controller = 1,
    Simple_IB_Controller = 2, 
    IB_Controller = 3,
    Calibration = 4,
};
extern int8_t using_controller ;
extern float flag[7] ;
#endif
