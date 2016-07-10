///This is the main file implementing the new controllers
///The Switch command is the main function, which will switch
///around controller depending on the chosen controller
/* New functions for a new method to calculate output pwm
/* ENSMA
/* Modified by NGUYEN Anh Quang
/*
/* Last update 23/4/2015
/*
/**/
#include "Copter.h"
#define X 0
#define Y 1
#define Z 2

#define ROLL 0
#define PITCH 1
#define YAW 2

t_MotorSpeed_io io_motors;

///Define variables for each controller

//new PID
//IO structure
t_PIDcontroller_io io;
//State structure
t_PIDcontroller_state state;

//Simple IB
//IO structure
t_IB_Controller_ga_attitude_final_io io_attitude;
t_Altitude_io io_alt;
//State structure
t_IB_Controller_ga_attitude_final_state state_attitude;
t_Altitude_state state_alt;


double motor_omega[4] = {0,0,0,0};
double time_old = 0;
GAREAL IB_results[4];

void Copter::init_controller() {
	//new PID initial
	PIDcontroller_init(&state);
	//simple IB initial
	IB_Controller_ga_attitude_final_init(&state_attitude);
	Altitude_init(&state_alt);
	//full IB initial
	//IB_ga_final_init(&state_ib);

}

///Declaration to make the program happy
///PID controller main code
GAREAL* Copter::PID_calculate() {
	double time = (double) millis() / 1000.0;
	float roll = ahrs.roll;
	float pitch = ahrs.pitch;
	/*
	 * Although these can be modified mid-flight with MAVlink packets,
	 * Note that there are strange domain restrictions, particulary in the
	 * roll, pitch and yaw derivatives.
	 *
	 * The explanation for the domain restrictions is that
	 * these are the parameters for the original PID controller
	 * and they have been stolen for use on the new PID controller.
	 */
	/*
	*/

	float Kp_z = g.pid_kp_throttle;//g.pid_accel_z.kP() * 3.0 ; // modified by TRAN TRUNG DUC-2016
	float Ki_z = g.pid_ki_throttle;//g.pid_accel_z.kI() / 10.0;
	float Kd_z = g.pid_kd_throttle;//g.pid_accel_z.kD();

	float Kp_phi = g.pid_kp_roll;//g.pid_rate_roll.kP();
	float Ki_phi = g.pid_ki_roll;//g.pid_rate_roll.kI();
	float Kd_phi = g.pid_kd_roll;//g.pid_rate_roll.kD() + 2.0;

	float Kp_theta = g.pid_kp_pitch;//g.pid_rate_pitch.kP();
	float Ki_theta = g.pid_ki_pitch;//g.pid_rate_pitch.kI();
	float Kd_theta = g.pid_kd_pitch;//g.pid_rate_pitch.kD() + 2.0;

	float Kp_psi = g.pid_kp_yaw;//g.pid_rate_yaw.kP();
	float Ki_psi = g.pid_ki_yaw;//g.pid_rate_yaw.kI();
	float Kd_psi = g.pid_kd_yaw;//g.pid_rate_yaw.kD() + 2.0;

	float gains[12]=  {Kp_z, Ki_z, Kd_z,
	                   Kp_phi, Ki_phi, Kd_phi,
	                   Kp_theta, Ki_theta, Kd_theta,
	                   Kp_psi, Ki_psi, Kd_psi
	                  };

	io.t = time;
	for (int i=0; i<12; i++) {
		//copy the gains
		io.gains[i] = gains[i];
	}

	//why?
	io.e[0] = data[3];//altitude error centimetres
	io.e[1] = data[0] ;//roll error rad
	//frame X for erlebrain2
	//io.e[2] = -data[1] ;//pitch error rad
	//frame +
	io.e[2] = data[1] ;
	io.e[3] = data[2] ;//yaw error rad 


	PIDcontroller_compute(&io,&state);

	float m = 1; //kg
	/*--------------------------------------------------------------------------------------------------------------------------*/
	// modified by TRAN TRUNG DUC - 2016
	if(control_mode != STABILIZE)
	{
		if ((cos(roll) * cos(pitch)) != 0.0) 
		{
				io.y[0] = (io.y[0] + 9.81) * m / (cos(roll) * cos(pitch));
		} else 
		{
				io.y[0] = (io.y[0] + 9.81) * m / (cos(roll) * cos(pitch) + 0.001);
		}
	}
	else
	{
		io.y[0]	= (get_pilot_desired_throttle(channel_throttle->control_in))/50 * m;
	}

	
	io.y[0] = constrain_float(io.y[0], 0, 19.6); //THRUST
	io.y[1] = constrain_float(io.y[1], -1.257, 1.257); //MOMENT
	io.y[2] = constrain_float(io.y[2], -1.257, 1.257); //MOMENT
	io.y[3] = constrain_float(io.y[3], -0.279, 0.279); //MOMENT
	
	return io.y;
}

////Inputs: current altitude,
////desired altitute, current orientation, current
////angular velocity, //ALL RADIANS
GAREAL* Copter::Simple_IB_calculate()
{
    double time = (double) millis() / 1000.0;
    //if sample time is over a second, something is horribly wrong
    if (fabs(time - time_old) > 1.0)
    {
        time_old = time;
        IB_results[0] = 0;
        IB_results[1] = 0;
        IB_results[2] = 0;
        IB_results[3] = 0;
        return IB_results;
    }
	float curr_alt = data[6]/100+0.2; // mets
	float target_alt = data[10]/100; // mets
	Vector3f curr_angle = Vector3f(ahrs.roll,ahrs.pitch,ahrs.yaw); // radians
	Vector3f angular_speed = ahrs.get_gyro(); // radians/s
	
    //Get the values
    io_alt.z = curr_alt; 
    io_alt.zd = target_alt; // target altitude
    
    io_alt.dt = time - time_old;
    io_alt.phi = curr_angle.x;
    io_alt.theta = curr_angle.y;
    //run altitude controle Block
    Altitude_compute(&io_alt, &state_alt);
    
    //check
    
    flag[3] = data[3]/100;
  
    
	
	//minimum thrust
    io_attitude.angles[0] = curr_angle.x;
    io_attitude.angles[1] = curr_angle.y;
    io_attitude.angles[2] = curr_angle.z;
    io_attitude.angles[3] = angular_speed.x;
    io_attitude.angles[4] = angular_speed.y;
    io_attitude.angles[5] = angular_speed.z;//*AC_ATTITUDE_CONTROL_DEGX100;
    io_attitude.dt = time - time_old;
    //subtraction???
    io_attitude.phid = data[7];
    io_attitude.thetad = data[8];
    io_attitude.psid = data[9];
    if(control_mode == STABILIZE)
    {
    	io_alt.U1	= (get_pilot_desired_throttle(channel_throttle->control_in))/50 * m;
   		io_attitude.phid = constrain_float(io_attitude.phid ,-PI/9,PI/9);
    	io_attitude.thetad = constrain_float(io_attitude.thetad,-PI/9,PI/9); 
    }
	io_attitude.phid = constrain_float(io_attitude.phid ,-PI/6,PI/6);
    io_attitude.thetad = constrain_float(io_attitude.thetad,-PI/6,PI/6); 
	IB_results[0] = constrain_float(io_alt.U1,0,20);

    io_attitude.omgs[0] = motor_omega[3];
    io_attitude.omgs[1] = motor_omega[0];
    io_attitude.omgs[2] = motor_omega[2];
    io_attitude.omgs[3] = motor_omega[1];
    
	//run attitude control Block
    IB_Controller_ga_attitude_final_compute(&io_attitude,&state_attitude);    
    time_old = time;
    
    IB_results[1] = io_attitude.U2;
    IB_results[2] = io_attitude.U3;
    IB_results[3] = io_attitude.U4;

    return IB_results;
}

// called from Copte::fast_loop()
void Copter::get_all_input() {
	attitude_control.getdata(data);// get attitude information
	pos_control.getdata(data);	// get position information

}
void Copter::new_controller_run() {
	//controller_output is the 4-vector of U values
	GAREAL *controller_output;
	//Integrate the calculation for new controllers here
	switch (g.using_controller) {
		case Original_PID_Controller:
			break;
		case New_PID_Controller:
			controller_output = PID_calculate();
			//test
			//IB_calculate();
			break;
		case Simple_IB_Controller:
			controller_output = Simple_IB_calculate();
			break;
			//}
	}
	new_motors_output(controller_output);
}




/// motors_output - send output to motors library which will adjust and send to ESCs and servos
/// ENSMA
void Copter::new_motors_output(GAREAL *output_value) {
	// Limits  our quadrotor
	// are these rotor parameters?
	float b = 0.00007; //Ns2
	float d = 0.00000175; //Nms2
	float l = 0.2616; //m
	float m =1;
	float lsin45 = l*sin(PI/4);
    switch (g.using_controller) 
	{
		/*case New_PID_Controller: 
		motor_omega[0] = ((output_value[0]-(output_value[1]+output_value[2])/lsin45)/b-output_value[3]/d)/4.0;
		motor_omega[1] = ((output_value[0]+(output_value[1]+output_value[2])/lsin45)/b-output_value[3]/d)/4.0;
		motor_omega[2] = ((output_value[0]+(output_value[1]-output_value[2])/lsin45)/b+output_value[3]/d)/4.0;
		motor_omega[3] = ((output_value[0]-(output_value[1]-output_value[2])/lsin45)/b+output_value[3]/d)/4.0;
		motor_omega[0] = (motor_omega[0]<0)?0:sqrt(motor_omega[0]);      
		motor_omega[1] = (motor_omega[1]<0)?0:sqrt(motor_omega[1]);   
		motor_omega[2] = (motor_omega[2]<0)?0:sqrt(motor_omega[2]);   
		motor_omega[3] = (motor_omega[3]<0)?0:sqrt(motor_omega[3]);
		*/
		//frame +
		case New_PID_Controller: 
		{
			io_motors.U1 = output_value[0]; //THRUST
    		io_motors.U2 = output_value[1]; //MOMENT
    		io_motors.U3 = output_value[2]; //MOMENT
    		io_motors.U4 = output_value[3]; //MOMENT
			//Call to the GA U -> omega transformation
			MotorSpeed_compute(&io_motors);
			motor_omega[3] = io_motors.omgs2[0];
			motor_omega[0] = io_motors.omgs2[1];
			motor_omega[2] = io_motors.omgs2[2];
			motor_omega[1] = io_motors.omgs2[3];
		}
		for (uint8_t i = 0; i<4; i++) 
		{
			if (motor_omega[i] <= 0.0) {
				motor_omega[i] = 0 ;
			}
			if (motor_omega[i] > 248.0) 
			{
				motor_omega[i] = 248.0;
			}
		}

	/*	if (ap.motor_test) 
		{
       		motor_test_output();
    	} 
		else 
		{
       		if (!ap.using_interlock)
			{
        	// if not using interlock switch, set according to Emergency Stop status
        	// where Emergency Stop is forced false during arming if Emergency Stop switch
        	// is not used. Interlock enabled means motors run, so we must
        	// invert motor_emergency_stop status for motors to run.
        		motors.set_interlock(!ap.motor_emergency_stop);
    		}*/
    		motors.calibration= 0;
    		motors.level_pwm = g.level_pwm;
			motors.new_output(motor_omega);
		//}	
		break;
		case Calibration: 
		motor_omega[0]= g.calibration_pwm;
		motor_omega[1]= g.calibration_pwm;
		motor_omega[2]= g.calibration_pwm;
		motor_omega[3]= g.calibration_pwm;
		motors.calibration= 1;
    	motors.new_output(motor_omega);	                
		
	}
	
}

