#ifndef DCMotor_h
#define DCMotor_h
#include <Arduino.h>
class DCMotor {
public :
	DCMotor ( float _kp, float _kd, float _ki ) ;
	void begin ( ) ;
	void forward(unsigned int distance) ;
	void stop( ) ;
	void left(int angle) ;
	void right(int angle) ;
	/*  int getValorRodaDir();
	int getValorRodaEsq();*/
	int get_left_ticks();
	int get_right_ticks();

private :
	int left_speed ;
	int right_speed ;
	long last_check_inst ;
	unsigned int ticks_limit;
	int Func_Angle(int angle);
	unsigned int Func_Dist(unsigned int distance);
	void setMotorVelocity ( ) ;
	void doPID () ;
	int kp ; //PID proportional gain constant
	float kd ; //PID derivative gain constant
	float ki ; //PID intergral gain constant
	int preverrorl ; // left motor previous error
	int preverrorr ; // right motor previous error
	int Ierrorl ; // left motor intergral error
	int Ierrorr ; // right motor intergral error
	int vel[2];
	static int count_left_ticks;
	static int count_right_ticks;
	static int count_error_left;
	static int count_error_right;
	static void inc_left_ticks( );
	static void inc_right_ticks( );

} ;
#endif


