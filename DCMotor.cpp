#include <Arduino.h>
#include <DCMotor.h>

//Medidas rover 5: robo24x24 cm ;~=333 ticks por volta da roda ;1.08 graus por tick; 3cm de raio da roda; ~=18cm de comprimento de roda
#define MAX_SPEED 255
#define PIN_MOTOR_LEFT 4
#define PIN_MOTOR_RIGHT 6
#define PART 4
#define SETPOINT 50
#define SAMPLING_RATE 1
#define SAMPLINGS 1000 / SAMPLING_RATE
#define PIN_PWM_LEFT 5
#define PIN_PWM_RIGHT 7
#define RADIUS_WHEEL 3
#define PI 3.14
#define ANGLE_PER_TICK 1.08			// 360/333 agunlo por tick
#define TICKS_PER_TURN 333		// 333 por volta

int DCMotor :: count_left_ticks = 0 ;
int DCMotor :: count_right_ticks = 0 ;
int DCMotor ::count_error_left = 0;
int DCMotor ::count_error_right = 0;

DCMotor :: DCMotor(float _kp, float _kd, float _ki) {
	left_speed = MAX_SPEED/3;
	right_speed = MAX_SPEED/3;
	kp = _kp ;
	kd = _kd ;
	ki = _ki ;
}

void DCMotor :: begin (){
	last_check_inst = millis ( ) ;
	Serial.print("begin =======================> ");
	Serial.println(last_check_inst);
	Ierrorl = 0 ;
	Ierrorr = 0 ;
	preverrorl = 0;
	preverrorr = 0;
	attachInterrupt( 0 , inc_left_ticks , CHANGE) ;
	attachInterrupt ( 1 , inc_right_ticks , CHANGE) ;
	pinMode(PIN_MOTOR_LEFT,OUTPUT);
	pinMode(PIN_MOTOR_RIGHT,OUTPUT);
	pinMode(PIN_PWM_LEFT,OUTPUT);
	pinMode(PIN_PWM_RIGHT,OUTPUT);
}
void DCMotor :: setMotorVelocity ( ) {
	digitalWrite (PIN_MOTOR_LEFT, LOW);
	digitalWrite (PIN_MOTOR_RIGHT, LOW ) ;
	analogWrite(PIN_PWM_LEFT,left_speed);
	analogWrite(PIN_PWM_RIGHT,right_speed);
}

void DCMotor :: forward (unsigned int distance ) {
	ticks_limit =Func_Dist(distance);
	Serial.print("ticks limit: ");
	Serial.println(ticks_limit);
	if (vel[0])
	{
		left_speed = vel[0];
	}else{
		left_speed = MAX_SPEED/3;
	}
	
	if (vel[1])
	{
		right_speed = vel[1];
	}else{
		right_speed = MAX_SPEED/3;
	}

	count_error_left = ticks_limit;
	count_error_right = ticks_limit;
	setMotorVelocity();
	//while(count_left_ticks <=ticks_limit || count_right_ticks <=ticks_limit){
	while(count_error_left >= 0 || count_error_right >= 0 ){
		/*Serial.print(" left ticks : ");
		Serial.println(count_left_ticks);
		Serial.print("right ticks: ");
		Serial.println(count_right_ticks);
		Serial.print(" left error : ");
		Serial.println(count_error_left);
		Serial.print("right error: ");
		Serial.println(count_error_right);*/
		//count_error_left -= count_left_ticks;
		//count_error_right -= count_right_ticks;
		//doPID(count_left_ticks,count_right_ticks);
		//DCMotor::stop();
		//delay(500);
		doPID();
		left_speed = vel[0];
		right_speed = vel[1];
		setMotorVelocity();
	}
	Serial.println("ticks passaram");
	Serial.print("ticks limit: ");
	Serial.println(ticks_limit);
	Serial.print("ticks limit/PART: ");
	Serial.println(ticks_limit/PART);
	Serial.print(" left ticks : ");
	Serial.println(count_left_ticks);
	Serial.print("right ticks: ");
	Serial.println(count_right_ticks);
	Serial.print(" left error : ");
	Serial.println(count_error_left);
	Serial.print("right error: ");
	Serial.println(count_error_right);
	DCMotor::stop();
	delay(1000);
	count_left_ticks = 0;
	count_right_ticks = 0;
	count_error_left = 0;
	count_error_right = 0;
	
}

void DCMotor :: stop ( ) {
	left_speed = 0 ;
	right_speed = 0 ;
	setMotorVelocity ( );
}

void DCMotor :: left (int angle) {
	left_speed = MAX_SPEED/4;
	right_speed = MAX_SPEED/4;
	ticks_limit = Func_Angle(angle);
		while(count_left_ticks <=ticks_limit || count_right_ticks <=ticks_limit){
			digitalWrite(PIN_MOTOR_LEFT, HIGH);
			digitalWrite(PIN_MOTOR_RIGHT, LOW );
			analogWrite(PIN_PWM_LEFT,left_speed);
			analogWrite(PIN_PWM_RIGHT,right_speed);
			/*Serial.print("left ticks: ");
			Serial.println(count_left_ticks);
			Serial.print("right ticks: ");
			Serial.println(count_right_ticks);*/
		}
		/*Serial.println("ticks passaram");
		Serial.print("ticks limit: ");
		Serial.println(ticks_limit);*/
		left_speed = 0;
		right_speed = 0;
		setMotorVelocity();
		count_left_ticks = 0;
		count_right_ticks = 0;			
}

void DCMotor :: right (int angle) {
	left_speed = MAX_SPEED/4;
	right_speed = MAX_SPEED/4;
	ticks_limit = Func_Angle(angle);
		while(count_left_ticks <=ticks_limit || count_right_ticks <=ticks_limit){
			digitalWrite(PIN_MOTOR_LEFT, LOW);
			digitalWrite(PIN_MOTOR_RIGHT, HIGH );
			analogWrite(PIN_PWM_LEFT,left_speed);
			analogWrite(PIN_PWM_RIGHT,right_speed);
			Serial.print("left ticks: ");
			Serial.println(count_left_ticks);
			Serial.print("right ticks: ");
			Serial.println(count_right_ticks);
		}
		Serial.println("ticks passaram");
		Serial.print("ticks limit: ");
		Serial.println(ticks_limit);
		left_speed = 0;
		right_speed = 0;
		setMotorVelocity();
		count_left_ticks = 0;
		count_right_ticks = 0;	
}	
	
void DCMotor :: doPID() {
	int errorl ; // error from left encoder
	int errorr ; // error from right encoder
	int derrorl ; // derivative error left
	int derrorr ; // derivative error right
	int cl ;
	int cr ;
	int setpoint = ticks_limit/PART;
		// calculate error values
		errorl = setpoint - count_left_ticks ;
		errorr = setpoint - count_right_ticks ;
		/*Serial.print("Esq = ");
		Serial.println(count_left_ticks);
		Serial.print("Dir = ");
		Serial.println(count_right_ticks);
		Serial.print("Esq speed = ");
		Serial.println(left_speed);
		Serial.print("Dir speed = ");
		Serial.println(right_speed);*/
		// reset encoder counts ready for next sample
		count_left_ticks = 0 ;
		count_right_ticks = 0 ;
		derrorl = errorl - preverrorl;
		derrorr = errorr - preverrorr;
		cl = (( kp * errorl ) + ( kd * derrorl ) + ( ki * Ierrorl )) ; //PID equations
		cr = (( kp * errorr ) + ( kd * derrorr ) + ( ki * Ierrorr )) ;
		/*Serial.print("Esq cl = ");
		Serial.println(cl);
		Serial.print("Dir cr = ");
		Serial.println(cr);*/
		if(count_left_ticks < count_right_ticks){
				if (( left_speed + cl ) > MAX_SPEED)
				// prevent speed from over running MAX_SPEED
					left_speed = MAX_SPEED;	
				else if (( left_speed + cl ) < MAX_SPEED/2)
					left_speed = MAX_SPEED;
				else 
				// use output from PID equations to alter motor speeds
					left_speed = left_speed + cl ;
		}else if(count_left_ticks > count_right_ticks){
				if (( right_speed + cr ) > MAX_SPEED)
				//where speed is PWM output values
					right_speed = MAX_SPEED;	
				else if (( right_speed + cr ) < MAX_SPEED/2)
					right_speed = MAX_SPEED;
				else
					right_speed = right_speed + cr ;
					// set previous error to current error				
			}
		else{
			right_speed = right_speed;
			left_speed = left_speed;
		}
		vel[0] = left_speed;
		vel[1] = right_speed;
		//DCMotor::stop();
		//delay(100);
		//count_left_ticks = 0 ;
		//count_right_ticks = 0 ;
		/*Serial.print("vel0 = ");
		Serial.println(vel[0]);
		Serial.print("vel1= ");
		Serial.println(vel[1]);		
		Serial.print("left ticks:");
		Serial.println(count_left_ticks);
		Serial.print("right ticks: ");
		Serial.println(count_right_ticks);*/
		preverrorl = errorl ;
		preverrorr = errorr ;
		//add current error to integral error
		Ierrorl = Ierrorl + errorl;
		Ierrorr = Ierrorr + errorr;

}

void DCMotor::inc_left_ticks( ){
	DCMotor::count_left_ticks++;
	DCMotor ::count_error_left--;
}

void DCMotor::inc_right_ticks( ){
	DCMotor::count_right_ticks++;
	DCMotor ::count_error_right--;
}

int DCMotor:: Func_Angle(int angle){
//	angle = angle*6; // angulo*(fator de proporcao do raio da roda com o raio do robo + uma correção de erro(4 + 2)) 
	angle = angle*4; // angulo*(fator de proporcao do raio da roda com o raio do robo)
	return (angle/ANGLE_PER_TICK);
}

unsigned int DCMotor:: Func_Dist(unsigned int distance){//verificar proporção
	return ((distance*TICKS_PER_TURN)/(2*PI*RADIUS_WHEEL));
}

/*int DCMotor::getValorRodaEsq(){
    return left_speed;
}

int DCMotor::getValorRodaDir(){
    return right_speed;
}*/

int DCMotor::get_left_ticks(){
	return count_left_ticks;
}

int DCMotor::get_right_ticks(){
	return count_right_ticks;
}