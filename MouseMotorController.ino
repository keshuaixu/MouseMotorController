#include <Wire.h>
#include <MouseMotor.h>;
#include <KinematicController.h>
#include <MotorControllerDefinitions.h>


int MD1PWM = 39;// Motor 1 PWM CNTRL
int MD2PWM = 2;// Motor 2 PWM CNTRL
int MD1EN = 38;// Motor 1 Direction
int MD2EN = 10;// Motor 2 Direction
int MD1SL = 37;// Motor 1 Direction
int MD2SL = 4;// Motor 2 Direction
int M1ENC1 = 36;// Motor 1 Encoder Input 1
int M1ENC2 = 6;// Motor 1 Encoder Input 2
int M2ENC1 = 7;// Motor 2 Encoder Input 1
int M2ENC2 = 8;// Motor 2 Encoder Input 2
const int SDA = 21;// I2C SDA
const int SCL = 20;// I2C SCL
long M1ENCNT = 0;// Motor 1 Encoder count
long M2ENCNT = 0;// Motor 2 Encoder count 
float WheelRadius = 10;//wheel radius in mm
int coutper = 30;// number of encoder counts per one rotation
int command;

long encleft;
long encright;

MouseMotor m1(MD1SL,MD1EN,MD1PWM,&M1ENCNT);
MouseMotor m2(MD2SL,MD2EN,MD2PWM,&M2ENCNT);

KinematicController kc(&m1,&m2,1,-1,100,10,12*4*30);

void setup(){
	Wire.begin(MOTOR_CONTROLLER_ADDRESS);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);

	pinMode(SDA,INPUT_PULLUP); //Set Pullup for I2C
	pinMode(SCL, INPUT_PULLUP); //Set Pullup for I2C

	pinMode(MD1PWM,OUTPUT); 
	pinMode(MD2PWM, OUTPUT); 
	pinMode(MD1EN,OUTPUT); 
	pinMode(MD2EN, OUTPUT); 
	pinMode(MD1SL,OUTPUT); 
	pinMode(MD2SL, OUTPUT); 
	pinMode(M1ENC1,INPUT); 
	pinMode(M1ENC2, INPUT); 
	pinMode(M2ENC1, INPUT); 
	pinMode(M2ENC2, INPUT); 

	kc.setAcceleration(1000,1000,1000,1000);
}

long lastHeartbeat = 0;

void loop(){
	kc.run();

	if ( millis()-lastHeartbeat > 5000 ){
	}
}


int responseState;

void receiveEvent(int howMany){
	byte command = Wire.read();
	uint16_t arg[16];
	switch (command) {
		case COMMAND_HEARTBEAT:
			lastHeartbeat = millis();
		break;
		case COMMAND_SETACCELERATION:
			for (int i = 0; i < 4; i++){
				arg[i] = Wire.read();
				arg[i] |= Wire.read() << 8;
			}
			kc.setAcceleration(arg[0],arg[1],arg[2],arg[3]);
		break;
		case COMMAND_GOVELOCITY:
			for (int i = 0; i < 2; i++){
				arg[i] = Wire.read();
				arg[i] |= Wire.read() << 8;
			}
			kc.goVelocity(arg[0],arg[1]);
		break;
		case COMMAND_BRAKE:
			kc.brake();
		break;
		case COMMAND_COAST:
			kc.coast();
		break;
		case COMMAND_REPORTSTANDBY:
			responseState = COMMAND_REPORTSTANDBY;
		break;
		case COMMAND_REPORTENCODER:
			responseState = COMMAND_REPORTENCODER;
		break;
		default:
		break;
	}
	while (Wire.available() > 0){
		Wire.read();
	}
}

void requestEvent(){
	if (responseState == COMMAND_REPORTSTANDBY) {
		byte resp = kc.isStandby()?0x01:0x00;
		Wire.write(resp);
		responseState = 0x00;
	} else if (responseState == COMMAND_REPORTENCODER){
		byte resp[8];
		int32_t fwd = kc.getOdometryForward();
		int32_t ccw = kc.getOdometryCCW();
		resp[0] = fwd;
		resp[1] = fwd >> 8;
		resp[2] = fwd >> 16;
		resp[3] = fwd >> 24;
		resp[4] = ccw;
		resp[5] = ccw >> 8;
		resp[6] = ccw >> 16;
		resp[7] = ccw >> 24;
		Wire.write(resp,8);
	}
}