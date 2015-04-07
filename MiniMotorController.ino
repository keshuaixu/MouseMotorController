#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <PWM.h>
#include <Wire.h>

#include <Encoder.h>
#include <RegulatedMotor.h>;
#include <KinematicController.h>
#include "MiniMotorControllerDefinitions.h"

Encoder enc1(2,4);
Encoder enc2(3,5);

RegulatedMotor m1(&enc1,7,6,10);
RegulatedMotor m2(&enc2,8,11,9);

KinematicController kc(&m1,&m2,1,1,100,50,64*4*30);

void setup(){
	Serial.begin(115200);
	Wire.begin(0x01);
	Wire.onReceive(receiveEvent);
	pinMode(6,OUTPUT);
	pinMode(7,OUTPUT);
	pinMode(8,OUTPUT);
	pinMode(9,OUTPUT);
	pinMode(10,OUTPUT);
	pinMode(11,OUTPUT);

	InitTimersSafe();      
	SetPinFrequencySafe(10, 150);
	SetPinFrequencySafe(9, 150);

	m1.setSampleTime(20000);
	m2.setSampleTime(20000);
	m1.setPID(0.19,0.0085,0.02,0.0);
	m2.setPID(0.19,0.0085,0.02,0.0);

	kc.setAcceleration(1000,1000,1000,1000);
}

long lastHeartbeat = 0;

void loop(){
	kc.run();
	m1.run();
	m2.run();

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
		default:
		break;
	}
	while (Wire.available() > 0){
		Wire.read();
		Serial.println("too many");
	}
}