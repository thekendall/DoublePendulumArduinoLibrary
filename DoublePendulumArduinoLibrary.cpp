/* 
 * Created by Kendall Lui
 *
 * Copyright 2017 Kendall Lui
 * An arduino interface library for a Double Pendulum Robot with
 * a pertubation unit.
*/

#include "DoublePendulumArduinoLibrary.h"

DoublePendulumRobot::DoublePendulumRobot() {
	pinMode(BM_PWM, OUTPUT);
	pinMode(BM_DIR, OUTPUT);
	pinMode(BM_PIN_A, INPUT);
	pinMode(BM_PIN_B, INPUT);
	pinMode(ESTOP, INPUT);

	//Initialize Variables
	BMtheta = 0;
	BMalpha = 0;
	BMomega = 0;
	dt = 0.0001;
	EStop = true;

	//Attach Encoder ISR
	attachInterrupt(digitalPinToInterrupt(BM_PIN_A),this->BM_ISR_PINA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BM_PIN_B),this->BM_ISR_PINB, CHANGE);

	//Attach Encoder CART
	attachInterrupt(digitalPinToInterrupt(CART_PIN_A),this->CART_ISR_PINA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CART_PIN_B),this->CART_ISR_PINB, CHANGE);
}

void DoublePendulumRobot::setBMPWM(int PWM) {
	if(EStop) 
	{
		PWM = 0;
	}
	if(PWM < 0) {
		digitalWrite(BM_DIR, 0);
		PWM = - PWM;
	} else {
		digitalWrite(BM_DIR, 1);
	}	
	if(PWM > 255) { PWM = 255; }
	analogWrite(BM_PWM,PWM);
}
double DoublePendulumRobot::getBMDegrees(){
	return BMtheta;
}

double DoublePendulumRobot::getBMCurrent() {
	return (analogRead(BM_CURRENT) - 512.0) * 5.0/512.0;
}


double DoublePendulumRobot::getBMAngularVelocity() {
	return BMomega;
}
double DoublePendulumRobot::getBMAngularAcceleration(){
	return BMalpha;
}

void DoublePendulumRobot::BMCalculate() {  // Updates Values.
	double theta = BMencoderCount*360/4096.0; // Current theta
	double omega = (theta - BMtheta)/dt; // previous BMtheta
	double alpha = (omega - BMomega)/dt; // previous BMomega

	BMtheta = theta;
	BMomega = omega;
	BMalpha = alpha;
}
void DoublePendulumRobot::setBMZero() {
	BMencoderCount = 0;
}

void DoublePendulumRobot::robotLoop(){
	dt = (millis() - prevTime);
	prevTime = millis();
	BMCalculate();
	BMController();
	safetySupervisor();
}

void DoublePendulumRobot::BMController() {
	//Override this function
}

//Avoids Large Velocities and angle changes
void DoublePendulumRobot::safetySupervisor() {
	if(BMtheta < -60 || BMtheta > 60 || digitalRead(ESTOP) == LOW) {
		EStop = true;
		EStop_SR();
	}
}

void DoublePendulumRobot::EStop_SR() {
	digitalWrite(BM_PWM,LOW);
	digitalWrite(BM_DIR,LOW);
}

void DoublePendulumRobot::clearEStop() {
	EStop = false;
}


//Encoder Interrupt Service Routines
long DoublePendulumRobot::BMencoderCount; // static member

void DoublePendulumRobot::BM_ISR_PINA() {
	if(digitalRead(BM_PIN_A) == HIGH) {
		if(digitalRead(BM_PIN_B) == HIGH) {
			BMencoderCount--;
		} else {
			BMencoderCount++;
		}
	} else {
		if(digitalRead(BM_PIN_B) == HIGH) {
			BMencoderCount++;
		} else {
			BMencoderCount--;
		}
	}
}

void DoublePendulumRobot::BM_ISR_PINB() {
	if(digitalRead(BM_PIN_B) == HIGH) {
		if(digitalRead(BM_PIN_A) == HIGH) {
			BMencoderCount++;
		} else {
			BMencoderCount--;
		}
	} else {
		if(digitalRead(BM_PIN_A) == HIGH) {
			BMencoderCount--;
		} else {
			BMencoderCount++;
		}
	}
	
}

//Encoder Interrupt Service Routines
long DoublePendulumRobot::CARTencoderCount; // static member

void DoublePendulumRobot::CART_ISR_PINA() {
	if(digitalRead(CART_PIN_A) == HIGH) {
		if(digitalRead(CART_PIN_B) == HIGH) {
			CARTencoderCount++;
		} else {
			CARTencoderCount--;
		}
	} else {
		if(digitalRead(BM_PIN_B) == HIGH) {
			CARTencoderCount--;
		} else {
			CARTencoderCount++;
		}
	}
}

void DoublePendulumRobot::CART_ISR_PINB() {
	if(digitalRead(CART_PIN_B) == HIGH) {
		if(digitalRead(CART_PIN_A) == HIGH) {
			CARTencoderCount--;
		} else {
			CARTencoderCount++;
		}
	} else {
		if(digitalRead(CART_PIN_A) == HIGH) {
			CARTencoderCount++;
		} else {
			CARTencoderCount--;
		}
	}
}

void PIDSinglePendulumRobot::BMController() {
	double error = 0 - getBMDegrees();
	double dError = (error - BMerror)/dt;
	BMerrorSum += error;
	BMerror = error;
	BMdError = dError;

	double output = Kp*BMerror + Ki*BMerrorSum + Kd*BMdError;
	setBMPWM(output);
}

void PIDSinglePendulumRobot::setPID(double kp, double ki, double kd) {
	Kp = kp;
	Ki = ki;
	Kd = kd;
}

PIDSinglePendulumRobot::PIDSinglePendulumRobot() {
	Kp = 0;
	Ki = 0;
	Kd = 0;
}
