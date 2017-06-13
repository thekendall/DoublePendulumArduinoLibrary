/* 
 * Created by Kendall Lui
 *
 * Copyright 2017 Kendall Lui
 * An arduino interface library for a Double Pendulum Robot with
 * a pertubation unit.
*/
#ifndef DOUBLE_PENDULUM_ARDUINO_LIBRARY
#define DOUBLE_PENDULUM_ARDUINO_LIBRARY

#include <Arduino.h>
#include <Stream.h>

// Bottom Motor Setup
#define BM_PWM 6 // motor driver PWM pin 6
#define BM_DIR 7 // motor driver Dir pin 7
#define BM_CURRENT 0 // Current Sensor a0
#define BM_PIN_A 2 // Encoder Interrupt Pin 2
#define BM_PIN_B 3 // Encoder Interrupt Pin 3

// Top Motor Setup
#define TM_PWM 8 // motor driver PWM pin 6
#define TM_DIR 9 // motor driver Dir pin 7
#define TM_CURRENT 2 // Current Sensor a0
#define TM_PIN_A 18 // Encoder Interrupt Pin 2
#define TM_PIN_B 19 // Encoder Interrupt Pin 3

// Cart Perturbation Setup
#define CART_PWM 10 // motor driver PWM pin 6
#define CART_DIR 11 // motor driver Dir pin 7
#define CART_CURRENT 0 // Current Sensor a0
#define CART_PIN_A 20 // Encoder Interrupt Pin 2
#define CART_PIN_B 21 // Encoder Interrupt Pin 3

#define ESTOP 13 //Estop Switch

class DoublePendulumRobot 
{
	public:
		DoublePendulumRobot();
		void robotLoop();
		
		void setBMPWM(int PWM);
		double getBMDegrees();
		double getBMCurrent();
		double getBMAngularVelocity();
		double getBMAngularAcceleration();
		void setBMZero();
		void clearEStop();

	private:
		virtual void BMController();
		static void BM_ISR_PINA();
		static void BM_ISR_PINB();
		void BMCalculate();
		static long BMencoderCount; 
		double BMtheta; 
		double BMomega;
		double BMalpha;

		//Cart
		static void CART_ISR_PINA();
		static void CART_ISR_PINB();
		static long CARTencoderCount; 


		//General
		void safetySupervisor();
		void EStop_SR();
		int prevTime;
		bool EStop;

	protected:
		double dt; //sec
};

class PIDSinglePendulumRobot: public DoublePendulumRobot
{
	public:
		PIDSinglePendulumRobot();
		void setPID(double kp, double ki, double kd);
	private:
		double Kp;
		double Ki;
		double Kd;
		double BMerror;
		double BMerrorSum;
		double BMdError;
		double setPoint;
		void BMController();
};

void setupInterrupts(DoublePendulumRobot db);

#endif

