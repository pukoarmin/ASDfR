/**********************************************************
 * This file is generated by 20-sim ANSI-C Code Generator
 *
 *  file:  common\xxinteg.h
 *  model: ControllerTilt
 *  expmt: Jiwy-with-controller
 *  date:  March 30, 2023
 *  time:  12:00:10 PM
 *  user:  20-sim 5.0 Student License
 *  from:  Universiteit Twente
 *  build: 5.0.2.12127
 **********************************************************/

#ifndef XX_INTEG_H
#define XX_INTEG_H

/* 20-sim include files */
#include "xxtypes.h"

class Submodel20sim;	//Forward declaration

class IntegrationMethod
{
  public:
	IntegrationMethod(void): model(NULL) {	};
	virtual ~IntegrationMethod(void) {	};
	virtual void Initialize(Submodel20sim* themodel) = 0;
	virtual void Terminate() = 0;
	virtual void Step() = 0;
	
  protected:
	Submodel20sim* model;
};

class Euler: public IntegrationMethod
{
  public:
	Euler(void);
	~Euler(void);
	void Initialize(Submodel20sim* themodel);
	void Terminate();
	void Step();
};

class Discrete: public IntegrationMethod
{
  public:
	Discrete(void);
	~Discrete(void);
	void Initialize(Submodel20sim* themodel);
	void Terminate();
	void Step();
};

class RungeKutta4: public IntegrationMethod
{
  public:
	RungeKutta4(void);
	~RungeKutta4(void);
	void Initialize(Submodel20sim* themodel);
	void Terminate();
	void Step();
	
  private:
	XXDouble* q0;
	XXDouble* q1;
	XXDouble* q2;
	XXDouble* q3;
	XXDouble* q4;
	const XXDouble OneOverSix = 1.0 / 6.0;
};

class RungeKutta2: public IntegrationMethod
{
  public:
	RungeKutta2(void);
	~RungeKutta2(void);
	void Initialize(Submodel20sim* themodel);
	void Terminate();
	void Step();
	
  private:
	XXDouble* q0;
};

#endif

