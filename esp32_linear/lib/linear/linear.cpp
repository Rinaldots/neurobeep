
#include "linear.h"

#include <stdio.h>
#include <math.h>


// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 30

#define DIR 41
#define STEP 42
#define SLEEP 13

#define ENDSTOP 39
volatile bool endstopTriggered = false;

#include "DRV8825.h"
#define M0 10
#define M1 11

DRV8825 stepper(MOTOR_STEPS, DIR, STEP, SLEEP, M0, M1);

void LinearCar::setup()
{  
  stepper.begin(RPM);
  stepper.enable();
  stepper.setMicrostep(1);

  pinMode(ENDSTOP, INPUT_PULLUP);
  
}

void LinearCar::step_loop()
{
  Serial.print("Passos: ");
  Serial.println(steps);
  bool endstopTriggered = digitalRead(ENDSTOP);
  

  if(estado == HOMING){
    if(endstopTriggered){
    estado = PARADO;
    stepper.startMove(-1);
    steps = 0;
  }else{
    stepper.startMove(1);
  }
  }
  bool logica = comando == PLAY && estado != HOMING;
  Serial.print(logica);
  if(comando == PLAY && estado != HOMING){

    Serial.print("Comando: ");
	  Serial.print(comando);
	  Serial.print(" Estado: ");
	  Serial.println(estado);

    if(steps >= -50){
      estado = INDO;
    }
    if(steps == -limitePassos)
    {      
      estado = VOLTANDO;    
    }

    if (estado == INDO)
    {      stepper.startMove(-1);
           steps = steps -1;   }
    else if(estado == VOLTANDO)
    {      stepper.startMove(1);
           steps = steps +1;
    }
  }
  stepper.nextAction();
}

void LinearCar::setTargetStep(long steps)
{
  stepper.startMove(steps-stepDelta);
}


LinearCar::LinearCar() {}

