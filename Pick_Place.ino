#include <AccelStepper.h>
#include <Servo_ESP32.h>

// Define stepper motor connections
#define STEP_PIN1 25
#define DIR_PIN1 26
#define STEP_PIN2 32
#define DIR_PIN2 33
#define STEP_PIN3 27
#define DIR_PIN3 14

// Create AccelStepper objects for both stepper motors
AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN3, DIR_PIN3);

static const int servoPin1 = 2;  //  J6   Angular movement (Servomotor)
static const int servoPin2 = 13;  //  J7   Angular movement (Servomotor)


Servo_ESP32 servo1;  //  Instancias para control de servomotores
Servo_ESP32 servo2;

void setup() {
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);

  stepper1.setMaxSpeed(2000);
  stepper1.setAcceleration(500);

  stepper2.setMaxSpeed(750);
  stepper2.setAcceleration(170);

  stepper3.setMaxSpeed(2000);
  stepper3.setAcceleration(500);
}

void loop() {
//  Tomar objeto
  stepper3.moveTo(3200);
  stepper2.moveTo(200);   // 400 pasos un arevolucion 
  stepper1.moveTo(12800);
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
  servo1.write(90);
  servo2.write(90);   //  Este servomotor cerrara las pinzas
  /*********************************************************************************************************************************************************/
  // Soltar objeto
  stepper3.moveTo(0);
  stepper2.moveTo(200);
  stepper1.moveTo(0);
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || stepper3.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
  servo1.write(90);
  servo2.write(0);    //  Se abrirán las pinzas
}
