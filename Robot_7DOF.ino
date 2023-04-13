//  CÓDIGO PARA ROBOT DE 7 GRADOS DE LIBERTAD
#include <AccelStepper.h>
#include <FastLED.h>
#include <Servo_ESP32.h>

//  NOTA PARA EJECUTAR UNA VUELTA COMPLETA EL PASO
//  STEPPER MOTOR CONTROL PIN OUT
//    _________________________________
//   | STEP 2 -- 32 ______ DIR 1 -- 33 |
//   | STEP 1 -- 25 ______ DIR 2 -- 26 |
//   | STEP 3 -- 27 ______ DIR 3 -- 14 |
//   | STEP 4 -- 12 ______ DIR 4 -- 16 |
//   | STEP 5 -- 17 ______ DIR 5 -- 5 |
//   | STEP 6 -- 18 ______ DIR 6 -- 19 |     ¡¡¡  NO UTILIZAR  !!!
//   |_________________________________|
//
//    ¡ LA RELACION DE TRANSMISIÓN DEL SIN FIN CORONA ES DE 1/16, ES DECIR, PARA EJECUTAR UNA VUELTA COMPLETA DE MOTOR,  ESTE ESTA DADO POR EL PRODUCTO DE 16 * 200 = 3200  !
//
// WS2812 LED PIN
// DATA_PIN ---> 4

//  GPIO PINS DISPONIBLES
//    2
//    13
//    15
//    23
/**************************************************************************************************************************************************************/
//  RUTINAS
//    - HOME POSITION
//    - REPAIR POSITION
//    - WELDING ROUTINE
/*************************************************************************************************************************************************************/
// Definimos el número de leds WS2812 a manipular
#define NUM_LEDS 1
// Definimos el PIN DATA para el control de color del led WS2812
#define DATA_PIN 4
// Define the array of leds
CRGB leds[NUM_LEDS];

AccelStepper stepper1(1, 32, 33);  //  J1   Linear movement   (Stepper motor)
AccelStepper stepper2(1, 25, 26);  //  J2   Angular movement  (Stepper motor)
AccelStepper stepper3(1, 27, 14);  //  J3   Angular movement  (Stepper motor)
AccelStepper stepper4(1, 12, 16);  //  J4   Angular movement  (Stepper motor)
AccelStepper stepper5(1, 17, 15);  //  J5   Angular movement  (Stepper motor)

static const int servoPin1 = 18;  //  J6   Angular movement (Servomotor)
static const int servoPin2 = 19;  //  J7   Angular movement (Servomotor)

Servo_ESP32 servo1;               //  Instancias para control de servomotores
Servo_ESP32 servo2;

void setup() {
  //  Configuramos LED WS2812
  FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);  // GRB ordering is typical

  servo1.attach(servoPin1);
  servo2.attach(servoPin2);

  //    CONFIGURACIÓN   DE  VELOCIDAD   Y   ACELERACION
  stepper1.setMaxSpeed(2000);      // X pasos/segundo
  stepper1.setAcceleration(1000);  // Set acceleration value for the stepper
  stepper1.setCurrentPosition(0);  // Set the current position to 0 steps

  stepper2.setMaxSpeed(2000);      // X pasos/segundo
  stepper2.setAcceleration(1000);  // Set acceleration value for the stepper
  stepper2.setCurrentPosition(0);  // Set the current position to 0 steps

  stepper3.setMaxSpeed(2000);      // X pasos/segundo
  stepper3.setAcceleration(1000);  // Set acceleration value for the stepper
  stepper3.setCurrentPosition(0);  // Set the current position to 0 steps

  stepper4.setMaxSpeed(2000);      // X pasos/segundo
  stepper4.setAcceleration(1000);  // Set acceleration value for the stepper
  stepper4.setCurrentPosition(0);  // Set the current position to 0 steps

  stepper5.setMaxSpeed(2000);      // X pasos/segundo
  stepper5.setAcceleration(1000);  // Set acceleration value for the stepper
  stepper5.setCurrentPosition(0);  // Set the current position to 0 steps

  Blink_Blue();                    // Ejecutamos rutina de parpadeo en azul para dar inicio a al menú seleccionable.
}


void loop() {
  
}


void Home_position() {
  stepper1.moveTo(0);    // NOTA: 3200 PASOS/REV
  stepper2.moveTo(0);
  stepper3.moveTo(0);
  stepper4.moveTo(0);
  stepper5.moveTo(0);

  while (stepper2.distanceToGo() != 0 || stepper1.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0 || stepper5.distanceToGo() != 0) {  //  TESTEAR CON currentposition();   (stepper2.distanceToGo() != 0 || stepper1.distanceToGo() != 0)
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
    stepper5.run();
  }
}

void Repair_Position() {
  stepper1.moveTo(0);
  stepper2.moveTo(0);
  stepper3.moveTo(0);
  stepper4.moveTo(0);
  stepper5.moveTo(0);

  while (stepper2.distanceToGo() != 0 || stepper1.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0 || stepper5.distanceToGo() != 0) {  //  TESTEAR CON currentposition();   (stepper2.distanceToGo() != 0 || stepper1.distanceToGo() != 0)
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
    stepper5.run();
  }
}
//  106.9 - 19.4 mm = 87.5 mm para librar acrilico  35,000 pasos
//  272 mm para maximo recorrido  108,800 pasos
//  PODEMOS JUGAR CON LA VELOCIDAD PARA HACER QUE LOS MOTORES STEPPER PAREZCAN QUE TERMINAN TODOS A LA VEZ
void Welding_Routine() {
  Blink_Green()
  stepper1.moveTo(0);
  stepper2.moveTo(0);
  stepper3.moveTo(0);
  stepper4.moveTo(0);
  stepper5.moveTo(0);

  while (stepper2.distanceToGo() != 0 || stepper1.distanceToGo() != 0 || stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0 || stepper5.distanceToGo() != 0) {  //  TESTEAR CON currentposition();   (stepper2.distanceToGo() != 0 || stepper1.distanceToGo() != 0)
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
    stepper5.run();
  }
}

void Blink_Green() {
  for (int i = 0; i <= 2; i++) {
    // Turn the LED on, then pause
    leds[0] = CRGB::Green;    //  Encendemos de color verde el WS2812
    FastLED.show();           //  Accionamos el Led
    delay(500);               //  Ponemos delay por medio segundo
    // Now turn the LED off, then pause
    leds[0] = CRGB::Black;
    FastLED.show();
    delay(500);
  }
}

void Blink_Red() {
  for (int i = 0; i <= 2; i++) {
    // Turn the LED on, then pause
    leds[0] = CRGB::Red;      //  Encendemos de color rojo el WS2812
    FastLED.show();           //  Accionamos el Led
    delay(500);               //  Ponemos delay por medio segundo
    // Now turn the LED off, then pause
    leds[0] = CRGB::Black;
    FastLED.show();
    delay(500);
  }
}

void Blink_yellow(){
  for (int i = 0; i <= 2; i++) {
    // Turn the LED on, then pause
    leds[0] = CRGB::Yellow;      //  Encendemos de color rojo el WS2812
    FastLED.show();           //  Accionamos el Led
    delay(500);               //  Ponemos delay por medio segundo
    // Now turn the LED off, then pause
    leds[0] = CRGB::Black;
    FastLED.show();
    delay(500);
  }
}

void Blink_Blue(){
  for (int i = 0; i <= 2; i++) {
    // Turn the LED on, then pause
    leds[0] = CRGB::Blue;      //  Encendemos de color rojo el WS2812
    FastLED.show();            //  Accionamos el Led
    delay(250);                //  Ponemos delay por medio segundo
    // Now turn the LED off, then pause
    leds[0] = CRGB::Black;
    FastLED.show();
    delay(250);
  }  
}
