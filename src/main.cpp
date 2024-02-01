// Project: Arduino-based RC Car with IR Remote Control
// Description: This code controls an RC car using an Arduino board. It
// interprets IR remote signals for navigation and employs ultrasonic sensors
// for distance measurement.
// Hardware Components: Arduino Mega 2560, L298N Motor Driver, 2x DC Motors,
// HC-SR04 Ultrasonic Sensor, Generic IR Remote.
// Functionality: The car can move forwards, backwards, and turn left/right
// based on IR remote commands. It includes obstacle avoidance using the
// ultrasonic sensor.
// Author: VAlexandersson
// Date: ~2023
// Version: 1.0

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <IRremote.h>
#include <event_groups.h>

#define debug(x) Serial.print(x);
#define debugln(x) Serial.println(x);

enum RemoteCodes {
  UP = 0x00FF18E7,
  DOWN = 0x00FF4AB5,
  LEFT = 0x00FF10EF,
  RIGHT = 0x00FF5AA5,
  ONE = 0x00FFA25D,
  TWO = 0x00FF629D,
  THREE = 0x00FFE21D,
  BLANK = 0xFFFFFFFF
};

enum Pins {
  L_SPEED = 13,
  L_FORWARD = 12,
  L_REVERSE = 11,
  R_SPEED = 8,
  R_FORWARD = 10,
  R_REVERSE = 9,
  TRIG_PIN = 7,
  ECHO_PIN = 6,
  IR_RECEIVER = 5
};

enum Bits {
  BIT0 = (1 << 0),
  BIT1 = (1 << 1),
  BIT2 = (1 << 2),
  BIT3 = (1 << 3),
  BIT4 = (1 << 4),
  BIT5 = (1 << 4)
};

// Global Variable Declarations
IRrecv receiver(IR_RECEIVER);
decode_results results;

uint32_t g_previousCommand, g_measuredDistance, g_motorSpeed = 50;

EventGroupHandle_t chauffuer_eventgroup;

TaskHandle_t h_irremotetask = NULL;
TaskHandle_t h_distancetask = NULL;
TaskHandle_t h_leftmotortask = NULL;
TaskHandle_t h_rightmotortask = NULL;

typedef struct Motor_t {
  uint8_t speedPin, forwardPin, reversePin;
  uint8_t bits;
  TaskHandle_t taskHandle;
} Motor_t;

Motor_t leftMotor = {L_SPEED, L_FORWARD, L_REVERSE, BIT0 | BIT2,
                     h_leftmotortask};
Motor_t rightMotor = {R_SPEED, R_FORWARD, R_REVERSE, BIT1 | BIT3,
                      h_rightmotortask};

// Function Prototypes
void setupGPIOPins();
void setupTasks();
void vMeasuredDistanceTask(void *p);
void vControlMotorTask(void *p);
void vProcessIRRemoteTask(void *p);

// Setup Function
void setup() {
  Serial.begin(9600);
  receiver.enableIRIn();
  setupGPIOPins();
  chauffuer_eventgroup = xEventGroupCreate();
  setupTasks();
}
void loop() {}

// Function Definitions
void setupGPIOPins() {
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  pinMode(leftMotor.speedPin, OUTPUT);
  pinMode(leftMotor.forwardPin, OUTPUT);
  pinMode(leftMotor.reversePin, OUTPUT);

  pinMode(rightMotor.speedPin, OUTPUT);
  pinMode(rightMotor.forwardPin, OUTPUT);
  pinMode(rightMotor.reversePin, OUTPUT);

  digitalWrite(TRIG_PIN, LOW);

  digitalWrite(leftMotor.forwardPin, LOW);
  digitalWrite(leftMotor.reversePin, LOW);
  digitalWrite(rightMotor.forwardPin, LOW);
  digitalWrite(rightMotor.reversePin, LOW);

  analogWrite(leftMotor.speedPin, g_motorSpeed);
  analogWrite(rightMotor.speedPin, g_motorSpeed);
}

void setupTasks() {
  xTaskCreate(vProcessIRRemoteTask, "IRRemoteTask", 256, NULL,
              tskIDLE_PRIORITY + 1, &h_irremotetask);
  xTaskCreate(vMeasuredDistanceTask, "DistanceTask", 256, NULL,
              tskIDLE_PRIORITY + 1, &h_distancetask);
  xTaskCreate(vControlMotorTask, "Right Motor", 64, &leftMotor,
              tskIDLE_PRIORITY + 1, &h_leftmotortask);
  xTaskCreate(vControlMotorTask, "Left Motor", 64, &rightMotor,
              tskIDLE_PRIORITY + 1, &h_rightmotortask);
  vTaskStartScheduler();
}

// Measures the distance using ultrasonic sensor and updates g_measuredDistance
void vMeasuredDistanceTask(void *p) {
  digitalWrite(TRIG_PIN, LOW);
  while (1) {

    xEventGroupWaitBits(chauffuer_eventgroup, BIT4, pdTRUE, pdTRUE,
                        portMAX_DELAY);

    digitalWrite(TRIG_PIN, HIGH);
    vTaskDelay(1);
    digitalWrite(TRIG_PIN, LOW);

    int dist = pulseIn(ECHO_PIN, HIGH);
    g_measuredDistance = (dist * 0.0172);
    debugln(g_measuredDistance);
  }
}

// Controls the motor based on the current commands and sensor readings
void vControlMotorTask(void *pvParameter) {
  Motor_t motor = (*(Motor_t *)pvParameter);
  EventBits_t bit;
  while (1) {
    bit = xEventGroupWaitBits(chauffuer_eventgroup, motor.bits, pdTRUE, pdFALSE,
                              portMAX_DELAY);

    analogWrite(motor.speedPin, g_motorSpeed);
    if (bit & (BIT0 | BIT1)) {
      if (g_measuredDistance > 36) {
        digitalWrite(motor.forwardPin, HIGH);
      }

    } else {
      digitalWrite(motor.reversePin, HIGH);
    }
    vTaskDelay(7);

    (bit & (BIT0 | BIT1)) ? digitalWrite(motor.forwardPin, LOW)
                          : digitalWrite(motor.reversePin, LOW);
  }
}

// Processes IR remote input and sets control flags and motor speed
void vProcessIRRemoteTask(void *p) {
  while (1) {
    EventBits_t bits = 0;
    if (receiver.decode(&results)) {

      if (results.value == BLANK) {
        results.value = g_previousCommand;
      }
      switch (results.value) {
      case UP:
        bits = BIT0 | BIT1 | BIT4;
        break;
      case DOWN:
        bits = BIT2 | BIT3;
        break;
      case LEFT:
        bits = BIT2;
        break;
      case RIGHT:
        bits = BIT3;
        break;

      case ONE:
        g_motorSpeed = 50;
        break;
      case TWO:
        g_motorSpeed = 100;
        break;
      case THREE:
        g_motorSpeed = 150;
        break;

      default:
        break;
      }
      g_previousCommand = results.value;
      if (bits & leftMotor.bits)
        debugln("LEFT");
      if (bits & rightMotor.bits)
        debugln("RIGHT");
      if (bits) {
        debug("g_d ");
        debugln(g_measuredDistance);
        xEventGroupSetBits(chauffuer_eventgroup, bits);
      }
      receiver.resume();
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}
