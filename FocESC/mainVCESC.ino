#include <SimpleFOC.h>

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 8);

// encoder instance
//Encoder encoder = Encoder(2, 3, 500);
// channel A and B callbacks
//void doA(){encoder.handleA();}
//void doB(){encoder.handleB();}
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// instantiate the commander
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() { 
  
  // initialize encoder sensor hardware
  //encoder.init();
  //encoder.enableInterrupts(doA, doB); 
  // link the motor to the sensor
  //motor.linkSensor(&encoder);

  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // set the torque control type
  motor.phase_resistance = 0.9; // 12.5 Ohms
  motor.torque_controller = TorqueControlType::voltage;
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command M
  command.add('M', doMotor, "motor");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target current using serial terminal:"));
  _delay(1000);
}

void loop() {

  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  // user communication
  command.run();
}
