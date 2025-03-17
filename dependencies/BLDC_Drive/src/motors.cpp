#include <SimpleFOC.h>
#include "SPI_Encoder.h"

#define LABELPRINT(label, value) Serial.print(label); Serial.print(": "); Serial.println(value);
#define CS_A A1
#define CS_B A2
#define PWM_A 0
#define PWM_B 1
#define PWM_C 2
#define EN 3
SPI_Encoder AS5047P = SPI_Encoder();
//MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI,10);
// Declare and init counters

float shunt_resistor = 0.01;
float gain = 50;//39.0;


// Init current sense modules
InlineCurrentSense cs_h = InlineCurrentSense(shunt_resistor, gain, CS_A, CS_B);//

// Init FOC motors and driver s
BLDCMotor motor_h = BLDCMotor(7, 1.6,3.5);  // Inits the motor. The first argument is the number of pole pairs physically in the motor; the EC-i 539482 has 4, the EC-max 272768 has 1. The second argument is motor terminal resistance
BLDCDriver3PWM driver_h = BLDCDriver3PWM(PWM_A, PWM_B, PWM_C, EN);  // Inits the PWM and enable channels for the motor. Arguments are pin numbers on the FOC shield. Do not change.

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor_h.target, cmd); }

void setup() {
  while(!Serial){
    ;
  }
  pinMode(CS_A,INPUT);
  pinMode(CS_B,INPUT);
  // check if pin22 has input "keeper"
	// volatile uint32_t *pad1 = portControlRegister(CS_A);
	// uint32_t padval1 = *pad1;
	// if ((padval1 & (IOMUXC_PAD_PUE | IOMUXC_PAD_PKE)) == IOMUXC_PAD_PKE) {
	// 	// disable keeper, as it messes up analog with higher source impedance
	// 	// but don't touch user's setting if ordinary pullup, which some
	// 	// people use together with capacitors or other circuitry
	// 	*pad1 = padval1 & ~IOMUXC_PAD_PKE;
	// }
  // volatile uint32_t *pad2 = portControlRegister(CS_B);
	// uint32_t padval2 = *pad2;
	// if ((padval2 & (IOMUXC_PAD_PUE | IOMUXC_PAD_PKE)) == IOMUXC_PAD_PKE) {
	// 	// disable keeper, as it messes up analog with higher source impedance
	// 	// but don't touch user's setting if ordinary pullup, which some
	// 	// people use together with capacitors or other circuitry
	// 	*pad2 = padval2 & ~IOMUXC_PAD_PKE;
	// }

  // ADC1_CFG = ADC_CFG_MODE(1)|ADC_CFG_ADSTS(3)|ADC_CFG_ADICLK(1)|ADC_CFG_ADHSC|ADC_CFG_ADLSMP;//10bit,IPG/2 clock speed, 
  // ADC1_GC = ADC_GC_CAL;
  // while (ADC1_GC>>7 & 0b1){
  //   ;
  // }

  // Link the motor to the sensor
  motor_h.linkSensor(&AS5047P);
  // sensor.init();
  // motor_h.linkSensor(&sensor);

  // Driver config
  driver_h.pwm_frequency = 20000;
  driver_h.voltage_power_supply = 24;
  driver_h.init();
  // Link Driver
  motor_h.linkDriver(&driver_h);
  // Setup current sensing
  cs_h.linkDriver(&driver_h);
  //cs_h.skip_align = true;  // prevent the FOC board from overriding user settings
  cs_h.init();
  motor_h.linkCurrentSense(&cs_h);

  // set torque mode:
  motor_h.torque_controller = TorqueControlType::foc_current;
  motor_h.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  motor_h.controller = MotionControlType::torque;
  // motor_h.sensor_direction = Direction::CW;      //Sets motor direction with respect to the sensor direction. Must be set for EC-max; autodetection will fail.
  // motor_h.zero_electric_angle = 0;     // since encoder is zeroed at the zero electrical angle, an angle zero search can be skipped.

  //foc current control parameters
  motor_h.PID_current_q.P = 5.0;
  motor_h.PID_current_q.I = 300.0;
  motor_h.PID_current_d.P = 5.0;
  motor_h.PID_current_d.I = 300.0;
  motor_h.LPF_current_q.Tf = 0.01f; // 1ms default
  motor_h.LPF_current_d.Tf = 0.01f; // 1ms default
  motor_h.current_limit = 2.0;
  Serial.begin(115200);
  motor_h.useMonitoring(Serial);
  motor_h.monitor_variables =  _MON_CURR_Q|_MON_TARGET;//_MON_CURR_D;
  motor_h.init();
  motor_h.initFOC();
  // Serial.println("A:");
  // Serial.println(cs_h.gain_a);
  motor_h.enable();
  _delay(2000);
}

void loop() {

  // main FOC algorithm function
  motor_h.loopFOC();
  // Motion control function
  motor_h.move(0.3);
  motor_h.monitor();

}
