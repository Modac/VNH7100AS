# Arduino VNH7100AS motor controller library
Arduino library for [VNH7100AS](https://www.st.com/en/automotive-analog-and-power/vnh7100as.html) based motor controller boards. The VNH7100AS chips contain a full H bridge to drive motors with currents up to 12A.

## Getting Started

To install this library use the **Clone or download > Download ZIP** button on the repository home page and then install the library in your Arduino environment using **Sketch > Include Library > Add .ZIP Library...**

This library supports all AVR based Arduino boards that support the ```analogWrite()``` function to generate PWM signals (most boards). If you install the ```ESP_AnalogWrite``` library, this library also works with ESP32 boards (not yet fully tested). Supports for STM32 boards: library compiles for this environment, but is not yet tested.

## VNH7100AS interfaces

There are two types of interfaces:
- **control** interface to control the motor (3 pins: INA, INB, PWM)
- **current measurement** interface (2 pins: SEL0, CS)

### Control interface

Most boards have **3** key **control** lines:
- PWM - duty cycle to control motor speed or brake power
- INA + INB - to control the function:

| INA | INB | Function      |
|-----|-----|---------------|
| 1   | 1   | brake to Vcc  |
| 0   | 0   | brake to Gnd  |
| 1   | 0   | forward spin  |
| 0   | 1   | backward spin |

The control lines need to be connected to a digital output of your Arduino board.
**Tip**: If you want to reverse the motor (forwards becomes backwards), you can either use negative values for the ```speed()``` function or simply reverse the INA and INB pins when calling the ```begin()``` function.

### Current sense interface

The **current sense** pin (typically marked as CS) provides a current proportional to the motor current. 
The **SEL0** pin is used to control the CurrentSense information present on the CS pin.
The factor varies for each chip due to tolerance differences, but in general the value for VNH7100AS chips is around 1200. Most boards already have a resistor of 1.5k between the CS line and GND to translate the current into a voltage the Arduino can measure (using ```analogRead()```). An additional RC circuit may be present to further stabilize the CS signal. This pin should be connected to an analog input pin of your Arduino and specified as ```csPin``` when calling the ```begin()``` function.

**Example**: if the motor current in your setup is 1A, the CS pin of the chip will provide a current of 1/1200 = 0.00083 A = 0.83 mA. With a board value of 1.5k = 1500 ohm, the voltage will be 0.83 * 1500 = 1250 mV = 1.25 Volt. The ```motorCurrent()``` function uses ```analogRead()``` to read this value, which means the value returned depends on the setting of ```analogReference()``` and the working voltage of your Arduino board:
- If your Arduino runs at 5V and ```analogReference()``` is set to ```DEFAULT```, ```motorCurrent()``` will return a value of 1.25 / 5 * 1023 = 256 for the above case
- If you first change your analog reference setting with ```analogReference(INTERNAL2V56)```, the ```motorCurrent()``` function will return a value of 1.25 / 2.56 * 1023 = 500.

**Tip**: It is always good practice to use ```analogReference()``` with one of the internal reference voltage options when you want to use ```analogRead()``` as the Arduino Vcc power may not be stable. This will improve stability of your readings.

**Tip**: You do not need to connect the current sense interface and can supply ```-1``` for ```csPin``` and ```sel0Pin``` when calling the ```begin()``` function. If either one of them is ```-1``` the current sense interface will not be available.

## Example code

This example shows all basic functions controlling a single motor. The example is also provided in the **Single.ino** sketch.

```
#include <VNH7100AS.h>

VNH7100AS Motor1;    // define control object for 1 motor

// motor pins
#define M1_PWM 3    // pwm pin motor (digital output)
#define M1_INA 4    // control pin INA (digital output)
#define M1_INB 5    // control pin INB (digital output)
#define M1_SEL0 6   // diagnose pins (combined DIAGA/ENA and DIAGB/ENB - digital input)
#define M1_CS A0    // current sense pin (analog input)

void setup() {
  Motor1.begin(M1_PWM, M1_INA, M1_INB, M1_SEL0, M1_CS);    // Motor 1 object connected through specified pins 
  Serial.begin(115200);   
}

void loop() {
  Serial.println("3/4 speed forward");
  Motor1.setSpeed(300); // motor 3/4-speed "forward"
  delay(2000); // wait for 2 seconds
  Serial.print("Current="); Serial.println(Motor1.motorCurrent());

  Serial.println("Motor stop (coast)");
  Motor1.setSpeed(0); // motor stop (coasting)
  delay(2000); // wait for 2 seconds
  Serial.print("Current at stop="); Serial.println(Motor1.motorCurrent());
 
  Serial.println("Half speed backward");
  Motor1.setSpeed(-200); // motor half-speed "backward"
  delay(2000); // wait for 2 seconds
 
  Serial.println("Motor stop (coast)");
  Motor1.setSpeed(0); // motor stop 
  delay(2000); // wait for 2 seconds

  Serial.println("Full speed backward");
  Motor1.setSpeed(-400); // motor full-speed "backward"
  delay(2000); // wait for 2 seconds
  Serial.print("Current="); Serial.println(Motor1.motorCurrent());

  Serial.println("Brake at 3/4 power");
  Motor1.brake(300); // motor brake at 3/4 power
  delay(10);
  Serial.print("Current during brake="); Serial.println(Motor1.motorCurrent());
  delay(4000); // wait for 4 seconds
  Serial.print("Current after brake="); Serial.println(Motor1.motorCurrent());
}
```

## Setting speed and braking

A motor controller will provide a fixed voltage to the motor, depending on the speed setting and the supply voltage of the VNH7100AS board. A ```setSpeed()``` setting of ```400``` or ```-400``` will provide the full supply voltage. A speed setting of ```0``` is equivalent to a non-connected motor. This means a motor will run free and the vehicle will not brake (except due to internal motor friction).

The ```brake()``` function will force the motor to a halt. The braking power can be ```0```, which is free run and the same as ```setSpeed(0)```. A setting of ```brake(400)``` represents maximum brake level and is equivalent to a motor where both power wires (black and red) are connected to each other: since a motor can act as dynamo, forcing the motor to turn will produce a current that immediately tries to stop it.

## Class member functions and data members

The VNH7100AS class exposes the following functions and internal variables:

```
- void begin(int8_t pwmPin, inaPin, inbPin, sel0Pin, csPin);
```
This initializes the library and allocates the defined pins. You can provide a ```-1``` for the ```sel0Pin``` and the ```csPin``` parameter if you do not want to use the current sense signals.

```
uint8_t setSpeed(int speed); 
```
Sets motor speed, returns true if success, returns false when VNH7100AS is in fault. Speed should be between ```-400``` and ```+400```. A speed of ```0``` means free run. Returns motor ```!isFault()``` (```true``` is ok).

```
uint8_t brake(int brakePower);
```
Brake motor. ```brakePower=0``` (or negative) is the same as ```setSpeed(0)``` - free run. If brakePower is positive, the motor will brake faster. A value of ```400``` is equivalent to full brake. Returns motor ```!isFault()``` (```true``` is ok).

```
uint8_t isFault(); 
```
Returns true if the voltage at ```csPin``` is at VsenseH (```true``` is fault, ```false``` means no fault). When a fault is present all pins will be set LOW. Always returns ```false``` when ```csPin``` or ```sel0Pin``` is set to ```-1```.

```
int motorCurrent();
```
Returns the value of ```analogRead()``` for the ```csPin```. This is a value proportional to the motor current. See above. Returns 0 when ``csPin``` or ```sel0Pin``` is set to ```-1```.

```
int speed;
```
This variable contains the current speed setting.

## Example sketches provided

Example sketches:

- **Single**: Controls a single motor (sketch as shown above).
- **Dual**: Controls two motors, including an illustration how to implement vehicle turning.
- **Robotcar**: Example remote controlled car using the IBus protocol
