/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  90 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  530 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;
uint8_t servo_center = 330;
int s0_save = servo_center;
int s1_save = servo_center;
int s2_save = servo_center;
int s3_save = servo_center;
int s4_save = servo_center;
int s5_save = servo_center;
int s6_save = servo_center;
int s7_save = servo_center;

const int iter = 17;

const int array_forward[iter][4] =  
{
{330,330,330,234},
{400,330,330,234},
{400,378,330,234},
{400,378,330,335},
{330,378,330,335},
{330,378,330,239},
{330,378,262,239},
{330,330,262,239},
{330,220,234,239},
{262,220,262,239},
{262,330,262,239},
{262,330,360,239},
{262,327,360,360},
{262,330,360,360},
{330,330,360,360},
{330,330,322,360},
{330,330,330,330}
};

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(26000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  //pwm.setPWM(0, 0, servo_center);
  pwm.writeMicroseconds(0, 1500);
  delay(1000);
  //pwm.setPWM(1, 0, servo_center);
  pwm.writeMicroseconds(1, 1500);
  delay(1000);
  //pwm.setPWM(2, 0, servo_center);
  pwm.writeMicroseconds(2, 1500);
  delay(1000);
  //pwm.setPWM(3, 0, servo_center);
  pwm.writeMicroseconds(3, 1500);
  delay(1000);
 
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void loop() {
  for(int x=0; x<iter; x++) {

    // s0
      int s0 = 0;
      if (array_forward[x][0] == s0_save) {
        Serial.println("Do nothing");
      } else if (array_forward[x][0] > s0_save) {
        
        for (s0 = s0_save; s0 <= array_forward[x][0]; s0++){
          Serial.print("Setting s0 to: ");Serial.println(s0);
          pwm.setPWM(0, 0, s0);
          delay(10);
        }
        s0_save = s0;
          
      } else {
        for (s0 = s0_save; s0 >= array_forward[x][0]; s0--){
          Serial.print("Setting s0 to: ");Serial.println(s0);
          pwm.setPWM(0, 0, s0);
          delay(10);
        }
        s0_save = s0;     
      }

      // s1
      int s1 = 0;
      if (array_forward[x][1] == s1_save) {
        Serial.println("Do nothing");
      } else if (array_forward[x][1] > s1_save) {
        
        for (s1 = s1_save; s1 <= array_forward[x][1]; s1++){
          Serial.print("Setting s1 to: ");Serial.println(s1);
          pwm.setPWM(1, 0, s1);
          delay(10);
        }
        s1_save = s1;
          
      } else {
        for (s1 = s1_save; s1 >= array_forward[x][1]; s1--){
          Serial.print("Setting s1 to: ");Serial.println(s1);
          pwm.setPWM(1, 0, s1);
          delay(10);
        }
        s1_save = s1;     
      }      

       // s2
      int s2 = 0;
      if (array_forward[x][2] == s2_save) {
        Serial.println("Do nothing");
      } else if (array_forward[x][2] > s2_save) {
        
        for (s2 = s2_save; s2 <= array_forward[x][2]; s2++){
          Serial.print("Setting s2 to: ");Serial.println(s2);
          pwm.setPWM(2, 0, s2);
          delay(10);
        }
        s2_save = s2;
          
      } else {
        for (s2 = s2_save; s2 >= array_forward[x][2]; s2--){
          Serial.print("Setting s2 to: ");Serial.println(s2);
          pwm.setPWM(2, 0, s2);
          delay(10);
        }
        s2_save = s2;     
      }     

       // s3
      int s3 = 0;
      if (array_forward[x][3] == s3_save) {
        Serial.println("Do nothing");
      } else if (array_forward[x][3] > s3_save) {
        
        for (s3 = s3_save; s3 <= array_forward[x][3]; s3++){
          Serial.print("Setting s3 to: ");Serial.println(s3);
          pwm.setPWM(3, 0, s3);
          delay(10);
        }
        s3_save = s3;
          
      } else {
        for (s3 = s3_save; s3 >= array_forward[x][3]; s3--){
          Serial.print("Setting s3 to: ");Serial.println(s3);
          pwm.setPWM(3, 0, s3);
          delay(10);
        }
        s3_save = s3;     
      }           
  }
}
