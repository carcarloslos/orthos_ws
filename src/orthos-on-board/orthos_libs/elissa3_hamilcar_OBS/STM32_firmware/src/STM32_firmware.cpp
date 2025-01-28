/*
MCMU firmware for hamilcar

This code is to be implemented with a platform.io project. 
Contact: christian.niebuhr@tu-bs.de
*/
#include <Arduino.h>
#include <Servo.h>

/*
Pinout
| Pin Label (in sheet and SW) | Label (on PCB) | Hardware Pin Nr. | Arduino/BluePill Pin Nr. | FAM column number |
| --------------------------- | -------------- | ---------------- | ------------------------ | ----------------- |
| pwm0                        | +X+Z           | PA10             |                          | 1                 |
| pwm1                        | +X-Z           | PA9              |                          | 2                 |
| pwm2                        | -X+Z           | PB8              |                          | 3                 |
| pwm3                        | -X-Z           | PA8              |                          | 4                 |
| pwm4                        | +Y+Z           | PB1              |                          | 5                 |
| pwm5                        | +Y-Z           | PB0              |                          | 6                 |
| pwm6                        | -Y+Z           | PA6              |                          | 7                 |
| pwm7                        | -Y-Z           | PA7              |                          | 8                 |

*/

// these pins have to be ordered according to the FAM column number!
int pwm_pins[8]{PA10,PA9,PB8,PA8,PB1,PB0,PA6,PA7};

// these hold servo objects
Servo pwm_obj[8];

//max pwm signal. This is clipped to prevent any damaging of the motors
int max_pwm=1500;

// communication input buffer
char in_buf[8*sizeof(float)];

// duty_cycle buffer
float dc_buf[8]   ={0,0,0,0,0,0,0,0};


void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    //pinMode(3,OUTPUT);
    //pinMode()

  //attach servos
  for (int i=0; i<8; i++){
    pwm_obj[i].attach(pwm_pins[i]);
  }

  //set duty cycle to min for a certain time to enable ESCs
  for (int i=0; i<8; i++){
    pwm_obj[i].writeMicroseconds(1000);
  }
  delay(2000);
}
  
void loop() {
  


  //receive pwm values via serial
  if (Serial.available() > 0) {
    Serial.readBytes(in_buf,sizeof(in_buf));
    memcpy(dc_buf,in_buf,sizeof(in_buf));
    //delay(100);
     
    //for now, just return the target val
    Serial.write((uint8_t*)dc_buf,sizeof(dc_buf));
  }
  Serial.write("test");

  //set pwm values
  for (int i=0; i<8; i++){
    if (dc_buf[i]<max_pwm){
      pwm_obj[i].writeMicroseconds(dc_buf[i]);
    }
    else{
      pwm_obj[i].writeMicroseconds(max_pwm);
    }
  }

}