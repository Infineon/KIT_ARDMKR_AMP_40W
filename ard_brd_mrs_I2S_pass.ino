/*----------------------------------------------------------
 * Title:   ard_brd_mrs_I2S_pass_20200929
 * Author:  Rien Oortgiesen
 * Company: Infineon
 * 
 * Description:
 * This code demonstrates external I2S input to be used with 
 * KIT_ARDMKR_AMP_40W. The board should be used together 
 * with an Arduino MKRZERO (or Arduino board with similar pinout)
 * The code below takes care of MA12070P (amplifier) config. 
 * as well as power management (DCDC boost) and voume control.
 *  
 * Revisions: 
 * 20200929: intial version
 *  
 * This code is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include <Wire.h>                                // using wire for I2C comm.

 /*-------- Variable declaration ---------*/
  int pmp_sel;
  int curr_vol;
  int GP_LED = A4;
  int VOLUME = A1;
  int volume = 0;
  int volume_curr = 0;
  int ana_val = 0;

  int enable_n = 5; // /AMP ENABLE connected to PD6
  int mute_n = 4; // /AMP MUTE connected tp PD7
  // note schematic mute enable naming bug

  const byte interruptPin6 = 6; //Push Button 2
  const byte interruptPin7 = 7; //Push Button 1
  const byte interruptPin0 = 0; //Push Button 3

  const byte interruptPin1 = 1; //Amp /ERROR pin
  
void setup ()
{
  pinMode(enable_n, OUTPUT);
  pinMode(mute_n, OUTPUT);
  
  pinMode(A3, OUTPUT);
  pinMode(GP_LED, OUTPUT);
  pinMode(A5, INPUT); //high-z to work-around MSEL bug
  pinMode(5, OUTPUT);

  // set I2S pins high-z
  pinMode(3, INPUT);
  pinMode(2, INPUT);
  pinMode(A6, INPUT);

  // Start-up sequence
  // make sure everything is disabled init state
  digitalWrite(mute_n, LOW);
  digitalWrite(enable_n, HIGH);
  //digitalWrite(A3, LOW); //boost disable
  delay(1000);
  digitalWrite(enable_n, LOW);                                   
  delay(1000);

  // start-up loop for DCDC booster
  // still needs fine tuning because of cap pre-charging
  for (int i = 0; i<5; i++)
  { 
    digitalWrite(A3, HIGH); //boost enable
    delay(100);
    digitalWrite(A3, LOW); //boost disable
    delay(100);
  }
  digitalWrite(A3, HIGH); //boost enable
  delay(1000);
  digitalWrite(enable_n, LOW);
                                                                     
  analogWrite(GP_LED, volume);                       
  analogReadResolution(6);
  
  delay(100);
                                  
  Wire.begin();  // begin I2C communication       
  
  // set i2s dataformat and enable processor  
  Wire.beginTransmission(byte(0x20)); 
  Wire.write(byte(0x35));        
  Wire.write(byte(0x09));              
  Wire.endTransmission();

  // -60dB master volume 
  Wire.beginTransmission(byte(0x20)); 
  Wire.write(byte(0x40));        
  Wire.write(byte(0x54));              
  Wire.endTransmission(); 

/* ------- Clear Error Handler --------
   *  Error registers need to be cleared 
   *  at start-up. This is done by toggle
   *  eh_clear (0x2D) bit 2.
   *  ---------------------------------- */
  Wire.beginTransmission(byte(0x20)); 
  Wire.write(byte(0x2D));        
  Wire.write(byte(0x34));              
  Wire.endTransmission();   

  Wire.beginTransmission(byte(0x20)); 
  Wire.write(byte(0x2D));        
  Wire.write(byte(0x30));              
  Wire.endTransmission(); 

  delay(500);
  digitalWrite(mute_n, HIGH); //AMP unmute 

}
                            
void loop ()
{
  volume = analogRead(VOLUME);  // read the input pin
  if ( volume != volume_curr )
  {
    volume_curr = volume;
    change_volume();
  }
} 

void change_volume ()
{
  /* ------- Volume control --------
   *  Volume interfacing is done through 
   *  analog potentio meter value read.
   *  Volume is set directly on the amp by 
   *  using the volume control registers
   *  ---------------------------------- */
  analogWrite(GP_LED, volume);
  Wire.beginTransmission(byte(0x20)); 
  Wire.write(byte(0x40));        
  Wire.write(byte(84-volume));              
  Wire.endTransmission();
}
