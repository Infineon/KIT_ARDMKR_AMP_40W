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
 * as well as power management (DCDC boost) and volume control.
 *  
 * Revisions: 
 * 20200929: intial version
 *  
 * This code is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */
#include <SD.h>
#include <ArduinoSound.h>

// filename of wave file to play
String filename[6] = {"track1.wav","track2.wav","track3.wav", "track4.wav","track5.wav","track6.wav"};
int nr = 0;
// variable representing the Wave File
SDWaveFile waveFile;

#include <Wire.h>                                // plug-in your MCU specific I2C library

 /*-------- Variable declaration ---------*/
  int pmp_sel;
  int curr_vol;
  int GP_LED = A4;
  int VOLUME = A1;
  int volume = 0;
  int volume_curr = 0;
  int ana_val = 0;

  int enable_n = 5;                             // /ENABLE connected to PD6
  int mute_n = 4;                               // /MUTE connected tp PD7
  // note schematic mute enable naming bug

  const byte interruptPin6 = 6;                                                    //Push Button 2
  const byte interruptPin7 = 7;                                                    //Push Button 1
  const byte interruptPin0 = 0;                                                    //Push Button 3

  const byte interruptPin1 = 1;                                                    //Amp /ERROR pin
  
void setup ()
{
  pinMode(enable_n, OUTPUT);
  pinMode(mute_n, OUTPUT);
  
  pinMode(A3, OUTPUT);
  pinMode(GP_LED, OUTPUT);
  pinMode(A5, INPUT); //high-z to work-around MSEL bug
  pinMode(5, OUTPUT);

  // Start-up sequence
  // make sure everything is disabled init state
  digitalWrite(mute_n, LOW);
  digitalWrite(enable_n, HIGH);
  //digitalWrite(A3, LOW); //boost disable
  delay(1000);
  digitalWrite(enable_n, LOW);                                   
  delay(1000);

  // start-up loop for booster
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
  Wire.write(byte(0x08));              
  Wire.endTransmission();

  // set 32fs bitclock for 16bits data
  Wire.beginTransmission(byte(0x20)); 
  Wire.write(byte(0x36));        
  Wire.write(byte(0x11));              
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

  // init SD card
  sd_card_read();
   
  Serial.begin(9600);
  Serial.println("init done :)");

}
  
void sd_card_read ()
{

  // setup the SD card, depending on your shield of breakout board
  // you may need to pass a pin number in begin for SS
  
  if (!SD.begin()) {
    return;
  }
  
  // create a SDWaveFile
  waveFile = SDWaveFile(filename[nr]);

  // check if the WaveFile is valid
  if (!waveFile) {
    while (1); // do nothing
  }

   // adjust the playback volume
  AudioOutI2S.volume(100);

  // check if the I2S output can play the wave file
  if (!AudioOutI2S.canPlay(waveFile)) {
    while (1); // do nothing
  }

  // start playback
   AudioOutI2S.play(waveFile);
//
  
  delay(500);
  digitalWrite(mute_n, HIGH); //unmute  
//      
}  // end of setup

void loop ()
{
  volume = analogRead(VOLUME);  // read the input pin
  if ( volume != volume_curr )
  {
    volume_curr = volume;
    change_volume();
  }
  // check if playback is still going on
  if (!AudioOutI2S.isPlaying()) {
    // playback has stopped
    digitalWrite(mute_n, LOW);            //mute immediately to prevent high-z input noise
    nr++;                                 //+1 nr. on tracklist
    sd_card_read();                       //read new track
  }
} 

void change_volume ()
{
  analogWrite(GP_LED, volume);
  Wire.beginTransmission(byte(0x20)); 
  Wire.write(byte(0x40));        
  Wire.write(byte(84-volume));              
  Wire.endTransmission();
  //Serial.println(24+volume);
}
