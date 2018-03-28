/****************************************************************************/
//
// TCS3414Lib is an Arduino library to communicate to and obtain values from
// the TCS3414 RGB color sensor.
//
// This work took minor parts (declarations)
// from the existing code by FrankieChu from www.seeedstudio.com
// available at https://github.com/Seeed-Studio/Grove_I2C_Color_Sensor

// Copyright (C) 2014, J.F. Omhover (jf.omhover@gmail.com)
//
// This file is part of TCS3414Lib
//
// TCS3414Lib is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// TCS3414Lib is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with TCS3414Lib.  If not, see <http://www.gnu.org/licenses/>.
//
/******************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include "TCS3414Lib.h"

template <class WireType>
void _writeReg(WireType wire, byte reg, byte val) {
  wire.beginTransmission(TCS3414_ADDR);
  wire.write(reg | CMD_WRITE | CMD_TRANSACTION_BYTE);  // systematically use CMD_WRITE bit to write values into registers
  wire.write(val);
  wire.endTransmission();
}

template <class WireType>
TCS3414<WireType>::TCS3414(WireType wire) {
  this->myWire = wire;
}

template <class WireType>
void TCS3414<WireType>::init(int mode) {
  switch(mode) {
  case TCS3414_LEVELINTERRUPTMODE:
    // the process below will set an interrupt each time the value of the CLEAR channel goes above 65530 (overflow, right ?)
    setLevelThreshold(REG_LOW_THRESH, 0x0000);
    setLevelThreshold(REG_HIGH_THRESH, 4000);
    _writeReg(this->myWire, REG_INT_SOURCE, INT_SOURCE_CLEAR); // set the interrupt source to the CLEAR channel
    break; 
  case TCS3414_FREEMODE:
  default:
    _writeReg(this->myWire, REG_TIMING, INTEG_MODE_FREE | INTEG_PARAM_INTTIME_12MS);
    _writeReg(this->myWire, REG_INT, INTR_CTL_DISABLE);
    setGain(GAIN_1, PRESCALER_1); // set default gain value and prescaler value
    //      _writeReg(REG_CTL, 0x03); // Enable ADCs; 
    break;
  };
}

template <class WireType>
void TCS3414<WireType>::start() {
  enableADC();
}

template <class WireType>
void TCS3414<WireType>::stop() {
  disableADC();
}

template <class WireType>
void TCS3414<WireType>::getRGB(uint16_t * red, uint16_t * green, uint16_t * blue, uint16_t * clr) {
  //    delay(500);

  this->myWire.beginTransmission(TCS3414_ADDR);
  this->myWire.write(CMD_WRITE | REG_BLOCK_READ);
  this->myWire.endTransmission();

  delay(20);
  this->myWire.beginTransmission(TCS3414_ADDR);
  this->myWire.requestFrom(TCS3414_ADDR, 8);
  while (this->myWire.available() < 8);  // TODO : do we really want to force to receive 8 bytes ???
  byte * b = (byte*)green;
  b[0] = this->myWire.read();
  b[1] = this->myWire.read();
  b = (byte*)red;
  b[0] = this->myWire.read();
  b[1] = this->myWire.read();
  b = (byte*)blue;
  b[0] = this->myWire.read();
  b[1] = this->myWire.read();
  b = (byte*)clr;
  b[0] = this->myWire.read();
  b[1] = this->myWire.read();
  this->myWire.endTransmission();
}

template <class WireType>
void TCS3414<WireType>::getValues(uint16_t * values) {
  //    delay(500);
  this->myWire.beginTransmission(TCS3414_ADDR);
  this->myWire.write(CMD_WRITE | REG_BLOCK_READ);
  this->myWire.endTransmission();

  this->myWire.beginTransmission(TCS3414_ADDR);
  this->myWire.requestFrom(TCS3414_ADDR, 8);
  while (this->myWire.available() < 8);  // TODO : do we really want to force to receive 8 bytes ???
  byte * b = (byte*)values;
  for (int i = 0; i < 8; i++) {
    b[i] = this->myWire.read();
    //Serial.println(readingdata[i],BIN);
  }
  this->myWire.endTransmission();
}

template <class WireType>
void TCS3414<WireType>::disableADC() {
  _writeReg(this->myWire, REG_CTL, CTL_POWER); 
}

template <class WireType>
void TCS3414<WireType>::enableADC() {
  _writeReg(this->myWire, REG_CTL, CTL_ADC_EN | CTL_POWER); 
}

template <class WireType>
void TCS3414<WireType>::powerOn() {
  _writeReg(this->myWire, REG_CTL, CTL_POWER); 
}

template <class WireType>
void TCS3414<WireType>::setLevelThreshold(byte reg, uint16_t thresh) {
  byte * b = (byte*)&thresh;
  this->myWire.beginTransmission(TCS3414_ADDR);
  this->myWire.write(reg | CMD_WRITE | CMD_TRANSACTION_WORD);
  this->myWire.write(b[0]);
  this->myWire.write(b[1]);
  this->myWire.endTransmission();
}

template <class WireType>
void TCS3414<WireType>::setIntegrationTime(byte itime) {
  //    this->itime = itime & 0x0F;
  _writeReg(this->myWire, REG_TIMING, itime);
}

template <class WireType>
void TCS3414<WireType>::setGain(byte gain, byte prescaler) {
  /*
    switch(gain) {
   case GAIN_4:
   this->gain = 4;
   break;
   case GAIN_16:
   this->gain = 16;
   break;
   case GAIN_64:
   this->gain = 64;
   break;
   case GAIN_1:
   default:
   this->gain = 1;
   break;
   };
   
   if (prescaler > 0)
   this->prescaler = 1 << prescaler;
   else
   this->prescaler = 1;
   */
  _writeReg(this->myWire, REG_GAIN, gain | prescaler);
}

template <class WireType>
void TCS3414<WireType>::clearInterrupt() {
  this->myWire.beginTransmission(TCS3414_ADDR);
  this->myWire.write(CMD_CLEARINT);
  this->myWire.endTransmission();
}

