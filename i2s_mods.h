/* Audio Logger for Teensy 3.6
 * Copyright (c) 2018, Walter Zimmer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef _I2S_MODS_H
#define _I2S_MODS_H
/*
 * NOTE: changing frequency impacts the macros 
 *      AudioProcessorUsage and AudioProcessorUsageMax
 * defined in stock AudioStream.h
 */
 
#include "kinetis.h"
#include "core_pins.h"

// attempt to generate dividers programmatically 
// always better to check 
void I2S_dividers(uint32_t *iscl, uint32_t fsamp, uint32_t nbits) 
{ 
    int64_t i1 = 1; 
    int64_t i2 = 1; 
    int64_t i3 = iscl[2]+1;
    int fcpu=F_CPU; 
    if(F_CPU<=96000000) fcpu=96000000; 
    float A=fcpu/2.0f/i3/(2.0f*nbits*fsamp); 
    float mn=1.0;  
    for(int ii=1;ii<=128;ii++)  
    { float xx; 
      xx=A*ii-(int32_t)(A*ii);  
      if(xx<mn && A*ii<256.0) { mn=xx; i1=ii; i2=A*ii;} //select first candidate 
    } 
    iscl[0] = (int) (i1-1); 
    iscl[1] = (int) (i2-1); 
    iscl[2] = (int) (i3-1); 
} 
 
void I2S_modification(uint32_t fsamp, uint16_t nbits) 
{ uint32_t iscl[3]; 

  iscl[2]=1; // this is good for PJRC sgtl5000 // could b
  
  I2S_dividers(iscl, fsamp ,nbits); 
  int fcpu=F_CPU; 
  if(F_CPU<=96000000) fcpu=96000000; 
  float mclk = fcpu * (iscl[0]+1.0f) / (iscl[1]+1.0f);
  float fs = (fcpu * (iscl[0]+1.0f)) / (iscl[1]+1.0f) / 2.0f / (iscl[2]+1.0f) / (2.0f*nbits); 
  #if DO_DEBUG >0 
    Serial.printf("%d %d: %d %d %d %d %d %d %d\n\r", 
        F_CPU, fcpu, fsamp, (int)fs, (int) mclk, nbits,iscl[0]+1,iscl[1]+1,iscl[2]+1); 
  #endif 
 
  // stop I2S 
  I2S0_RCSR &= ~(I2S_RCSR_RE | I2S_RCSR_BCE); 
 
  // modify sampling frequency 
  I2S0_MDR = I2S_MDR_FRACT(iscl[0]) | I2S_MDR_DIVIDE(iscl[1]); 

  // configure transmitter 
  I2S0_TCR2 = I2S_TCR2_SYNC(0) | I2S_TCR2_BCP | I2S_TCR2_MSEL(1) 
    | I2S_TCR2_BCD | I2S_TCR2_DIV(iscl[2]); 

  // configure receiver (sync'd to transmitter clocks) 
  I2S0_RCR2 = I2S_RCR2_SYNC(1) | I2S_TCR2_BCP | I2S_RCR2_MSEL(1) 
    | I2S_RCR2_BCD | I2S_RCR2_DIV(iscl[2]); 
 
  //restart I2S 
  I2S0_RCSR |= I2S_RCSR_RE | I2S_RCSR_BCE; 
} 

// ********************************************** following is to change SGTL5000 samling rates ********************
#define SGTL5000_I2C_ADDR  0x0A  // CTRL_ADR0_CS pin low (normal configuration)
#define CHIP_CLK_CTRL     0x0004
#define CHIP_I2S_CTRL     0x0006

#include "Wire.h"
unsigned int chipRead(unsigned int reg)
{
  unsigned int val;
  Wire.beginTransmission(SGTL5000_I2C_ADDR);
  Wire.write(reg >> 8);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(SGTL5000_I2C_ADDR, 2) < 2) return 0;
  val = Wire.read() << 8;
  val |= Wire.read();
  return val;
}

bool chipWrite(unsigned int reg, unsigned int val)
{
  Wire.beginTransmission(SGTL5000_I2C_ADDR);
  Wire.write(reg >> 8);
  Wire.write(reg);
  Wire.write(val >> 8);
  Wire.write(val);
  if (Wire.endTransmission() == 0) return true;
  return false;
}

unsigned int chipModify(unsigned int reg, unsigned int val, unsigned int iMask)
{
  unsigned int val1 = (chipRead(reg)&(~iMask))|val;
  if(!chipWrite(reg,val1)) return 0;
  return val1;
}

void SGTL5000_modification(uint32_t fs_mode)
{ if(fs_mode>3) fs_mode = 3;
  chipWrite(CHIP_CLK_CTRL, fs_mode<<2);  // 256*Fs fs_mode = 0:32 kHz; 1:44.1 kHz; 2:48 kHz; 3:96 kHz 
}

#endif
