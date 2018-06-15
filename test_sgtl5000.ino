#include "core_pins.h"
#include "usb_serial.h"

#define DO_DEBUG 1
#define FSI 1 // desired sampling frequency index
uint32_t fsamps[] = {32000, 44100, 48000, 96000, 192000, 220500, 240000, 360000};
/*
 * NOTE: changing frequency impacts the macros 
 *      AudioProcessorUsage and AudioProcessorUsageMax
 * defined in stock AudioStream.h
 */

#define MQUEU 550 // number of buffers in aquisition queue
#define SEL_LR 1  // record only a single channel (0 left, 1 right)

  #include "input_i2s.h"
  AudioInputI2S         acq;

  #include "m_queue.h"
  mRecordQueue<int16_t, MQUEU> queue1;

  AudioConnection     patchCord1(acq,SEL_LR, queue1,0);

  #include "control_sgtl5000.h"
  AudioControlSGTL5000 audioShield;

// private 'library' included directly into sketch
#include "i2s_mods.h"
#include "logger_if.h"

//__________________________General Arduino Routines_____________________________________
extern "C" void setup() {
  // put your setup code here, to run once:

  while(!Serial);
  AudioMemory (600);
  audioShield.enable();
  audioShield.inputSelect(AUDIO_INPUT_LINEIN);  //AUDIO_INPUT_LINEIN or AUDIO_INPUT_MIC
   //
  I2S_modification(fsamps[FSI],32);

  delay(1);
  SGTL5000_modification(FSI); // must be called after I2S initialization stabilized (0:32 kHz, 1:44.1 kHz, 2:48 kHz, 3:96 kHz, 4:192 kHz)
  
  uSD.init();
  Serial.println("USD");

  Serial.println("start");
  queue1.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  static int16_t state=0; // 0: open new file, -1: last file

  if(queue1.available())
  {  // have data on que
    if(state==0)
    { // generate header before file is opened
       uint32_t *header=(uint32_t *) headerUpdate();
       uint32_t *ptr=(uint32_t *) outptr;
       // copy to disk buffer
       for(int ii=0;ii<128;ii++) ptr[ii] = header[ii];
       outptr+=256; //(512 bytes)
       state=1;
    }
    // fetch data from queue
    int32_t * data = (int32_t *)queue1.readBuffer();
    //
    // copy to disk buffer
    uint32_t *ptr=(uint32_t *) outptr;
    for(int ii=0;ii<64;ii++) ptr[ii] = data[ii];
    queue1.freeBuffer(); 
    //
    // adjust buffer pointer
    outptr+=128; // (128 shorts)
    //
    // if necessary reset buffer pointer and write to disk
    // buffersize should be always a multiple of 512 bytes
    if(outptr == (diskBuffer+BUFFERSIZE))
    {
      outptr = diskBuffer;
 
      // write to disk ( this handles also opening of files)
      if(state>=0)
        state=uSD.write(diskBuffer,BUFFERSIZE); // this is blocking
    }
  }
  else
  {  // queue is empty
    // do nothing
  }
 
  // some statistics on progress
  static uint32_t loopCount=0;
  static uint32_t t0=0;
  loopCount++;
  if(millis()>t0+1000)
  {  Serial.printf("loop: %5d %4d; %4d",
           loopCount, uSD.getNbuf(),
           AudioMemoryUsageMax());
     Serial.println();
     AudioMemoryUsageMaxReset();
     t0=millis();
     loopCount=0;
  }
  asm("wfi"); // to save some power switch off idle cpu
}

