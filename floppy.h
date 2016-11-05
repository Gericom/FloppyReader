#ifndef __FLOPPY_H__
#define __FLOPPY_H__

#include <Arduino.h>

typedef struct
{ 
  uint32_t reversals;
  uint32_t reversals_left;
  byte mfm_byte;
  uint32_t half_nibbles_left;
  byte* pMFMData;
  uint32_t mfmDataLeft;
} floppy_decoder_work;

typedef struct
{
  uint32_t count : 24;
  uint32_t timing : 8;
} timing_entry;

#endif
