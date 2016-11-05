#include "floppy.h"

#define FLOPPY_PIN_INDEX  12
#define FLOPPY_PIN_MOTEA  11
#define FLOPPY_PIN_MOTEB  10
#define FLOPPY_PIN_DIR    9
#define FLOPPY_PIN_STEP   8
#define FLOPPY_PIN_WDATE  7
//hardwired to gnd for safety reasons
//#define FLOPPY_PIN_WGATE
#define FLOPPY_PIN_TRK00  6
#define FLOPPY_PIN_WPT    4
#define FLOPPY_PIN_RDATA  5

#define FLOPPY_DATA_COUNT  (4096)

byte floppyData[512];
byte data[FLOPPY_DATA_COUNT];
timing_entry timings[256];
boolean timing_cap_mode = true;
int timing_1 = 0x69;
int timing_2 = 0x96;
volatile byte* pData = &data[0];
volatile byte* pDataMax = &data[1024];
volatile byte curval = 0;
volatile int offset = 0;
volatile int rolstate = 0;

volatile byte done;

int state = 0;

#define TIME_OFFSET  6 //42 //10 //21 //8 //15 //0

#pragma GCC push_options
#pragma GCC optimize ("O3")
void TC6_Handler()
{
  //reads the interrupt. necessary to clear the interrupt flag.
  const uint32_t status=TC_GetStatus(TC2, 0);
  if(offset >= FLOPPY_DATA_COUNT)
    return;

  if(!(status & TC_SR_LDRAS))
    return;


  int counter = TC2->TC_CHANNEL[0].TC_RA;


  /*if (counter >= 42 + TIME_OFFSET && counter < 84 + TIME_OFFSET)
   {
   curval <<= 2;
   curval |= 1;
   }
   else if (counter >= 84 + TIME_OFFSET && counter < 126 + TIME_OFFSET)
   {
   curval <<= 2;
   curval |= 2;
   }
   else if (counter >= 126 + TIME_OFFSET && counter < 168 + TIME_OFFSET)
   {
   curval <<= 2;
   curval |= 3;
   }
   else
   return;*/
  if(timing_cap_mode)
  {
    if(counter > 255)
      counter = 255;
    data[offset++] = counter;
  }
  else
  {

    if(counter < 42)
      return;
    else if (counter < timing_1)
    {
      curval <<= 2;
      curval |= 1;
    }
    else if (counter < timing_2)
    {
      curval <<= 2;
      curval |= 2;
    }
    else
    {
      curval <<= 2;
      curval |= 3;
    }

    if (rolstate == 3)
    {
      data[offset++] = curval;
      curval = 0;
      rolstate = 0;
    }
    else
      rolstate++;
  }
}

void floppy_decoder_fill_half_nibbles(floppy_decoder_work* pWork)
{
  if(pWork->mfmDataLeft >= 1)
  {
    pWork->mfm_byte = *pWork->pMFMData++;
    pWork->mfmDataLeft--;
    pWork->half_nibbles_left = 4;
  }
}

void floppy_decoder_fill_reversals(floppy_decoder_work* pWork)
{
  while(32 - pWork->reversals_left >= 4)
  {
    if(pWork->half_nibbles_left < 1)
      floppy_decoder_fill_half_nibbles(pWork);
    if(pWork->half_nibbles_left < 1)
      return;
    uint32_t timing = pWork->mfm_byte >> 6;
    pWork->mfm_byte <<= 2;
    pWork->half_nibbles_left--;
    switch (timing)
    {
    case 1:
      pWork->reversals |= 2 << (32 - pWork->reversals_left - 2);
      pWork->reversals_left += 2;
      break;
    case 2:
      pWork->reversals |= 4 << (32 - pWork->reversals_left - 3);
      pWork->reversals_left += 3;
      break;
    case 3:
      pWork->reversals |= 8 << (32 - pWork->reversals_left - 4);
      pWork->reversals_left += 4;
      break;
    }
  }
}

byte floppy_decoder_get_bit(floppy_decoder_work* pWork)
{
  if(pWork->reversals_left < 2)
    floppy_decoder_fill_reversals(pWork);
  if(pWork->reversals_left < 2)
    return 0xFF;
  uint32_t reversal = pWork->reversals >> 30;
  pWork->reversals <<= 2;
  pWork->reversals_left -= 2;
  if (reversal == 1) //1
  {
    return 1;
  }
  else if (reversal == 2 || reversal == 0)
  {
    return 0;
  }
  return 0xFF;
}

void floppy_decoder_init_work(floppy_decoder_work* pWork, byte* mfm, uint32_t mfmLength)
{
  pWork->reversals = 0;
  pWork->reversals_left = 0;
  pWork->mfm_byte = 0;
  pWork->half_nibbles_left = 0;
  pWork->pMFMData = mfm;
  pWork->mfmDataLeft = mfmLength;
}

boolean decodeFloppyData(byte* mfm, uint32_t mfmLength, byte* sectorData, int requested_sector)
{
  floppy_decoder_work work;
  floppy_decoder_init_work(&work, mfm, mfmLength);
  byte sector;
  do
  {
    //find id record
    uint32_t shifter = 0;
    for(int i = 0; i < 32; i++)
    {
      byte bitt = floppy_decoder_get_bit(&work);
      while(bitt == 0xFF && work.mfmDataLeft > 0)
      {
        //Serial.println(bitt, HEX);
        bitt = floppy_decoder_get_bit(&work);
      }
      shifter <<= 1;
      shifter |= bitt;
    }
    if(work.mfmDataLeft <= 0)
      return false;
    while(shifter != 0xA1A1A1FE && work.mfmDataLeft > 0)
    {
      byte bitt = floppy_decoder_get_bit(&work);
      while(bitt == 0xFF && work.mfmDataLeft > 0)
      {
        //Serial.println(bitt, HEX);
        bitt = floppy_decoder_get_bit(&work);
      }
      shifter <<= 1;
      shifter |= bitt;
    }
    if(work.mfmDataLeft <= 0)
      return false;
    byte cylinder = 0;
    for(int j = 0; j < 8; j++)
    {
      byte bitt = floppy_decoder_get_bit(&work);
      while(bitt == 0xFF && work.mfmDataLeft > 0)
      {
        //  Serial.println(bitt, HEX);
        bitt = floppy_decoder_get_bit(&work);
      }
      cylinder <<= 1;
      cylinder |= bitt;
    }
    if(work.mfmDataLeft <= 0)
      return false;
    byte side = 0;
    for(int j = 0; j < 8; j++)
    {
      byte bitt = floppy_decoder_get_bit(&work);
      while(bitt == 0xFF && work.mfmDataLeft > 0)
      {
        //  Serial.println(bitt, HEX);
        bitt = floppy_decoder_get_bit(&work);
      }
      side <<= 1;
      side |= bitt;
    }
    if(work.mfmDataLeft <= 0)
      return false;
    sector = 0;
    for(int j = 0; j < 8; j++)
    {
      byte bitt = floppy_decoder_get_bit(&work);
      while(bitt == 0xFF && work.mfmDataLeft > 0)
      {
        //  Serial.println(bitt, HEX);
        bitt = floppy_decoder_get_bit(&work);
      }
      sector <<= 1;
      sector |= bitt;
    }
    if(work.mfmDataLeft <= 0)
      return false;
    byte sector_info[4];
    sector_info[0] = cylinder;
    sector_info[1] = side;
    sector_info[2] = sector;
    sector_info[3] = 0;
    for(int j = 0; j < 8; j++)
    {
      byte bitt = floppy_decoder_get_bit(&work);
      while(bitt == 0xFF && work.mfmDataLeft > 0)
      {
        //  Serial.println(bitt, HEX);
        bitt = floppy_decoder_get_bit(&work);
      }
      sector_info[3] <<= 1;
      sector_info[3] |= bitt;
    }
    if(work.mfmDataLeft <= 0)
      return false;
    uint16_t val = 0;
    for(int j = 0; j < 16; j++)
    {
      byte bitt = floppy_decoder_get_bit(&work);
      while(bitt == 0xFF && work.mfmDataLeft > 0)
      {
        //Serial.println(bitt, HEX);
        bitt = floppy_decoder_get_bit(&work);
      }
      val <<= 1;
      val |= bitt;
    }
    uint16_t realcrc = crc16(&sector_info[0], 4, 0xFE);
    if(realcrc != val)
      sector = 0xFF;
    //Serial.println(cylinder);
    // Serial.println(side);
    // Serial.println(sector);
  }
  while(sector != requested_sector);
  int bits_skipped = 0;
  uint32_t shifter = 0;
  for(int i = 0; i < 32; i++)
  {
    byte bitt = floppy_decoder_get_bit(&work);
    while(bitt == 0xFF && work.mfmDataLeft > 0)
    {
      //Serial.println(bitt, HEX);
      bitt = floppy_decoder_get_bit(&work);
    }
    shifter <<= 1;
    shifter |= bitt;
  }
  if(work.mfmDataLeft <= 0)
    return false;
  while(shifter != 0xA1A1A1FB && work.mfmDataLeft > 0)
  {
    if(shifter == 0xFFFFFFFF)
    {
      if(work.reversals_left <1 )
        floppy_decoder_fill_reversals(&work);
      work.reversals <<= 1;
      work.reversals_left--;
    }
    byte bitt = floppy_decoder_get_bit(&work);
    while(bitt == 0xFF && work.mfmDataLeft > 0)
    {
      // Serial.println(bitt, HEX);
      bitt = floppy_decoder_get_bit(&work);
    }
    shifter <<= 1;
    shifter |= bitt;
    bits_skipped++;
    //Serial.println(shifter, HEX);
    if(bits_skipped > 0x80 * 8)
      return false;
  }
  //Serial.println(bits_skipped);
  if(work.mfmDataLeft <= 0)
    return false;
  // Serial.println("Reading Sector");
  byte* pData = sectorData;
  //create the sector data
  for(int i = 0; i < 512; i++)
  {
    byte val = 0;
    for(int j = 0; j < 8; j++)
    {
      byte bitt = floppy_decoder_get_bit(&work);
      while(bitt == 0xFF && work.mfmDataLeft > 0)
      {
        // Serial.println(bitt, HEX);
        bitt = floppy_decoder_get_bit(&work);
      }
      val <<= 1;
      val |= bitt;
    }
    *pData++ = val;
  }
  if(work.mfmDataLeft <= 0)
    return false;
  //read crc
  uint16_t val = 0;
  for(int j = 0; j < 16; j++)
  {
    byte bitt = floppy_decoder_get_bit(&work);
    while(bitt == 0xFF && work.mfmDataLeft > 0)
    {
      //Serial.println(bitt, HEX);
      bitt = floppy_decoder_get_bit(&work);
    }
    val <<= 1;
    val |= bitt;
  }
  uint16_t realcrc = crc16(sectorData, 512, 0xFB);
  //Serial.println(val, HEX);
  // Serial.println(realcrc, HEX);
  if(realcrc != val)
    return false;
  return true;
}

int curtrack = 0;

void floppy_step_to_track(int track)
{
  if(track == curtrack)
    return;
  if(track < curtrack)
  {
    while(track != curtrack)
    {
      stepOut();
      curtrack--;
    }
  }
  else
  {
    while(track != curtrack)
    {
      stepIn();
      curtrack++;
    }
  }
  delay(20);
}

void floppy_process_timings(byte* data, uint32_t count)
{
  //frequency table:
  for(int i = 0; i < 256; i++)
  {
    timings[i].count = 0;
    timings[i].timing = i;
  }
  for(int i = 0; i < count; i++)
  {
    int val = *data++;
    if(val < 35)
      continue;
    timings[val].count++;
  }
  for(int i = 0; i < 256; i++)
  {
    if(timings[i].count <= 2)
      timings[i].count = 0;
  }
  //sort based on count
  for(int i=0; i<256; i++)
  {
    for(int j=0; j<(256-i - 1); j++)
    {
      if(timings[j].count < timings[j+1].count)
      {
        timing_entry temp = timings[j];
        timings[j] = timings[j+1];
        timings[j+1] = temp;
      }
    }
  }
  //sort top 20 based on timing
  for(int i=0; i<20; i++)
  {
    for(int j=0; j<(20-i - 1); j++)
    {
      if(timings[j].timing > timings[j+1].timing)
      {
        timing_entry temp = timings[j];
        timings[j] = timings[j+1];
        timings[j+1] = temp;
      }
    }
  }
  int part1start = 0;
  for(int i = 0; i < 20; i++)
  {
    if(timings[i].count != 0)
      break;
    part1start++;
  }
  int part1 = 0;//length of part 1
  for(int i = part1start + 1; i < 20; i++)
  {
    if(timings[i].timing - timings[i-1].timing > 25)
    {
      part1 = i;
      break;
    }
  }
  int part2 = 0;//end of part 2 (i + 1)
  for(int i = part1 + 1; i < 20; i++)
  {
    if(timings[i].timing - timings[i-1].timing > 25)
    {
      part2 = i;
      break;
    }
  }

  int part1_top = 0;
  for(int i = part1start; i < part1; i++)
  {
    part1_top += timings[i].timing;
  }
  part1_top /= part1 - part1start;
  int part2_top = 0;
  for(int i = part1; i < part2; i++)
  {
    part2_top += timings[i].timing;
  }
  part2_top /= part2 - part1;

  int part3_top = 0;
  for(int i = part2; i < 20; i++)
  {
    part3_top += timings[i].timing;
  }
  part3_top /= 20 - part2;

  timing_1 = (part1_top + part2_top) / 2;
  timing_2 = (part2_top + part3_top) / 2;
  // timing_1 = 0x69;
  //timing_2 = 0x96;
  //Serial.print("Timing1: ");
  //Serial.println(timing_1);
  //Serial.print("Timing2: ");
  //Serial.println(timing_2);
  /*if(timing_1 < 80)
   {
   for(int i = 0; i < 20; i++)
   {
   printf("%d: %d\n", timings[i].timing, timings[i].count);
   }
   while(1); 
   }*/
}

void floppy_read_track(int track, byte* dst, int sector)
{
  floppy_step_to_track(track);
  //delay(100);
  //delay(50);

  //first capture the full timings to determine the right timing borders
  curval = 0;
  rolstate = 0;
  timing_cap_mode = true;
  //index pulse
  while(REG_PIOD_PDSR & (1 << 8));
  while(!(REG_PIOD_PDSR & (1 << 8)));
  offset = 0;
  TC_Start(TC2,0);
  while(offset < FLOPPY_DATA_COUNT);
  floppy_process_timings(&data[0], FLOPPY_DATA_COUNT);
  int timing_dev = 0;
  while(1)
  {
    for(int i = 0; i < 2; i++)
    {
      //read the actual data
      curval = 0;
      rolstate = 0;
      timing_cap_mode = false;
      //index pulse
      while(REG_PIOD_PDSR & (1 << 8));
      while(!(REG_PIOD_PDSR & (1 << 8)));
      // delayMicroseconds(REG_TRNG_ODATA & 0x1F);
      delay(10 * (sector - 1));
      offset = 0;
      TC_Start(TC2,0);
      while(offset < FLOPPY_DATA_COUNT);

      if(decodeFloppyData(&data[0], FLOPPY_DATA_COUNT, dst, sector))
        return;
    }
    if(timing_dev <= 0)
    {
      timing_1-=2;
      timing_2-=2;
      timing_dev-=2;
    }
    else
    {
      timing_1+=2;
      timing_2+=2;
      timing_dev+=2;
    }
    if(timing_dev < -42)
    {
      timing_1 -= timing_dev;
      timing_2 -= timing_dev;
      timing_dev = 2;
      timing_1+=2;
      timing_2+=2;
    }
    else if(timing_dev > 42)
    {
      timing_1 -= timing_dev;
      timing_2 -= timing_dev;
      timing_dev = 0;
    }
  }
}

uint16_t crc16(uint8_t* data_p, uint32_t length, uint8_t id){
  uint8_t x;
  uint16_t crc = 0xFFFF;

  for(int i = 0; i < 3; i++)
  {
    x = crc >> 8 ^ 0xA1;
    x ^= x>>4;
    crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);
  }
  x = crc >> 8 ^ id;
  x ^= x>>4;
  crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);

  while (length--){
    x = crc >> 8 ^ *data_p++;
    x ^= x>>4;
    crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);
  }
  return crc;
}
#pragma GCC pop_options

void setup() {
  // put your setup code here, to run once:
  pinMode(FLOPPY_PIN_INDEX, INPUT_PULLUP);
  pinMode(FLOPPY_PIN_MOTEA, OUTPUT);
  pinMode(FLOPPY_PIN_MOTEB, OUTPUT);
  pinMode(FLOPPY_PIN_DIR, OUTPUT);
  pinMode(FLOPPY_PIN_STEP, OUTPUT);
  pinMode(FLOPPY_PIN_WDATE, OUTPUT);
  pinMode(FLOPPY_PIN_TRK00, INPUT_PULLUP);
  pinMode(FLOPPY_PIN_WPT, INPUT_PULLUP);
  pinMode(FLOPPY_PIN_RDATA, INPUT);//INPUT_PULLUP);
  digitalWrite(FLOPPY_PIN_MOTEA, HIGH);
  digitalWrite(FLOPPY_PIN_MOTEB, HIGH);
  digitalWrite(FLOPPY_PIN_DIR, HIGH);
  digitalWrite(FLOPPY_PIN_STEP, HIGH);
  digitalWrite(FLOPPY_PIN_WDATE, HIGH);
  Serial.begin(115200);

  REG_TRNG_CR = TRNG_CR_KEY(0x524e47);
  REG_TRNG_CR |= TRNG_CR_ENABLE;

  REG_TC2_WPMR=0x54494D00;
  REG_PIOC_WPMR=0x50494F00;
  REG_PIOC_PDR |= PIO_PDR_P25;
  REG_PIOC_ABSR |= PIO_ABSR_P25;
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC6);
  TC_Configure(TC2, 0,  TC_CMR_ABETRG | TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_LDRA_RISING/* | TC_CMR_LDRA_RISING*/ | TC_CMR_ETRGEDG_RISING /*| TC_CMR_CPCTRG */);
  // TC_SetRC(TC2, 0, 42);
  const uint32_t flags=  TC_IER_LDRAS; /*|TC_IER_CPCS */
  ;//TC_IER_LDRAS;/*TC_IER_COVFS  |*/ //;
  TC2->TC_CHANNEL[0].TC_IER=flags;
  TC2->TC_CHANNEL[0].TC_IDR=~flags;//assume IER and IDR are equally defined.
  NVIC_EnableIRQ(TC6_IRQn);
  digitalWrite(FLOPPY_PIN_MOTEA, LOW);
  digitalWrite(FLOPPY_PIN_MOTEB, LOW);
  for(int i = 0; i < 10; i++)
    stepIn();
  //move to track 0
  while(digitalRead(FLOPPY_PIN_TRK00) == HIGH)
    stepOut();
  /*floppy_step_to_track(7);
   delay(100);
   offset = 0;
   curval = 0;
   rolstate = 0;
   while(REG_PIOD_PDSR & (1 << 8));//digitalRead(FLOPPY_PIN_INDEX) == HIGH);
   while(!(REG_PIOD_PDSR & (1 << 8)));//digitalRead(FLOPPY_PIN_INDEX) == LOW);
   
   TC_Start(TC2,0);
   while(offset < 4096);*/
  Serial.println("START");
  for(int i = 0; i < 80; i++)
  {
    for(int j = 1; j <= 18; j++)
    {
      floppy_read_track(i, &floppyData[0], j);
      for(int i = 0; i < 512; i++)
      {
        printf("%.2X\n", floppyData[i]);
        //Serial.println(floppyData[i], HEX);//(data[i] >> 3) & 1);
      }
    }
  }
  floppy_step_to_track(10);
  while(1);
}

void stepOut()
{
  digitalWrite(FLOPPY_PIN_DIR, HIGH);
  digitalWrite(FLOPPY_PIN_STEP, LOW);
  delay(1);
  digitalWrite(FLOPPY_PIN_STEP, HIGH);
  delay(5);
}

void stepIn()
{
  digitalWrite(FLOPPY_PIN_DIR, LOW);
  digitalWrite(FLOPPY_PIN_STEP, LOW);
  delay(1);
  digitalWrite(FLOPPY_PIN_STEP, HIGH);
  delay(5);
}

void loop() {
  Serial.print("timer value is ");
  Serial.println( TC_ReadCV(TC2,0));
  delay(4000);
  /*digitalWrite(FLOPPY_PIN_MOTEA, LOW);
   digitalWrite(FLOPPY_PIN_MOTEB, LOW);
   for(int i = 0; i < 20; i++)
   stepIn();
   //move to track 0
   while(digitalRead(FLOPPY_PIN_TRK00) == HIGH)
   stepOut();
   Serial.println("Track0");
   delay(100);
   noInterrupts();
   while(digitalRead(FLOPPY_PIN_INDEX) == HIGH);
   while(digitalRead(FLOPPY_PIN_INDEX) == LOW);
   //*ptr++ = PIND;
   TC_Start(TC2,0);
   while(1);
   // done = 0;
  /*offset = 0;
   curval = 0;
   rolstate = 0;
   TCCR2A = 0;
   TCCR2B = 1;
   TCNT2 = 0;
   TIMSK2 = 0;
   pData = &data[0];
   pDataMax = &data[1024];
   attachInterrupt(3, on_data_raising, RISING);
   interrupts();
   while(offset < 1024);///(uint16_t)pData < (uint16_t)pDataMax);
   // while(!done);
   detachInterrupt(1);
   //readstuff();
  /*do
   {
   if(ptr < ptrmax)
   {
   *ptr++ = (PIND >> 4) & 1;
   }
   }
   while(PINB & (1 <<4));/
   Serial.println("OK");
   for(int i = 0; i < 1024; i++)
   {
   Serial.println(data[i]);//(data[i] >> 3) & 1);
   }
  /*while(1)
   {
   //Serial.println(digitalRead(FLOPPY_PIN_INDEX));
   while(PINB & (1 <<4));//digitalRead(FLOPPY_PIN_INDEX) == LOW);
   while(!(PINB & (1 <<4)));//digitalRead(FLOPPY_PIN_INDEX) == HIGH);
   Serial.println("Index");
   }/
   while(1);*/
}
/*#pragma GCC push_options
 #pragma GCC optimize ("O3")
 void on_data_raising()
 {
 //byte counter = TCNT2;
 //TCNT2 = 0;
 if(offset < 255)
 {
 data[offset++] = TCNT2;
 TCNT2 = 0;
 }
 //offset++;
/*asm volatile("lds r19, 0xB2");
 asm volatile("clr r18");
 asm volatile("sts 0xB2, r18");
 asm volatile("lds r30, lo8(pData)");
 asm volatile("lds r18, lo8(pDataMax)");
 asm volatile("cp r30, r18");
 asm volatile("brge 1f");
 asm volatile("lds r31, hi8(pData)");
 asm volatile("lds r18, hi8(pDataMax)");
 asm volatile("cp r31, r18");
 asm volatile("brge 1f");
 asm volatile("st Z+, r19");
 asm volatile("sts lo8(pData), r30");
 asm volatile("sts hi8(pData), r31");
 asm volatile("1:");*/

//byte counter = TCNT2;
//TCNT2 = 0;
//if(offset >= 1024)
// return;
//Serial.println(counter);
/*curval <<= 2;
 if (counter >= 16 && counter < 32)
 {
 curval |= 1;
 }
 else if (counter >= 32 && counter < 48)
 {
 curval |= 2;
 }
 else if (counter >= 48 && counter < 64)
 {
 curval |= 3;
 }
 if (rolstate == 3)
 {
 data[offset++] = curval;
 curval = 0;
 rolstate = 0;
 }
 else
 rolstate++;/
 //data[offset++] = counter;
 }
 #pragma GCC pop_options*/

/*void readstuff()
 {
 asm volatile("cli");    
 asm volatile("ldi r30, lo8(data)");
 asm volatile("ldi r31, hi8(data)");
 asm volatile("1:");
 asm volatile("sbic 0x3, 4");
 asm volatile("jmp 1b");
 asm volatile("2:");
 asm volatile("sbis 0x3, 4");
 asm volatile("jmp 2b");
 asm volatile(".rept 1024\n\t in R19, 0x09 \n\t st Z+, r19 \n\t .endr");
 asm volatile("sei");
 }*/















