#include <RCSwitch.h>
#include <STM32Sleep.h>
#include <RTClock.h>

// Defined for power and sleep functions pwr.h and scb.h
#include <libmaple/pwr.h>
#include <libmaple/scb.h>

RCSwitch mySwitch = RCSwitch();
RTClock rt(RTCSEL_LSE);

int led = PC13; // LED is connected to this pin on Blue Pill board
int but = PB8; // Button is connected to this pin on Maple board (no button on maple board)
int nosleeppin = PA3; // If this pin is low do not go to sleep
int rfoutpin = PB6; // DATA pin of 433MHz module is connected here

uint32_t address = 0; // Adress of this device, calculated as 16-bit CRC of CPU ID code (which itself is 96 bits)

volatile int ctr = 0;
volatile int oldctr = -1;
volatile int bLED = LOW;

const long int alarmDelay = 10;

/**
 * Interrupt handler for gpio pin "but"
 * 
 * This interrupt handler does not do anything since the interrupt also wakes the system 
 * from sleep thus everything is done in main loop
 */
void interruptFunction()
{
  detachInterrupt(but);
  bLED = 1-bLED;
  ctr++;
  //digitalWrite(led, bLED);
}

void setup() {

  /* get CPU Id (unique Id for each STM32 processor) */
  uint16* idBase0 =  (uint16 *) (0x1FFFF7E8);
  uint16* idBase1 =  (uint16 *) (0x1FFFF7E8+0x02);
  uint32* idBase2 =  (uint32 *) (0x1FFFF7E8+0x04);
  uint32* idBase3 =  (uint32 *) (0x1FFFF7E8+0x08);

  unsigned char const* id = (unsigned char*)0x1FFFF7E8;

  // set RTC to a date (any date is fine, just don't leave it uninitialized)
  rt.setTime(9201600);

   // Disable ADC to save power
  adc_disable_all();

  // Set all GPIO pins to Analog input to save power (this disables pretty 
  // much all I/O incl. Serial)
  setGPIOModeToAllPins(GPIO_INPUT_ANALOG);
    
  pinMode(led, OUTPUT);
  pinMode(but,INPUT_PULLUP);
  pinMode(rfoutpin, OUTPUT);
  pinMode(nosleeppin, INPUT_PULLUP);

  attachInterrupt(but, interruptFunction, CHANGE);

  mySwitch.enableTransmit(rfoutpin);

  Serial.end(); // Stop USB serial so USB interrupt does not keep waking us from sleep
  Serial1.begin(115200);
  Serial1.println("\n\nUnd los gehts auf Port 1...");

  address = crc_update(address, (const void*) id, 12); // we only have 16 bits to use as device address ==> generate address as CRC of unique processor ID
  delay(10);
}

void loop() {
  reset_timer();
  start_timer();
  Serial1.begin(115200);
  
  do
  {
    unsigned char filt = 0;  // debounce filter
    unsigned int i;
    int inp = 0;
    for (i=0; i < 5; i++)
    {
      filt += digitalRead(but);
      delay(2);
    }
    if (filt > 2) inp = 1; // filtered Button state
    oldctr = ctr;
    Serial1.print(ctr);
    Serial1.print(": ");
    Serial1.print(". Taster ist ");
    Serial1.print(inp == HIGH ? "gedrueckt. Zeit: " : "NICHT gedrueckt. Zeit: ");
    digitalWrite(led, bLED);
    mySwitch.send(address<<2 | (inp << 1) | (1-inp), 24); // Der 433mhz Sender versendet die uebergebene Zahl
    Serial1.print(getCycles() / 72000);
    Serial1.println(" ms");
    attachInterrupt(but, interruptFunction, CHANGE);
    delay(10);
  } while (oldctr != ctr);
  
  if (digitalRead(nosleeppin))
  {
    // Set CPU to sleep mode. Will return only on (external) Interrupt and timer interrupt (alarmDelay time)
    //sleepAndWakeUp(STOP, &rt, alarmDelay);
    sleepAndWakeUp(STANDBY, &rt, alarmDelay); // should use less power than STOP mode, but wakes up through reset only.
    // After sleep, set clock source to EXT and start PLL
    rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_9);      // 72MHz  => 48 mA
  }
  else
  {
    // Do not put processor to sleep because Pin nosleeppin is pulled low (indicates development mode)
    Serial1.println("CPU not sleeping due to pin pulled high (Debug mode) (main loop runs every 500ms)");
    delay(500);
  }
}  





volatile unsigned int *DWT_CYCCNT  ;
volatile unsigned int *DWT_CONTROL ;
volatile unsigned int *SCB_DEMCR   ;

void reset_timer(){
    DWT_CYCCNT   = (unsigned int *)0xE0001004; //address of the register
    DWT_CONTROL  = (unsigned int *)0xE0001000; //address of the register
    SCB_DEMCR    = (unsigned int *)0xE000EDFC; //address of the register
    *SCB_DEMCR   = *SCB_DEMCR | 0x01000000;
    *DWT_CYCCNT  = 0; // reset the counter
    *DWT_CONTROL |= 1; 
}

void start_timer(){
    *DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter
}

void stop_timer(){
    *DWT_CONTROL = *DWT_CONTROL | 0 ; // disable the counter    
}

unsigned int getCycles(){
    return *DWT_CYCCNT;
}










// ------------------------------------------------------------------------------
// Empty routine used to set alarm
// ------------------------------------------------------------------------------

static void noop() {}


// ------------------------------------------------------------------------------
// Go to sleep, can be Standby or Stop mode
// ------------------------------------------------------------------------------

void goToSleep(uint8_t mode) {
    // Clear PDDS and LPDS bits
    PWR_BASE->CR &= ~PWR_CR_PDDS;
    PWR_BASE->CR &= ~PWR_CR_LPDS;

    // Clear previous wakeup register
    PWR_BASE->CR |= PWR_CR_CWUF;

    if (mode == 1) {
      PWR_BASE->CR |= PWR_CR_PDDS;
    }

    PWR_BASE->CR |= PWR_CR_LPDS;

    SCB_BASE->SCR |= SCB_SCR_SLEEPDEEP;

    // Now go into stop mode, wake up on interrupt
    asm("    wfi");
}




/**
 * \file
 * Functions and types for CRC checks.
 *
 * Generated on Mon Apr  2 20:31:55 2018
 * by pycrc v0.9.1, https://pycrc.org
 * using the configuration:
 *  - Width         = 16
 *  - Poly          = 0x1021
 *  - XorIn         = 0xffff
 *  - ReflectIn     = False
 *  - XorOut        = 0x0000
 *  - ReflectOut    = False
 *  - Algorithm     = table-driven
 */
#include <stdlib.h>
#include <stdint.h>

/**
 * Static table used for the table_driven implementation.
 */
static const uint16_t crc_table[16] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
};


uint16_t crc_update(uint16_t crc, const void *data, size_t data_len)
{
    const unsigned char *d = (const unsigned char *)data;
    unsigned int tbl_idx;

    while (data_len--) {
        tbl_idx = (crc >> 12) ^ (*d >> 4);
        crc = crc_table[tbl_idx & 0x0f] ^ (crc << 4);
        tbl_idx = (crc >> 12) ^ (*d >> 0);
        crc = crc_table[tbl_idx & 0x0f] ^ (crc << 4);
        d++;
    }
    return crc & 0xffff;
}



