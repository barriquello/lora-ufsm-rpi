/**************************************************************************
  NODE LORAWAN AU915MHZ WITH RASPBERRY PI 3 + (RFM95-SX1276)
***************************************************************************
  AUTHOR: LUCAS MAZIERO - Electrical Engineer
  EMAIL: lucas.mazie.ro@hotmail.com
  CITY: Santa Maria - Rio Grande do Sul - Brasil
  FOUNDATION: Fox IoT
***************************************************************************
  Version: 1.0
  Date: 27/06/2018
  Modified: 09/10/2018 by Luiz Odilon de Lima Brites
  Added functionality in reading DevAddr, Nwskey and Appskey from file
***************************************************************************
  Copyright(c) by: Fox IoT.
**************************************************************************/
/**************************************************************************
CONFIGURATION NODE LORA
**************************************************************************/
#define DATA_RATE_UP_DOWN DR_SF10 // Spreading factor (DR_SF7 - DR_SF12)
#define TX_POWER          14      // power option: 2, 5, 8, 11, 14 and 20
#define SESSION_PORT      0x01    // Port session

/**************************************************************************
  AUXILIARY LIBRARIES
**************************************************************************/
#include <stdio.h>
#include <time.h>
#include <wiringPi.h>
#include <lmic.h>
#include <hal.h>
#include <local_hal.h>
#include <stdlib.h>
#include <string.h>




/**************************************************************************
  VARIABLES AND DEFINITIONS
**************************************************************************/
// Module RFM95 pin mapping
#define RFM95_PIN_NSS 6
#define RFM95_PIN_RST 0
#define RFM95_PIN_D0  4
#define RFM95_PIN_D1  5
//#define RFM95_PIN_D2 1

#define STATUS_PIN_LED 2


/*************************************************************************
  DECLARATION FOR READING DATA FROM FILE - BY BRITES, L. O. L
/************************************************************************/
FILE *fp;
char c, str1[2], str2[4];
int i = 0, j = 0, k = 0, l = 0;
int DevAddr_a[4], Nwkskey_a[16], Appskey_a[16];
/************************************************************************
  END DECLARATION Added and code continues
/***********************************************************************/
// LoRaWAN end-device address (DevAddr)
u1_t DevAddr[4]; //={0x26, 0x01, 0x17, 0x2E};

// LoRaWAN NwkSKey, network session key
u1_t Nwkskey[16]; //{0x81, 0x58, 0xC3, 0x00, 0xAB, 0x35, 0x5E, 0xD7, 0xF4, 0x1D, 0xC9, 0x0F, 0xDD, 0x4C, 0xB1, 0xB9};

// LoRaWAN AppSKey, application session key
u1_t Appskey[16]; //{0xB7, 0xA2, 0x76, 0xBE, 0xAB, 0x79, 0xDF, 0x9C, 0xA0, 0x7B, 0xB6, 0xB4, 0x4A, 0xF5, 0x67, 0xE8};

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
int TX_INTERVAL = 30;

// Convert u4_t in u1_t(array)
#define msbf4_read(p) (u4_t) ((u4_t) (p)[0] << 24 | (u4_t) (p)[1] << 16 | (p)[2] << 8 | (p)[3])

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain.
void os_getArtEui(u1_t * buf){}

void os_getDevEui(u1_t * buf){}

void os_getDevKey(u1_t * buf){}

u4_t cntr = 0;
u1_t mydata[25];

static osjob_t sendjob;

// Pin mapping
lmic_pinmap pins =
{
  .nss = RFM95_PIN_NSS,
  .rxtx = UNUSED_PIN,   // Not connected on RFM92/RFM95
  .rst = RFM95_PIN_RST, // Needed on RFM92/RFM95
  .dio ={RFM95_PIN_D0, RFM95_PIN_D1, UNUSED_PIN}, // D0, D1, D2(Not used)
};

void onEvent(ev_t ev)
{
  switch (ev)
  {
    // scheduled data sent (optionally data received)
    // note: this includes the receive window!
    case EV_TXCOMPLETE:

      // use this event to keep track of actual transmissions
      fprintf(stdout, "Event EV_TXCOMPLETE, time: %d\n", millis() / 1000);

      // Check ACK
      if (LMIC.txrxFlags & TXRX_ACK) fprintf(stdout, "Received ACK!\n");
      else if (LMIC.txrxFlags & TXRX_NACK)  fprintf(stdout, "No ACK received!\n");

      // Check DOWN
      if (LMIC.dataLen)
      {

	fprintf(stdout, "RSSI: ");
	fprintf(stdout, "%ld", LMIC.rssi - 96);
	fprintf(stdout, " dBm\n");

	fprintf(stdout, "SNR: ");
	fprintf(stdout, "%ld", LMIC.snr * 0.25);
	fprintf(stdout, " dB\n");

	fprintf(stdout, "Data Received!\n");
        for (int i = 0; i < LMIC.dataLen; i++)
        {
          fprintf(stdout, "0x%02x ", LMIC.frame[LMIC.dataBeg + i]);
        }
        fprintf(stdout, "\n");
      }
    break;
    default:
      fprintf(stdout, "Unknown event\n");
    break;
  }
}

static void do_send(osjob_t * j)
{
  time_t t = time(NULL);
  fprintf(stdout, "[%x] (%ld) %s\n", hal_ticks(), t, ctime(& t));
  // Show TX channel (channel numbers are local to LMIC)
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    fprintf(stdout, "OP_TXRXPEND, not sending");
  }
  else
  {
    // Prepare upstream data transmission at the next possible time.
    char buf[25];
    sprintf(buf, "Hello world! [%d]", cntr++);
    int i = 0;
    while (buf[i])
    {
      mydata[i] = buf[i];
      i++;
    }
    mydata[i] = '\0';
    LMIC_setTxData2(1, mydata, strlen(buf), 0);
  }
  // Schedule a timed job to run at the given timestamp (absolute system time)
  os_setTimedCallback(j, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
}

/*
PI_THREAD (blinkRun)
{
  while (1)
  {
    if ((millis() % 3000) < 250) digitalWrite(STATUS_PIN_LED, HIGH);
    else digitalWrite(STATUS_PIN_LED, LOW);
  }
}
*/

void setup()
{
  //wiringPi init
  wiringPiSetup();

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

/**************************************************************************
  NEW FUNCTIONALITY FOR DATA FILE HANDLER
/**************************************************************************/
  fp=fopen("id_data.txt", "r");

  c=getc(fp);
  while(i < 1){
    sprintf(str2,"%s", "0x");
    sprintf(str1, "%c", c);
    strcat(str2, str1);
    c=getc(fp);
    sprintf(str1, "%c", c);
    strcat(str2, str1);
    c=getc(fp);
    if(i == 0 && j < 4){
      DevAddr_a[j] = (int)strtol(str2, NULL, 0);
      j++;
    }
    if(j == 4){
      i++;
      printf("\n=== line %d ok\n", i);
      c=getc(fp);
    }
  }
  for(i=0; i<4; i++){
    DevAddr[i] = (u1_t)DevAddr_a[i];
  }
  for(i=0; i<4; i++){
    printf("devaddr-%d===%x\n", i, DevAddr[i]);
  }
  i=0;
  j=0;
  c=getc(fp);
  while(i < 1){
    sprintf(str2,"%s", "0x");
    sprintf(str1, "%c", c);
    strcat(str2, str1);
    c=getc(fp);
    sprintf(str1, "%c", c);
    strcat(str2, str1);
    c=getc(fp);
    if(i == 0 && j < 16){
      Nwkskey_a[j] = (int)strtol(str2, NULL, 0);
      j++;
    }
    if(j == 16){
      i++;
      printf("\n=== line %d ok 2 \n", i);
      c=getc(fp);
    }
  }
  for(i=0; i<16; i++){
    Nwkskey[i] = (u1_t)Nwkskey_a[i];
  }
  for(i=0; i<16; i++){
    printf("Nwkskey-%d===%x\n", i, Nwkskey[i]);
  }
  i=0;
  j=0;
  c=getc(fp);
  while(i < 1){
    sprintf(str2,"%s", "0x");
    sprintf(str1, "%c", c);
    strcat(str2, str1);
    c=getc(fp);
    sprintf(str1, "%c", c);
    strcat(str2, str1);
    c=getc(fp);
    if(i == 0 && j < 16){
      Appskey_a[j] = (int)strtol(str2, NULL, 0);
      j++;
    }
    if(j == 16){
      i++;
      printf("\n=== line %d ok 3 \n", i);
      c=getc(fp);
    }
  }

  for(i=0; i<16; i++){
    Appskey[i] = (u1_t)Appskey_a[i];
  }

  for(i=0; i<16; i++){
    printf("Appskey-%d===%x\n", i, Appskey[i]);
  }

  for(i=0; i<4; i++){
    printf("devaddr-%d===%x\n", i, DevAddr[i]);
  }
  for(i=0; i<16; i++){
    printf("Nwkskey-%d===%x\n", i, Nwkskey[i]);
  }
  
  for(i=0; i<16; i++){
    printf("Appskey-%d===%x\n", i, Appskey[i]);
  }
  fclose(fp);
  printf("\nCopied from file in read operation!==========By Luiz Odilon\n");
/***************************************************************************
    END HANDLER OPERATIONS AND THE CODE continues
/**************************************************************************/

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  LMIC_setSession(SESSION_PORT, msbf4_read((u1_t *)DevAddr),(u1_t *)Nwkskey,(u1_t *)Appskey);  /*

/*
  // Multi channel AU915 (CH0-CH7)
  // First, disable channels 0-7
  for (int channel = 0; channel < 8; ++channel)
    LMIC_disableChannel(channel);
  // Now, disable channels 16-72 (is there 72 ??)
  for (int channel = 16; channel < 72; ++channel)
    LMIC_disableChannel(channel);
  // This means only channels 8-15 are up
*/

  // Single channel AU915 (CH0)
  //First, disable channels 0-7
  for (int channel = 0; channel < 8; ++channel) LMIC_disableChannel(channel);
  // Now, disable channels 9-72 (is there 72 ??)
  for (int channel = 9; channel < 72; ++channel) LMIC_disableChannel(channel);
  //This means only channel 8 are up (activate channel 0 AU915)

  // Disable data rate adaptation
  LMIC_setAdrMode(0);

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // Disable beacon tracking
  LMIC_disableTracking();

  // Stop listening for downstream data (periodical reception)
  LMIC_stopPingable();

  // TTN RX2 window.
  LMIC.dn2Dr = DATA_RATE_UP_DOWN;

  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DATA_RATE_UP_DOWN, TX_POWER);

  // Set pin direction
  pinMode(STATUS_PIN_LED, OUTPUT);

  //Add thread
  //piThreadCreate (blinkRun);

    // Start job
  do_send(& sendjob);
}

void loop()
{
  os_runloop();
}

int main()
{
  setup();
  while (1)
  {
   loop();
  }
  return 0;
}

// TTN decode payload
/*
function Decoder(bytes, port)
{
  var result = "";
  for (var i = 0; i < bytes.length; i++)
  {
    result += (String.fromCharCode(bytes[i]));
  }

  return {payload: result};
}
*/
