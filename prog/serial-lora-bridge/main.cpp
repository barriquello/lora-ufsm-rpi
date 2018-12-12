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
  These datatypes were modified for store dinamically the data
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
#include <wiringSerial.h>
#include <lmic.h>
#include <hal.h>
#include <local_hal.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "config.h"



/**************************************************************************
  VARIABLES AND DEFINITIONS
**************************************************************************/
#define MAX_LEN  128
u1_t in_buffer[2*MAX_LEN+1];
uint8_t in_idx = 0;
u1_t out_buffer[MAX_LEN+1];
uint8_t out_idx = 0;

/************************************************************************
  END DECLARATION Added and code continues
/***********************************************************************/
// LoRaWAN end-device address (DevAddr)
u1_t DevAddr[4];
// LoRaWAN NwkSKey, network session key
u1_t Nwkskey[16]; 
// LoRaWAN AppSKey, application session key
u1_t Appskey[16];

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
int TX_INTERVAL = 15;

// Convert u4_t in u1_t(array)
#define msbf4_read(p) (u4_t) ((u4_t) (p)[0] << 24 | (u4_t) (p)[1] << 16 | (p)[2] << 8 | (p)[3])

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain.
void os_getArtEui(u1_t * buf){}

void os_getDevEui(u1_t * buf){}

void os_getDevKey(u1_t * buf){}


static osjob_t sendjob;

// Pin mapping
lmic_pinmap pins =
{
  .nss = RFM95_PIN_NSS,
  .rxtx = UNUSED_PIN,   // Not connected on RFM92/RFM95
  .rst = RFM95_PIN_RST, // Needed on RFM92/RFM95
  .dio ={RFM95_PIN_D0, RFM95_PIN_D1, UNUSED_PIN}, // D0, D1, D2(Not used)
};

int fds = -1;
char serial_dev[16];

	
void serial_rw(void)
{
	
  int i,j;
 
	/* se não conectado, tenta conectar e envia estado da conexão */
  if(fds < 0)
  {
	  for (i=0;i<4;i++)
	  {
		  serial_dev[11] = '0' + i;
		  fprintf(stdout, "tentanto abrir porta %s\n", serial_dev);
		  if ((fds = serialOpen (serial_dev, 115200)) >= 0)
		  {
			  printf("serial %d aberta \n", i);
			  out_idx = 0;
			  break;
		  }
	  }
	  if(i==4)
	  {
		  out_buffer[0] = '?';	  
		  out_idx = 1;
	  }	  
      
	  
  }
  
  /* se conectado, descarrega o buffer de entrada e recarrega o buffer de saída com dados, se houver, ou o estado da conexão */
  if(fds >= 0 && out_idx == 0)
  {
	   if(in_idx > 0)
	   {
		   fprintf(stdout, "RX_BUFFER:");
		   for(j=0;j<in_idx; j++)
		   {
			   serialPutchar (fds, in_buffer[j]);
			   fprintf(stdout,"%c", in_buffer[j]);
		   }
		   fprintf(stdout,"\n\r");
		   in_idx = 0;		   
	   }
	   
	   while (serialDataAvail (fds))
	   {
		   out_buffer[out_idx++]  = serialGetchar (fds);
		   if(out_idx == MAX_LEN)
		   {
			   break;
		   }   		   
	   }
	   
	   if(out_idx == 0)
	   {
		   out_buffer[0] = '!';	
		   out_buffer[1] = serial_dev[11];		   
		   out_idx = 2;
	   }	   
  }
}

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
      if (LMIC.txrxFlags & TXRX_ACK) 
	  {
		  fprintf(stdout, "Received ACK!\n");
		  out_idx = 0;
	  }
      else if (LMIC.txrxFlags & TXRX_NACK)  fprintf(stdout, "No ACK received!\n");

      // Check DOWN
      if (LMIC.dataLen && LMIC.dataLen <= 2*MAX_LEN)
      {

		fprintf(stdout, "RSSI: ");
		fprintf(stdout, "%ld", LMIC.rssi - 96);
		fprintf(stdout, " dBm\n");

		fprintf(stdout, "SNR: ");
		fprintf(stdout, "%ld", LMIC.snr * 0.25);
		fprintf(stdout, " dB\n");

		fprintf(stdout, "Data Received!\n");	
		
		in_idx = 0;
        for (int i = 0; i < LMIC.dataLen; i++)
        {
          fprintf(stdout, "0x%02x ", LMIC.frame[LMIC.dataBeg + i]);

		  in_buffer[in_idx++] = LMIC.frame[LMIC.dataBeg + i];
			  
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
  
  serial_rw();
  
  // Show TX channel (channel numbers are local to LMIC)
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    fprintf(stdout, "OP_TXRXPEND, not sending\n");
  }
  else
  {
    // Prepare upstream data transmission at the next possible time.
	#define CONFIRMED 1
	if(out_idx > 0)
	{
		out_buffer[out_idx] = '\0';
		fprintf(stdout, "TX_BUFFER: %s\n", out_buffer);
		LMIC_setTxData2(SESSION_PORT, out_buffer, out_idx, CONFIRMED);		
	}    
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
uint32_t reverse_bytes(uint32_t bytes)
{
    uint32_t aux = 0;
    uint8_t byte;
    int i;

    for(i = 0; i < 32; i+=8)
    {
        byte = (bytes >> i) & 0xff;
        aux |= byte << (32 - 8 - i);
    }
    return aux;
}

static int read_config(void)
{
	FILE * fp;
    char * line = NULL;
    size_t len = 0;
    ssize_t read;
	int i,j, ret;
	u4_t long_number = 0;
	char temp_string[9];
	
	fp=fopen("id_data.txt", "r");
	if (fp == NULL) {
		printf("config file not found!\r\n");
		return -1;
	}
	
	if(ret = getline(&line, &len, fp) == -1) goto exit;
	
	printf("%s", line);
	long_number = reverse_bytes(strtoul(line, NULL, 16));
	memcpy(DevAddr,&long_number,4);
	
	if(ret = getline(&line, &len, fp) == -1) goto exit;	
	printf("%s", line);	

	for(j=0;j<4;j++)
	{
		strncpy(temp_string, line + 8*j, 8); temp_string[8]  = '\0';
		long_number = reverse_bytes(strtoul(temp_string, NULL, 16));		
		memcpy(Nwkskey+4*j,&long_number,4);		
	}	
	
	if(ret = getline(&line, &len, fp) == -1) goto exit;	
	printf("%s", line);	
	
	for(j=0;j<4;j++)
	{
		strncpy(temp_string, line + 8*j, 8); temp_string[8]  = '\0';
		long_number = reverse_bytes(strtoul(temp_string, NULL, 16));		
		memcpy(Appskey+4*j,&long_number,4);		
	}	
	
	ret = 0;
				
	exit:
	fclose(fp);
	if (line)
        free(line);
	
	return ret;
    
} 

void setup()
{
  int i;
  
  strcpy(serial_dev,"/dev/ttyUSBx");
	
  //wiringPi init
  wiringPiSetup();

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

/**************************************************************************
  NEW FUNCTIONALITY FOR DATA FILE HANDLER
/**************************************************************************/
  if(read_config() != 0)  printf("config error!\r\n");

  for(i=0; i<4; i++){
    printf("%02x", DevAddr[i]);	
  }
  printf("\n");
  
  for(i=0; i<16; i++){
    printf("%02x", Nwkskey[i]);	
  }
  printf("\n");
  
  for(i=0; i<16; i++){
    printf("%02x", Appskey[i]);	
  }
  printf("\n");

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
