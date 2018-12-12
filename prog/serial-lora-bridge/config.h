#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED


/**************************************************************************
PINOUT CONFIG
**************************************************************************/
// Module RFM95 pin mapping
#define RFM95_PIN_NSS 6
#define RFM95_PIN_RST 0
#define RFM95_PIN_D0  4
#define RFM95_PIN_D1  5
#define STATUS_PIN_LED 2


/**************************************************************************
CONFIGURATION NODE LORA
**************************************************************************/
#define DATA_RATE_UP_DOWN DR_SF10 // Spreading factor (DR_SF7 - DR_SF12)
#define TX_POWER          14      // power option: 2, 5, 8, 11, 14 and 20
#define SESSION_PORT      0x01    // Port session


#endif // CONFIG_H_INCLUDED
