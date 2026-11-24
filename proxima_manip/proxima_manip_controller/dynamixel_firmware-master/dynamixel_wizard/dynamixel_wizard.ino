#include <stdarg.h>     


#define DXL_USB_VER           20170915   
#define CMD_PORT              Serial    
#define DXL_PORT              Serial1 
//#define DXL_BAUD              57600
#define DXL_BAUD              1000000   

#define DXL_TX_BUFFER_LENGTH  1024    //bufor nadawczy (TX) 

#define TX_PIN 6    //pin kontroli kierunku

uint8_t tx_buffer[DXL_TX_BUFFER_LENGTH];

static int rx_led_count = 0;
static int tx_led_count = 0;

static int rx_led_update_time;
static int tx_led_update_time;
static uint32_t update_time[8];


static uint32_t rx_data_cnt = 0;
static uint32_t tx_data_cnt = 0;

static uint32_t rx_bandwidth = 0;   
static uint32_t tx_bandwidth = 0;   
uint32_t usb_baud;

void setup() 
{
  CMD_PORT.begin(57600);  
  DXL_PORT.begin(DXL_BAUD);
  pinMode(TX_PIN, OUTPUT); 
    digitalWrite(TX_PIN, LOW); //tryb odbierania
}

void loop() 
{
  update_dxl(); 
}


void update_dxl() //osbługa komunikacji z serwami
{
  int length;
  int i;


  //-- USB -> DXL
  length = CMD_PORT.available();
  if( length > 0 ) 
  {
    digitalWrite(TX_PIN, HIGH); //tryb nadawania
    for(i=0; i<length; i++ ) 
    {
      DXL_PORT.write(CMD_PORT.read()); 
      DXL_PORT.flush();
    }
    digitalWrite(TX_PIN, LOW); //tryb odbierania
  }

  //-- DXL -> USB
  length = DXL_PORT.available();
  if( length > 0 )
  {
    if( length > DXL_TX_BUFFER_LENGTH )
    {
      length = DXL_TX_BUFFER_LENGTH;
    }
    for(i=0; i<length; i++ ) 
    {
      tx_buffer[i] = DXL_PORT.read();
    }
    CMD_PORT.write(tx_buffer, length);
  }
}
