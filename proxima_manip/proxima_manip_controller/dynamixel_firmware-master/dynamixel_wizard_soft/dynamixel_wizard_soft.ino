#include <SoftHalfDuplexSerial.h>


#define DXL_USB_VER           20170915    

#define CMD_PORT              Serial      // USB  
#define DXL_PORT              hdSerial  
#define DXL_BAUD              57600   

#define DXL_TX_BUFFER_LENGTH  1024

uint8_t tx_buffer[DXL_TX_BUFFER_LENGTH];

uint32_t usb_baud;  


softHalfDuplexSerial hdSerial(8);           // using pin 8 see lib for limitation

void setup()                          
{
  CMD_PORT.begin(57600);            
  DXL_PORT.begin(DXL_BAUD);         //obsługa serw

void loop()  
{
  /*
  if(CMD_PORT.getBaudRate() != DXL_PORT.getBaudRate()) //jeśli prędkość z jednego portu nie jest równa prędkości z drugiego portu
  {
    DXL_PORT.begin(CMD_PORT.getBaudRate()); //uruchomienie drugiego portu z prędkością pierwszego portu
  }*/
  
  update_dxl();   
}


void update_dxl()        //odpowiada za komunikację z serwami
{
  int length;
  int i;

  //-- USB -> DXL
  length = CMD_PORT.available();      
  if( length > 0 )    
  {
    for(i=0; i<length; i++ )    
    {
      DXL_PORT.write(CMD_PORT.read());   
      DXL_PORT.flush();              y 
    }
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
