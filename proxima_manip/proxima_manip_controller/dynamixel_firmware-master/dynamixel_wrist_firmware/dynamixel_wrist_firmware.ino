#include <Dynamixel2Arduino.h>  
#include <mcp2515.h>    

#define DXL_PORT              Serial1     
#define DXL_BAUD              1000000   
#define DXL_PIN               6         

#define MSG_MOTOR_COMMAND     0x01  //nowe pozycje dla silników/osi
#define MSG_ACTION_REQUEST    0x02  //polecenie akcji systemowej
#define MSG_STATUS_REPORT     0x03  //aktualny stan, gdy sterownik nadaje do komputera/Ros'a
#define MSG_AXIS_REPORT       0x04  //aktualne pozycje osi
#define MSG_MOTOR_REPORT      0x05  //temperatura i napięcia silników

#define CTRLSIGNAL_POSITION   0x02  //docelowe pozycje dla 0,1,2

#define ACTION_ARM            0x00  //uzbrojenie/zasilanie silników 
#define ACTION_DISARM         0x01  //rozbrojenie/odłączenie zasilania 
#define ACTION_SOFTWARE_RESET 0x05  //resetowanie sterownika


const int SPI_CS_PIN = 10; //(Chip Select) dla interfejsu SPI
const int CAN_INT_PIN = 2;  //pin INT (Interrupt), stan niski
      
const uint8_t canId = 0x0;  //ID CAN 
const uint8_t gripperCanId = 0x2c; //dedykowane ID CAN dla chwytaka 
struct can_frame canMsg;

Dynamixel2Arduino dxl(DXL_PORT, DXL_PIN); 
MCP2515 CAN(SPI_CS_PIN);    


bool setProfileVelocity(uint8_t id, float value)    
{
    return dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, value);
}



bool setProfileAcceleration(uint8_t id, float value)    
{
    return dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, value);
}


uint16_t getPresentTemperature(uint8_t id) {        
    return dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE, id);    
}



typedef union { 
    uint32_t u32;   
    float f32;      
} float_int_union; 


float Uint32AsFloat(uint32_t u32) {  
    float_int_union u; 
    u.u32 = u32;    
    return u.f32;    
}   


uint32_t FloatAsUint32(float f32) { 
    float_int_union u;  
    u.f32 = f32;   
    return u.u32;   
}   



bool armed = false;   //ignoruje polecenia ruchu przesyłane z magistrali CAN lub USB
float axis[4] = {0.0, 0.0, 0.0, 0.0};  //serwy (ID: 0, 1, 2, 3)

float axisPosition[3] = {0.0, 0.0, 0.0}; 
float axisVelocity[3] = {0.0, 0.0, 0.0};
//dla ID: 0, 1, 2

void initCan() { //inicjalizacja kontrolera MCP2515, ustawienie prędkości komunikacji, sprzętowe filtrowanie ramek CAN
    while (!(CAN.reset() == MCP2515::ERROR_OK   
          && CAN.setBitrate(CAN_1000KBPS, MCP_16MHZ) == MCP2515::ERROR_OK  
          && CAN.setNormalMode() == MCP2515::ERROR_OK   
        )) {
        delay(100);

    //MCP2515 - maski i filtry
    CAN.setConfigMode();    
    CAN.setFilterMask(MCP2515::MASK0, 0, 0b11110000000); //Bufor 0 (RXB0): MASK0 i dwa filtry (RXF0, RXF1)
    CAN.setFilterMask(MCP2515::MASK1, 0, 0b11111100000); //Bufor 1 (RXB1): MASK1 i cztery filtry (RXF2, RXF3, RXF4, RXF5)
    CAN.setFilter(MCP2515::RXF0, 0, canId << 5); 
    CAN.setFilter(MCP2515::RXF1, 0, 0); 
    CAN.setFilter(MCP2515::RXF2, 0, gripperCanId << 5); 
    CAN.setFilter(MCP2515::RXF3, 0, 1);
    CAN.setFilter(MCP2515::RXF4, 0, 1);
    CAN.setFilter(MCP2515::RXF5, 0, 1);
    CAN.setNormalMode();   
}


void waitForConnection() {  //odczekanie na gotowość serwów
    dxl.torqueOff(0);
    dxl.torqueOff(1);
    dxl.torqueOff(2);
    dxl.torqueOff(3);
    //wyłącza zasilanie silników na serwach o ID 0, 1, 2, 3

    //Serial.println("Waiting for motors");
    while(!dxl.ping(0) || !dxl.ping(1) || !dxl.ping(2) || !dxl.ping(3));
    initMotors();   
    updateMotors();    
}



void initMotors() { //ustawia parametry pracy serw
    //Serial.println("Initializing motors");
    
    dxl.reboot(0);  
    dxl.reboot(1);
    dxl.reboot(2);
    dxl.reboot(3);

    delay(200);    
    dxl.torqueOn(0);   
    dxl.setGoalPosition(0, 2048.0); 
    dxl.torqueOn(1);    
    dxl.setGoalPosition(1, 0.0);    
  
    float velocity; 
    do {
        velocity = dxl.getPresentVelocity(0);       
    } while(velocity > 0.01 || velocity < -0.01);   
    do {
        velocity = dxl.getPresentVelocity(1);       
    } while(velocity > 0.01 || velocity < -0.01); 
  
    dxl.torqueOff(2);   
    dxl.setOperatingMode(2, ControlTableItem::OP_CURRENT);      //tryb sterowania prądem
    dxl.torqueOn(2); 
    dxl.setGoalCurrent(2, 200);    
  
    delay(100); 
    do {
        velocity = dxl.getPresentVelocity(2);       
    } while(velocity > 0.1 || velocity < -0.1);    
  
    dxl.torqueOff(2);   
    dxl.setOperatingMode(2, ControlTableItem::OP_EXTENDED_POSITION); //tryb wieloobrotowy
    dxl.reboot(2);  
    delay(200); 
    dxl.ping(2);    
    dxl.torqueOn(2);    

    dxl.torqueOff(3);   
    dxl.setOperatingMode(3, ControlTableItem::OP_CURRENT); //tryb sterowania prądem
    dxl.torqueOn(3);    
    dxl.setGoalCurrent(3, -200.0);     
    delay(100); 
    do {
        velocity = dxl.getPresentVelocity(3);    
    } while(velocity > 0.2 || velocity < -0.2); 
    dxl.setGoalCurrent(3, 200.0);   
    delay(100);
    dxl.setGoalCurrent(3, 0);       

    setProfileVelocity(0, 190); 
    setProfileVelocity(1, 190); 
    setProfileVelocity(2, 190); 
    setProfileAcceleration(0, 80); 
    setProfileAcceleration(1, 80);  
    setProfileAcceleration(2, 80);  

    //Serial.println("Motors ready!");
}


bool updateMotors(){    //ustawia pozycje początkowe, włącza zasilanie silników
    if (!armed) {  
        bool status = true;
        status &= dxl.torqueOff(0); 
        status &= dxl.torqueOff(1); 
        status &= dxl.torqueOff(2); 
        status &= dxl.torqueOff(3); 
        return status;  
    } else {
        dxl.torqueOn(0);   
        dxl.torqueOn(1);   
        dxl.torqueOn(2);    
        dxl.torqueOn(3);    
    }
    float wrist = -axis[0]; 
    float pitch = -axis[1];
    float roll = axis[2] + 0.25; 
    float motor0 = (wrist + 0.5) * 4096.0; 
    float motor1 = (roll + pitch) * 4096.0; 
    float motor2 = (roll - pitch) * 4096.0;
    float motor3 = axis[3] * 910.0; 

    bool status = true;
    status &= dxl.setGoalPosition(0, motor0);   //polecenie ustawienia zadanej pozycji, zwrot wartości
    status &= dxl.setGoalPosition(1, motor1);   
    status &= dxl.setGoalPosition(2, motor2);   
    status &= dxl.setGoalCurrent(3, motor3);  

    return status; 
}



void handleCan() {      //komunikacja przez magistralę CAN
    while (CAN.readMessage(&canMsg) == MCP2515::ERROR_OK) { 
       
        if (((canMsg.can_id >> 5) & ~0b11) == canId) {
           
            uint8_t axisId = (canMsg.can_id >> 5) & 0b11; 
            uint8_t messageType = canMsg.can_id & 0b11111; 
            uint64_t data = 0;  
            for (int i = 0; i < canMsg.can_dlc; i++) {
                data = data << 8; 
                data |= canMsg.data[i];  
            } 
            if (messageType == MSG_MOTOR_COMMAND) { 
                uint8_t controlSignalId = (data >> 56) & 0xff; 
                float value = Uint32AsFloat((data >> 24) & 0xffffffff);
                if (controlSignalId == CTRLSIGNAL_POSITION) { 
                    axis[axisId] = value; 
                }
            }
            if (messageType == MSG_ACTION_REQUEST) {
                switch (data) { 
                case ACTION_ARM: 
                    armed = true;  
                    break;
                case ACTION_DISARM: 
                    armed = false;  
                    break;
                case ACTION_SOFTWARE_RESET: 
                    waitForConnection();    
                }
            }
        } 
        if (((canMsg.can_id >> 5)) == gripperCanId) { 
            uint64_t data = 0;  
            for (int i = 0; i < canMsg.can_dlc; i++) { 
                data = data << 8; 
                data |= canMsg.data[i]; 
            }
            axis[3] = Uint32AsFloat(data & 0xffffffff); 
        }
    }
}



void transmitCanFrame(uint16_t frameId, uint64_t data) { //do transportu z komputera do can 
    canMsg.can_id  = frameId; 
    canMsg.can_dlc = 8; 
    canMsg.data[0] = (data >> 56) & 0xFF;
    canMsg.data[1] = (data >> 48) & 0xFF;
    canMsg.data[2] = (data >> 40) & 0xFF;
    canMsg.data[3] = (data >> 32) & 0xFF;
    canMsg.data[4] = (data >> 24) & 0xFF;
    canMsg.data[5] = (data >> 16) & 0xFF;
    canMsg.data[6] = (data >> 8) & 0xFF;
    canMsg.data[7] = data & 0xFF; 
    CAN.sendMessage(&canMsg); //wysłanie wiadomości do canu
}


void transmitStatus() { //pobiera pozycje każdego z osi ramienia
    float rawPos[3] = { 
        dxl.getPresentPosition(0), 
        dxl.getPresentPosition(1), 
        dxl.getPresentPosition(2)
    };
  
    axisPosition[0] = (rawPos[0] / 4096.0) - 0.5;
    axisPosition[1] = (rawPos[1] - rawPos[2]) / 8192.0;        //znoszą się, znormalizowanie
    axisPosition[2] = (rawPos[1] + rawPos[2]) / 8192.0;        //napędzają się, znormalizowanie

    /*
    axisVelocity[0] = dxl.getPresentVelocity(0, UNIT_RPM) / 60.0;
    axisVelocity[1] = (dxl.getPresentVelocity(1, UNIT_RPM) - dxl.getPresentVelocity(2, UNIT_RPM)) / 120.0;
    axisVelocity[2] = (dxl.getPresentVelocity(1, UNIT_RPM) + dxl.getPresentVelocity(2, UNIT_RPM)) / 120.0;
    axisVelocity[0] *= -1;
    axisVelocity[1] *= -1;
    */ //idk czemu to wyłączyli

    axisPosition[0] *= -1; 
    axisPosition[1] *= -1;
    
    axisPosition[2] -= 0.25;
    for (uint8_t id = 0; id < 3; id++) { 
        uint16_t frameId; 
        uint64_t data;
        
        frameId = ((canId | id) << 5) | MSG_STATUS_REPORT; //temperatura, stan uzbrojenia
        data = ((uint64_t)armed << 62) | 
                       ((uint64_t)(getPresentTemperature(id) * 10) << 24) | 0; //TODO: add flags
        transmitCanFrame(frameId, data); 
    
        frameId = ((canId | id) << 5) | MSG_AXIS_REPORT;        //pozycja i prędkość osi w przestrzeni
        uint32_t position = FloatAsUint32(axisPosition[id]); 
        uint32_t velocity = FloatAsUint32(axisVelocity[id]); 
        data = (uint64_t)position << 32 | velocity; 
        transmitCanFrame(frameId, data);
    
        frameId = ((canId | id) << 5) | MSG_MOTOR_REPORT;       //pobór prądu i Pulse Width Modulation 
        uint32_t current = FloatAsUint32(dxl.getPresentCurrent(id, UNIT_MILLI_AMPERE) / 1000.0);
        uint32_t duty = FloatAsUint32(dxl.getPresentPWM(id, UNIT_PERCENT) / 100.0);
        data = (uint64_t)current << 32 | duty;
        transmitCanFrame(frameId, data);
    }
}


void setup()        
{
    Serial.begin(1000000); 
    dxl.begin(DXL_BAUD);   
    dxl.setPortProtocolVersion(2.0);        
  
    delay(3000);          
  
    initCan();
    waitForConnection();
}


int a = 0;
void loop() 
{
    a = millis(); //czas w jakim program się wykonał w milisekundach
    handleCan();
    transmitStatus();

    if (!updateMotors()) {
        waitForConnection();
    }
}
