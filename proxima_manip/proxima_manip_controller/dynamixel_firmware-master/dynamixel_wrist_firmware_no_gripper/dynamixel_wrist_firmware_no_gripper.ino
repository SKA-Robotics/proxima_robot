#include <Dynamixel2Arduino.h> 

const int SPI_CS_PIN = 10;
const int CAN_INT_PIN = 2;

#include <mcp2515.h>
MCP2515 CAN(SPI_CS_PIN);
struct can_frame canMsg;
const uint8_t canId = 0x0;
//bez gripper can id

#define DXL_PORT              Serial1
#define DXL_BAUD              1000000
#define DXL_PIN               6

Dynamixel2Arduino dxl(DXL_PORT, DXL_PIN);
using namespace ControlTableItem;

bool setProfileVelocity(uint8_t id, float value)
{
    return dxl.writeControlTableItem(PROFILE_VELOCITY, id, value);
}

bool setProfileAcceleration(uint8_t id, float value)
{
    return dxl.writeControlTableItem(PROFILE_ACCELERATION, id, value);
}

uint16_t getPresentTemperature(uint8_t id) {
    return dxl.readControlTableItem(PRESENT_TEMPERATURE, id);
}

void setup()
{
    Serial.begin(1000000);  
    delay(3000);
    //Serial.println("aaa"); //test
    dxl.begin(DXL_BAUD);
    //Serial.println("bbb"); //test
    dxl.setPortProtocolVersion(2.0);
    //Serial.println("ccc"); //test
  
    delay(3000);
    //Serial.println("Started"); //test/info
  
    initCan();
    waitForConnection();
}

void waitForConnection() { //czeka na gotowość serw
    dxl.torqueOff(0);
    dxl.torqueOff(1);
    dxl.torqueOff(2);
    //Serial.println("Waiting for motors"); //info
    while(!dxl.ping(0) || !dxl.ping(1) || !dxl.ping(2)); //bez serwa3 bo no_gripper

    initMotors();
    updateMotors();
}

void initMotors() { //parametry pracy
   // Serial.println("Initializing motors"); //info
    
    dxl.reboot(0);
    dxl.reboot(1);
    dxl.reboot(2);
    delay(200);

    setProfileVelocity(0, 80);
    setProfileVelocity(1, 190);
    setProfileVelocity(2, 190);
    setProfileAcceleration(0, 40);
    setProfileAcceleration(1, 40);
    setProfileAcceleration(2, 40);

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
    dxl.setOperatingMode(2, OP_CURRENT);
    dxl.torqueOn(2);
    dxl.setGoalCurrent(2, 200);
  
    delay(100);
  
    do {
        velocity = dxl.getPresentVelocity(2);
    } while(velocity > 0.01 || velocity < -0.01);
  
    dxl.torqueOff(2);
    dxl.setOperatingMode(2, OP_EXTENDED_POSITION); //tryb wieloobrotowy
    dxl.reboot(2);
    delay(200);
    dxl.ping(2);
    dxl.torqueOn(2);
  
    Serial.println("Motors ready!");
}

bool armed = false;
float axis[3] = {0.0, 0.0, 0.0};

bool updateMotors(){ //sprawdza łączność serw
    if (!armed) {
        bool status = true;
        status &= dxl.torqueOff(0);
        status &= dxl.torqueOff(1);
        status &= dxl.torqueOff(2);
        return status;
    } else {
        dxl.torqueOn(0);
        dxl.torqueOn(1);
        dxl.torqueOn(2);
    }
    float wrist = -axis[0];
    float pitch = -axis[1];
    float roll = axis[2]; //bez +0.25
    float motor0 = (wrist + 0.5) * 4096.0;
    float motor1 = (roll + pitch) * 4096.0;
    float motor2 = (roll - pitch) * 4096.0;

    bool status = true;
    status &= dxl.setGoalPosition(0, motor0);
    status &= dxl.setGoalPosition(1, motor1);
    status &= dxl.setGoalPosition(2, motor2);

    return status;
}

void initCan() { //inicjalizacja mcp2515, filtrowanie wiadomości
    Serial.println("Initializing can");
    while (!(CAN.reset() == MCP2515::ERROR_OK
          && CAN.setBitrate(CAN_1000KBPS, MCP_16MHZ) == MCP2515::ERROR_OK
          && CAN.setNormalMode() == MCP2515::ERROR_OK
        )) {
        delay(100);
    }
    
    CAN.setConfigMode(); //tryb konfiguracyjny
    CAN.setFilterMask(MCP2515::MASK0, 0, 0b11110000000); //maski
    CAN.setFilterMask(MCP2515::MASK1, 0, 0x7FF);
    CAN.setFilter(MCP2515::RXF0, 0, 0); //filtry
    CAN.setFilter(MCP2515::RXF1, 0, 0);
    CAN.setFilter(MCP2515::RXF2, 0, 1);
    CAN.setFilter(MCP2515::RXF3, 0, 1);
    CAN.setFilter(MCP2515::RXF4, 0, 1);
    CAN.setFilter(MCP2515::RXF5, 0, 1);
    CAN.setNormalMode(); //normalny tryb pracy
    //Serial.println("Can ready!"); //test
}

#define MSG_MOTOR_COMMAND     0x01 //nowe pozycje dla silników
#define MSG_ACTION_REQUEST    0x02 //rozkaz akcji
#define MSG_STATUS_REPORT     0x03 //aktualny stan
#define MSG_AXIS_REPORT       0x04 //aktualne pozycje osi
#define MSG_MOTOR_REPORT      0x05 //temperatura i napięcia

#define CTRLSIGNAL_POSITION 0x02 //docelowe pozycje/kąty obrotu

#define ACTION_ARM 0x00 //uzbrój (torqueOn)
#define ACTION_DISARM 0x01 //rozbrój (torqueOff)
#define ACTION_SOFTWARE_RESET 0x05 //reset serw

typedef union { //połączenie ze sobą zmiennych
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

void handleCan() { //komunikacja przez can
    while (CAN.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        if (((canMsg.can_id >> 5) & ~0b11) == canId) {
            uint8_t axisId = (canMsg.can_id >> 5) & 0b11;
            
            uint8_t messageType = canMsg.can_id & 0b11111;
            uint64_t data = 0;
            for (int i = 0; i < canMsg.can_dlc; i++) {
                data = data << 8;
                data |= canMsg.data[i];
            }
            //Serial.println(axisId);
            //Serial.println(messageType);
            if (messageType == MSG_MOTOR_COMMAND) { //nowe pozycje dla serw
                uint8_t controlSignalId = (data >> 56) & 0xff;
                float value = Uint32AsFloat((data >> 24) & 0xffffffff);
                if (controlSignalId == CTRLSIGNAL_POSITION) { //docelowe pozycje
                    axis[axisId] = value;
                //Serial.println(controlSignalId);
                //Serial.println(value);
                }
            }
            if (messageType == MSG_ACTION_REQUEST) { //rozkas akcji
                switch (data) {
                case ACTION_ARM:
                    armed = true;
                    break;
                case ACTION_DISARM:
                    armed = false;
                    break;
                case ACTION_SOFTWARE_RESET:
                    waitForConnection();
                    break;
                }
            }
        }
        
    }
}

float axisPosition[3] = {0.0, 0.0, 0.0};
float axisVelocity[3] = {0.0, 0.0, 0.0};

void transmitStatus() { //przygotowanie i wysyłanie
    float rawPos[3] = {
        dxl.getPresentPosition(0),
        dxl.getPresentPosition(1),
        dxl.getPresentPosition(2)
    };
  
    axisPosition[0] = (rawPos[0] / 4096.0) - 0.5;
    axisPosition[1] = (rawPos[1] - rawPos[2]) / 8192.0;
    axisPosition[2] = (rawPos[1] + rawPos[2]) / 8192.0;

    /*
    axisVelocity[0] = dxl.getPresentVelocity(0, UNIT_RPM) / 60.0;
    axisVelocity[1] = (dxl.getPresentVelocity(1, UNIT_RPM) - dxl.getPresentVelocity(2, UNIT_RPM)) / 120.0;
    axisVelocity[2] = (dxl.getPresentVelocity(1, UNIT_RPM) + dxl.getPresentVelocity(2, UNIT_RPM)) / 120.0;
    axisVelocity[0] *= -1;
    axisVelocity[1] *= -1;
    */
    axisPosition[0] *= -1;
    axisPosition[1] *= -1;

    axisPosition[2] -= 0.25;
    for (uint8_t id = 0; id < 3; id++) {
        uint16_t frameId;
        uint64_t data;
        
        frameId = ((canId | id) << 5) | MSG_STATUS_REPORT;
        data = ((uint64_t)armed << 62) |
               ((uint64_t)(getPresentTemperature(id) * 10) << 24) | 0; //TODO: add flags
        transmitCanFrame(frameId, data);
    
        frameId = ((canId | id) << 5) | MSG_AXIS_REPORT;
        uint32_t position = FloatAsUint32(axisPosition[id]);
        uint32_t velocity = FloatAsUint32(axisVelocity[id]);
        data = (uint64_t)position << 32 | velocity;
        transmitCanFrame(frameId, data);
    
        frameId = ((canId | id) << 5) | MSG_MOTOR_REPORT;
        uint32_t current = FloatAsUint32(dxl.getPresentCurrent(id, UNIT_MILLI_AMPERE) / 1000.0);
        uint32_t duty = FloatAsUint32(dxl.getPresentPWM(id, UNIT_PERCENT) / 100.0);
        data = (uint64_t)current << 32 | duty;
        transmitCanFrame(frameId, data);
    }
}

void transmitCanFrame(uint16_t frameId, uint64_t data) { //pakowanie i wysyłanie
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
    CAN.sendMessage(&canMsg);
}

int a = 0;
int errorCount = 0;
void loop()
{
    a = millis();
    handleCan();
    transmitStatus();

    if (!updateMotors()) {
        errorCount += 1;
        Serial.print("error count = ");
        Serial.println(errorCount);
    } else {
        errorCount = 0;
    }
    if (errorCount > 10) {
        waitForConnection();
    }
}
