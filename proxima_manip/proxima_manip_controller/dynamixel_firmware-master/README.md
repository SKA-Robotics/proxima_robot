# Firmware nadgarstka i chwytaka w manipulatorze łazika

Firmware odpowiedzialny za sterowanie serwomechanizmami nadgarstka i chwytaka w manipulatorze łazika Sirius2 i Proxima. Pośredniczy między magistralą CAN, a magistralą serwomechanizmów.

# Content

## Piny
| Pin Arduino | Funkcja | Opis |
| :--- | :--- | :--- |
|`Serial1`|Magistrala Dynamixel|Sprzętowa komunikacja UART z serwami|
|`D6`|`DXL_PIN`|Sterowanie kierunkiem nadawania/odbioru|
|`D8`|`SoftHalfDuplexSerial`|Programowa linia danych (w "dynamixel_wizard_soft.ino")|
|`D10`|`SPI_CS_PIN`|Chip Select dla kontrolera CAN mcp2515|
|`D2`|`CAN_INT_PIN`|Pin przerwania z kontrolera CAN mcp2515|

## Biblioteki
Dynamixel2Arduino - obsługa Protokołów Dynamixel 1.0 i 2.0. <br>
mcp2515.h - komunikacja magistrali CAN po SPI. <br>
SoftHalfDuplexSerial - wymagana tylko dla "dynamixel_wizard_soft", odpowiada za komunikację mikrokontrolera z serwami Dynamixel bez potrzeby wykorzystania portu UART.  <br>


## Konfiguracja

### dynamixel_wizard
Sprzętowa przelotka USB `Serial1` z przełączaniem kierunku na pinie `D6`.

### dynamixel_wizard_soft
Programowa przelotka Half-Duplex na pinie `D8`.

## Sterowanie nadgarstkiem

### dynamixel_wrist_firmware
Program podstawowy, na którym bazowane są programy do specyficznych przypadków. Obsługuje wszystkie 4 serwa w Protokole 2.0.

### dynamixel_wrist_firmware_ax12_gripper
Program do obsługi nadgarstka manipulatora z wykorzystaniem serwa AX12 do chwytaka. AX12 obsługiwane jest przez Protokół 1.0 dla Dynamixeli, reszta serw przez Protokół 2.0.

### dynamixel_wrist_firmware_ax12_gripper_interrupt
Wersja również wykorzystująca serwo AX12 do chwytaka. Posiada ścisłe filtry ramek CAN i obsługę czyszczenia przerwań mcp2515 (`CAN.clearInterrupts()`).

### dynamixel_wrist_firmware_broken_gear
Wersja z uszkodzoną przekładnią. Serwo o ID 2 nie wchodzi w tryb sterowania prądowego, ustawia tylko pozycję "0.0". Chroni zęby przekładni przed dalszym uszkodzeniem.

### dynamixel_wrist_firmware_no_gripper
Wersja sterowania nadgarstka bez chwytaka. Obsługuje tylko osie "0, 1 i 2".

### dynamixel_wrist_firmware_no_spill
Wersja do przenoszenia przedmiotów z ryzykiem wylania lub wysypania. Pomija otwieranie/zamykanie chwytaka podczas inicjalizacji, reset nie powoduje upuszczenia przedmiotu.

## Protokół komunikacji CAN
* ID robota (`canId`): `0x0` (osie 0, 1, 2)
* ID chwytaka (`gripperCanId`): `0x2c` (oś 3)

## Typy ramek
|Kod HEX|Nazwa|Opis|
|:---|:---|:---|
|`0x01`|`MSG_MOTOR_COMMAND`|Zadana pozycja osi (`CTRLSIGNAL_POSITION = 0x02`)|
|`0x02`|`MSG_ACTION_REQUEST`|Rozkazy systemowe (uzbrojenie - `0x00`, rozbrojenie - `0x01`, software reset - `0x05` )|
|`0x03`|`MSG_STATUS_REPORT`|Stan uzbrojenia (`armed`) i temperatura serwa|
|`0x04`|`MSG_AXIS_REPORT`|Aktualna pozycja i prędkość|
|`0x05`|`MSG_MOTOR_REPORT`|Pobór prądu i wypełnienie PWM|
