#undef max
#undef min

#include "SerialComOptimizer.h"

SerialComOptimizer usbSerial(&Serial);
SerialComOptimizer hardwareSerial(&Serial1);

void setup()
{
    Serial.begin(115200);
    Serial1.begin(115200);
}

void loop()
{
    usbSerial.collectReadData();
    while (usbSerial.available() > 0)
    {
        hardwareSerial.write(usbSerial.read());
    }
    hardwareSerial.sendWrittenData();

    hardwareSerial.collectReadData();
    while (hardwareSerial.available() > 0)
    {
        usbSerial.write(hardwareSerial.read());
    }
    usbSerial.sendWrittenData();
}
