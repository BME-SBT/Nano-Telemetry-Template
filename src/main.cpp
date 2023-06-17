#include <Arduino.h>
#include <CAN.h>
#include <SoftwareSerial.h>

#include "sensor.h"

SoftwareSerial motor_serial(3, 4);
Sensor<uint16_t> motor_rpm(0b00001010010, 10);
Sensor<uint16_t> motor_current(0b01111110010, 10);
Sensor<uint16_t> motor_temp(0b00010110010, 10);
Sensor<uint16_t> motor_controller_temp(0b00011010010, 10);
Sensor<int32_t> motor_power(0b10000110010, 10);
void setup()
{
    // USB debugging
    Serial.begin(115200);

    // CAN init
    CAN.setPins(10, 2);
    CAN.setSPIFrequency(100E3);
    CAN.setClockFrequency(8E6);
    while (!CAN.begin(500E3))
    {
        Serial.println("Starting CAN failed!");
        delay(100);
    }

    // Motor UART init
    motor_serial.begin(19200);
}
static uint8_t motor_data_raw[47];
void read_motor_data()
{

    motor_serial.write(0x80);
    motor_serial.write(0x8d);
    delay(5);
    motor_serial.setTimeout(10);
    motor_serial.readBytes(motor_data_raw, 47);
    if (motor_data_raw[2] == 0x7c && motor_data_raw[3] == 0x8d)
    {
        if (motor_data_raw[45] == 0x7d)
        {
            int controller_temp = motor_data_raw[18] - 20;
            int motor_tmp = motor_data_raw[19] - 20;
            int rpm = ((motor_data_raw[24] << 8) | motor_data_raw[23]) * 10;
            float current = ((motor_data_raw[31] << 8) | motor_data_raw[30]) / 10.0;
            float motor_voltage = ((motor_data_raw[33] << 8) | motor_data_raw[32]) / 10.0;
            float power = current * motor_voltage;
            motor_rpm.set_value((uint16_t)(rpm));
            motor_current.set_value((uint16_t)(floor(current * 10)));
            motor_temp.set_value((uint16_t)(floor(motor_tmp * 10)));
            motor_controller_temp.set_value((uint16_t)(floor(controller_temp * 10)));
            motor_power.set_value((int32_t)(floor(power * 1000)));
        }
    }
    else
    {
        motor_rpm.disable();
        motor_current.disable();
        motor_temp.disable();
        motor_controller_temp.disable();
        motor_power.disable();
    }
}

void loop()
{
    read_motor_data();
    motor_rpm.send();
    motor_current.send();
    motor_temp.send();
    motor_controller_temp.send();
    motor_power.send();
    delay(1);
}