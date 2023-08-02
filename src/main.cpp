#include <Arduino.h>
#include <CAN.h>
#include <SoftwareSerial.h>

#include "sensor.h"
#include "thermistor.h"

#define THERM_PIN A0
#define FLOW_PIN PB0

SoftwareSerial motor_serial(3, 4);

Sensor<uint16_t> s_motor_rpm(0b00001010010, 10, CAN);
Sensor<uint16_t> s_motor_current(0b01111110010, 10, CAN);
Sensor<uint16_t> s_motor_temp(0b00010110010, 10, CAN);
Sensor<uint16_t> s_motor_controller_temp(0b00011010010, 10, CAN);
Sensor<int32_t> s_motor_power(0b10000110010, 10, CAN);
Sensor<int8_t> s_throttle_position(0b00000010101, 10, CAN);
Sensor<uint16_t> s_coolant_temp(0b00111010100, 1, CAN); // A1: termistor
Sensor<uint16_t> s_coolant_flow(0b10101010100, 1, CAN); // PB0: flow

volatile int flow_in10ms;
unsigned long currentTime;
unsigned long cloopTime;
int l_hour;

void dump_bytes(uint8_t *ptr, size_t size)
{
    for (size_t pos = 0; pos < size; pos++)
    {
        if (ptr[pos] < 0x10)
            Serial.print('0');
        Serial.print(ptr[pos], HEX);
        Serial.print(' ');
        if ((pos + 1) % 16 == 0)
            Serial.println();
    }
    Serial.println();
}

void setup()
{
    pinMode(FLOW_PIN, INPUT);
    digitalWrite(FLOW_PIN, HIGH);
    // USB debugging
    Serial.begin(115200);

    // CAN init
    CAN.setPins(10, 2);
    CAN.setSPIFrequency(250000);
    CAN.setClockFrequency(8000000);

    // Flow sensor init
    currentTime = millis();
    cloopTime = currentTime;

    while (!CAN.begin(500000))
    {
        Serial.println("Starting CAN failed!");
        delay(100);
    }

    // Motor UART init
    motor_serial.begin(19200);
    pinMode(PIN7, INPUT);
}

float motor_voltage = 0.0f;
bool success = false;
static uint8_t motor_data_raw[47];
void read_motor_data()
{
    memset(motor_data_raw, 0, 45);
    motor_serial.write(0x80);
    motor_serial.write(0x8d);
    delay(5);
    // motor_serial.setTimeout(50);
    motor_serial.readBytes(motor_data_raw, 45);
    // dump_bytes(motor_data_raw, 45);

    if (motor_data_raw[0] == 0x7c && motor_data_raw[1] == 0x8d && motor_data_raw[43] == 0x7d)
    {
        success = true;
        int controller_temp = motor_data_raw[16] - 20;
        int motor_temp = motor_data_raw[17] - 20;
        int rpm = ((motor_data_raw[22] << 8) | motor_data_raw[21]) * 10;
        float current = ((motor_data_raw[29] << 8) | motor_data_raw[28]) / 10.0;
        motor_voltage = ((motor_data_raw[31] << 8) | motor_data_raw[30]) / 10.0;
        float power = current * motor_voltage;

        s_motor_rpm.set_value((uint16_t)(rpm));
        s_motor_current.set_value((uint16_t)(floor(current * 10)));
        s_motor_temp.set_value((uint16_t)(floor(motor_temp * 10)));
        s_motor_controller_temp.set_value((uint16_t)(floor(controller_temp * 10)));
        s_motor_power.set_value((int32_t)(floor(power * 1000)));

        int controller_tmp = motor_data_raw[16] - 20;
        int motor_tmp = motor_data_raw[17] - 20;
        int motor_rpm = ((motor_data_raw[22] << 8) | motor_data_raw[21]) * 10;
        int motor_current = (float)((motor_data_raw[29] << 8) | motor_data_raw[28]);
        int motor_voltage = (float)((motor_data_raw[31] << 8) | motor_data_raw[30]);
        int motor_power = (motor_current / 10.0) * (motor_voltage / 10.0);

        static char json_buffer[256] = {0};
        snprintf(json_buffer, 256, "{\"success\": true, \"controller_temp\": %d, \"motor_temp\": %d, \"motor_rpm\": %d, \"motor_current\": %d, \"motor_voltage\": %d, \"motor_power\": %d}", controller_tmp, motor_tmp, motor_rpm, motor_current, motor_voltage, motor_power);
        Serial.println(json_buffer);
    }
    else
    {
        s_motor_rpm.disable();
        s_motor_current.disable();
        s_motor_temp.disable();
        s_motor_controller_temp.disable();
        s_motor_power.disable();
        success = false;
    }
}

int8_t throttle_pos = 0;
void read_throttle()
{
    unsigned long pulse = pulseIn(PIN7, HIGH, 100000UL);
    if (pulse != 0)
    {
        int percent = map(pulse, 1280, 1830, -100, 100);
        throttle_pos = percent < -100 ? -100 : (percent > 100 ? 100 : percent);
        s_throttle_position.set_value(throttle_pos);
    }
    else
    {
        throttle_pos = 0;
        s_throttle_position.disable();
    }
}

THERMISTOR thermistor(THERM_PIN, 50000, 3950, 50000);
void read_termistor()
{
    uint16_t temp = thermistor.read() * 10;
    s_coolant_temp.set_value(temp);
}

void read_flow()
{
    cloopTime = currentTime;
    bool last = digitalRead(FLOW_PIN);
    // For 10 ms enable interrupt to count the high signs
    while(currentTime - cloopTime < 10) {
        bool val = digitalRead(FLOW_PIN);
        // If the pin's sign changed from low to high increase the counter
        if(val != last) {
            if(last == false) {
                flow_in10ms++;
            }
            last = val;
        }
        currentTime = millis();
    }

    // Pulse frequency (Hz) = 11 Q, Q is flow rate in L/min.
    l_hour = (flow_in10ms * 6000 / 11.0); // (Pulse frequency in half second x 60 min) / 11 Q = flowrate in L/hour
    flow_in10ms = 0; // Reset Counter
}

void loop()
{
    Serial.println("loop");
    read_motor_data();
    read_throttle();
    s_motor_rpm.send();
    s_motor_current.send();
    s_motor_temp.send();
    s_motor_controller_temp.send();
    s_motor_power.send();
    s_throttle_position.send();
    s_coolant_temp.send();
    s_coolant_flow.send();

    Serial.print((int)throttle_pos);
    Serial.println();

    CAN.clearWriteError();
    delay(1);
}