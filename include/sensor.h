#include <Arduino.h>
#include <CAN.h>
#include <stdint.h>

/**
 * @brief 
 * 
 */


/**
 * @brief 
 * 
 * @tparam T 
 * @param datatype 
 * @param out 
 */
template <typename T>
inline void to_be_bytes(T datatype, u8 *out)
{

    size_t byte_count = sizeof(T);

    // we cannot use reinterpret_cast<u8*>... as that depends on the platform

    // endianness

    for (size_t byte_nc = byte_count; byte_nc > 0; byte_nc--)
    {

        size_t byte_n = byte_nc - 1;

        T mask = (0xff << (byte_n * 8));

        u8 byte = (datatype & mask) >> (byte_n * 8);

        out[byte_count - byte_n - 1] = byte;
    }
}

template <typename T>
inline T from_be_bytes(const u8 *bytes)
{

    auto byte_count = sizeof(T);

    T value = 0;

    for (u32 byte_n = 0; byte_n < byte_count; byte_n++)
    {

        u8 byte = bytes[byte_n];

        auto offset = byte_count - byte_n - 1;

        T v = ((T)byte << (offset * 8));

        value |= v;
    }

    return value;
}

template <typename T>
class Sensor
{
public:
    Sensor(uint16_t id, uint64_t frequency, MCP2515Class &can) : id(id), freq(frequency), can(can)
    {
        send_micros = 1000000 / freq;
    }

    void set_value(uint8_t *data)
    {
        disabled = false;
        memcpy(buffer, data, sizeof(T));
    }

    void set_value(T value)
    {
        disabled = false;
        to_be_bytes(value, buffer);
    }

    T get_value()
    {
        return from_be_bytes<T>(reinterpret_cast<uint8_t *>(buffer));
    }

    void disable()
    {
        disabled = true;
    }

    void send()
    {
        if (disabled)
            return;
        uint64_t now = micros();
        if (now - last_send_micros > send_micros)
        {
            can.beginPacket(id);
            can.write(buffer, sizeof(T));
            if (!can.endPacket())
            {
                Serial.print(id, HEX);
                Serial.print(" ");
                Serial.println("failed");
            }
            else
            {

                last_send_micros = now;
                Serial.print(id, HEX);
                Serial.print(" ");
                Serial.print(sizeof(T));
                Serial.print(" ");
                Serial.println("sent");
            }
        }
    }

private:
    uint16_t id;
    uint64_t freq;
    uint8_t buffer[sizeof(T)];
    uint64_t send_micros;
    uint64_t last_send_micros;
    bool disabled = false;
    MCP2515Class &can;
};