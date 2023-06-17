#include <Arduino.h>
#include <CAN.h>
#include <stdint.h>

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
    Sensor(uint16_t id, uint64_t frequency) : id(id), freq(frequency), data_len(sizeof(T))
    {
        send_micros = 1000000 / freq;
    }

    void set_value(uint8_t *data)
    {
        disabled = false;
        memcpy(buffer, data, data_len);
    }

    void set_value(T value)
    {
        disabled = false;
        static uint8_t data[sizeof(T)];
        to_be_bytes(value, data);
        memcpy(buffer, data, data_len);
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
            CAN.beginPacket(id);
            CAN.write(buffer, data_len);
            CAN.endPacket();
            last_send_micros = now;
        }
    }

private:
    uint16_t id;
    uint64_t freq;
    uint8_t buffer[8];
    size_t data_len;
    uint64_t send_micros;
    uint64_t last_send_micros;
    bool disabled = false;
};