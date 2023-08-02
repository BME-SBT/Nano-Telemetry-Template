#include <Arduino.h>
#include <CAN.h>
#include <stdint.h>

/**
 * @brief Header files of the telemtry sensors used by the BME Solar Boat Team
 * the sensors use the team's own CAN protocol designed by Mark Bakonyi
 */


/**
 * @brief Converts the c type data to binary data in little-endian
 * 
 * @tparam T the c type datatype
 * @param datatype the input data
 * @param out the buffer to write the bianry data to
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

/**
 * @brief Converts the little-endian data to the specified c type datatype from binary
 * 
 * @tparam T the c type datatype
 * @param bytes bytes to convert
 * @return T the converted data
 */
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

/**
 * @brief Telemetry sensor connected to the CAN bus
 * each sensor is writing its value on the CAN with the frequency specified in the contructor
 * @tparam T data type which the sensor sends its datas in to the CAN
 */
template <typename T>
class Sensor
{
public:
    /**
     * @brief Construct a new Sensor object
     * 
     * @param id the CAN id of the sensor
     * @param frequency the frequency to write the data on the CAN with
     * @param can reference of the MCP2515 CAN module object
     */
    Sensor(uint16_t id, uint64_t frequency, MCP2515Class &can) : id(id), freq(frequency), can(can)
    {
        send_micros = 1000000 / freq;
    }

    /**
     * @brief Sets the current value of the sensor
     * 
     * @param data the value in binary
     */
    void set_value(uint8_t *data)
    {
        disabled = false;
        memcpy(buffer, data, sizeof(T));
    }

    /**
     * @brief Sets the current value of the sensor
     * 
     * @param value the value in c datatype
     */
    void set_value(T value)
    {
        disabled = false;
        to_be_bytes(value, buffer);
    }

    /**
     * @brief Get the current value of the sensor
     * 
     * @return T value of the sensor
     */
    T get_value()
    {
        return from_be_bytes<T>(reinterpret_cast<uint8_t *>(buffer));
    }

    /**
     * @brief Disable the sensor
     * 
     */
    void disable()
    {
        disabled = true;
    }

    /**
     * @brief Sends CAN packet to the bus with the current value of the sensor
     * 
     */
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
    /** @brief The id of the sensor specified in the protocol */
    uint16_t id;
    /** @brief The frequency to update with */
    uint64_t freq;
    /** @brief The buffer where the current value of the sensor is stored in little-endian binary */
    uint8_t buffer[sizeof(T)];
    /** @brief The period to send the data with in micro seconds */
    uint64_t send_micros;
    /** @brief Micro seconds since the last update */
    uint64_t last_send_micros;
    /** @brief Indicates whether the sensor is disabled */
    bool disabled = false;
    /** @brief reference of the MCP2515 CAN module*/
    MCP2515Class &can;
};