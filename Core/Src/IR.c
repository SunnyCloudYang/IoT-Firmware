#include "stm32f10x.h"
#include "IR.h"

// Define the infrared transmitter pin
#define IR_SENDER_GPIO_PORT GPIOA
#define IR_SENDER_GPIO_PIN GPIO_Pin_1

// Define the infrared transmitter protocol parameters
#define IR_PROTOCOL_FREQ 38000               // Infrared protocol frequency
#define IR_PROTOCOL_START_DURATION 9000      // Infrared protocol start bit duration
#define IR_PROTOCOL_LEADIN_DURATION 4500     // Infrared protocol lead-in bit duration
#define IR_PROTOCOL_DATA_BIT_0_DURATION 560  // Infrared protocol data bit 0 duration
#define IR_PROTOCOL_DATA_BIT_1_DURATION 1690 // Infrared protocol data bit 1 duration

// Send an infrared protocol signal
void IR_SendProtocol(uint32_t protocol)
{
    uint8_t i, j;
    uint32_t data_bit;

    // Send start bit
    GPIO_SetBits(IR_SENDER_GPIO_PORT, IR_SENDER_GPIO_PIN);
    delay_us(IR_PROTOCOL_START_DURATION);

    // Send lead-in bit
    GPIO_ResetBits(IR_SENDER_GPIO_PORT, IR_SENDER_GPIO_PIN);
    delay_us(IR_PROTOCOL_LEADIN_DURATION);
    GPIO_SetBits(IR_SENDER_GPIO_PORT, IR_SENDER_GPIO_PIN);
    delay_us(IR_PROTOCOL_LEADIN_DURATION);

    // Send protocol data bit by bit
    for (i = 0; i < 14; i++)
    {
        data_bit = (protocol >> i) & 0x01; // Get the value of the current bit

        // Send data bit
        GPIO_ResetBits(IR_SENDER_GPIO_PORT, IR_SENDER_GPIO_PIN);
        if (data_bit)
        {
            delay_us(IR_PROTOCOL_DATA_BIT_1_DURATION);
        }
        else
        {
            delay_us(IR_PROTOCOL_DATA_BIT_0_DURATION);
        }
        GPIO_SetBits(IR_SENDER_GPIO_PORT, IR_SENDER_GPIO_PIN);
        delay_us(IR_PROTOCOL_DATA_BIT_0_DURATION);
    }
}

// Control the power of the Gree air conditioner
void IR_SetPower(uint8_t on_off)
{
    uint32_t protocol;

    if (on_off)
    {
        // Power on
        protocol = 0x8820000200200000;
    }
    else
    {
        // Power off
        protocol = 0x8820000000200000;
    }

    IR_SendProtocol(protocol);
}

// Control the temperature of the Gree air conditioner
void IR_SetTemperature(uint8_t temperature)
{
    uint32_t protocol;

    if (temperature < 16 || temperature > 31)
    {
        return; // Invalid temperature range
    }

    // Temperature code is 0x20 plus the actual temperature value
    protocol = 0x8820002020000000 | (temperature - 16);

    IR_SendProtocol(protocol);
}

// Control the mode of the Gree air conditioner
void IR_SetMode(uint8_t mode)
{
    uint32_t protocol;

    switch (mode)
    {
    case 0: // Cooling mode
        protocol = 0x8820000200200000;
        break;
    case 1: // Heating mode
        protocol = 0x8820000400200000;
        break;
    case 2: // Auto mode
        protocol = 0x8820000800200000;
        break;
    case 3: // Fan mode
        protocol = 0x8820001000200000;
        break;
    default:
        return; // Invalid mode
    }

    IR_SendProtocol(protocol);
}

// Control the fan speed of the Gree air conditioner
void IR_SetFanSpeed(uint8_t fan_speed)
{
    uint32_t protocol;

    switch (fan_speed)
    {
    case 0: // Auto fan speed
        protocol = 0x8820002000200000;
        break;
    case 1: // Low fan speed
        protocol = 0x8820004000200000;
        break;
    case 2: // Medium fan speed
        protocol = 0x8820008000200000;
        break;
    case 3: // High fan speed
        protocol = 0x8820010000200000;
        break;
    default:
        return; // Invalid fan speed
    }

    IR_SendProtocol(protocol);
}
