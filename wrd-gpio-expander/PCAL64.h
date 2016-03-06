/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __GPIO_PCAL64_H__
#define __GPIO_PCAL64_H__

#include "mbed-drivers/mbed.h"
#include "wrd-utilities/I2CRegister.h"

using namespace mbed::util;

class PCAL64
{
public:
    typedef enum {
        PRIMARY_ADDRESS   = 0x40,
        SECONDARY_ADDRESS = 0x42
    } address_t;

    typedef enum {
        P0_0 = (1 <<  0),
        P0_1 = (1 <<  1),
        P0_2 = (1 <<  2),
        P0_3 = (1 <<  3),
        P0_4 = (1 <<  4),
        P0_5 = (1 <<  5),
        P0_6 = (1 <<  6),
        P0_7 = (1 <<  7),

        P1_0 = (1 <<  8),
        P1_1 = (1 <<  9),
        P1_2 = (1 << 10),
        P1_3 = (1 << 11),
        P1_4 = (1 << 12),
        P1_5 = (1 << 13),
        P1_6 = (1 << 14),
        P1_7 = (1 << 15),
        Pin_End
    } pin_t;

    PCAL64(PinName sda, PinName scl, uint16_t address, PinName irq = NC);
    ~PCAL64(void);

    /**
     * @brief Read pin values.
     * @details The result is passed as a parameter in the callback function.
     *
     * @param callback Function with pin values as parameter.
     * @return Boolean result. True means command was accepted, False means it was not.
     */
    bool bulkRead(FunctionPointer1<void, uint32_t> callback);

    /**
     * @brief Set direction and values for at most 32 pins.
     * @details Pins are labeled LSB. For I/O expanders with less than 32 pins
     *          the higher bits are ignored. I/O expanders with more than 32 pins
     *          are not supported.
     *
     * @param pins The pins affected by this call are set high in bitmap (LSB).
     * @param directions Pin directions. 0 means input, 1 means output.
     * @param values Pin values. 0 means low, 1 means high.
     * @param callback Function to call when I/O expander is ready for next command.
     * @return Boolean result. True means command was accepted, False means it was not.
     */
    bool bulkWrite(uint32_t pins, uint32_t directions, uint32_t values, FunctionPointer0<void> callback);

    /**
     * @brief Toggles output on given pins.
     * @details Only affects pins that are already set to output.
     *
     * @param pins The pins affected by this call are set high in bitmap (LSB).
     * @param callback Function to call when I/O expander is ready for next command.
     * @return Boolean result. True means command was accepted, False means it was not.
     */
    bool bulkToggle(uint32_t pins, FunctionPointer0<void> callback);

    /**
     * @brief Set pins to be trigger interrupts.
     * @details When interrupts are triggered the callback handler contains the pin values.
     *
     * @param pins Pins affected by this call.
     * @param values Interrupt mask. 0 interrupt is disabled, 1 interrupt is enabled.
     * @param callback Function is called when next command can be send.
     * @return Boolean result. True means command was accepted, False means it was not.
     */
    bool bulkSetInterrupt(uint32_t pins, uint32_t values, FunctionPointer0<void> callback);

    /**
     * @brief Interrupt callback function.
     * @details The callback function has the I2C address, fired pins, and pin values
     *          as parameters. The address is passed so a single callback function can
     *          handle multiple expanders. The pins and values are useful for determining
     *          which pins have fired and what edge triggered the interrupt.
     *
     * @param uint16_t address
     * @param uint32_t pins
     * @param uint32_t values
     */
    typedef FunctionPointer3<void, uint16_t, uint32_t, uint32_t> IRQCallback_t;

    /**
     * @brief Callback function for when interrupts have fired.
     * @details The fired pins and values are passes as arguments in callback function.
     *
     * @param callback Parameters: address, fired pins, and pin values.
     */
    void setInterruptHandler(IRQCallback_t callback);

    /**
     * @brief Clear handler set with setInterruptHandler.
     */
    void clearInterruptHandler(void);

private:

    void eventHandler(void);
    void internalHandlerIRQ(void);
    void internalHandlerTask(void);

    I2CRegister i2c;
    uint16_t address;
    InterruptIn irq;

    uint16_t pins;
    uint16_t param1;
    uint16_t param2;
    uint16_t cache;

    uint16_t backupStatus;
    uint16_t backupValues;

    uint8_t readBuffer[2];

    FunctionPointer0<void>                               externalDoneHandler;
    FunctionPointer1<void, uint32_t>                     externalReadHandler;
    FunctionPointer3<void, uint16_t, uint32_t, uint32_t> externalIRQHandler;

    typedef enum {
        STATE_READ_GET_STATUS,
        STATE_READ_GET_VALUES,
        STATE_WRITE_GET_DIRECTIONS,
        STATE_WRITE_SET_DIRECTIONS,
        STATE_WRITE_GET_VALUES,
        STATE_TOGGLE_GET_VALUES,
        STATE_INTERRUPT_GET_DIRECTIONS,
        STATE_INTERRUPT_SET_DIRECTIONS,
        STATE_INTERRUPT_GET_LATCH,
        STATE_INTERRUPT_SET_LATCH,
        STATE_INTERRUPT_GET_MASK,
        STATE_INTERRUPT_GET_STATUS,
        STATE_INTERRUPT_GET_VALUES,
        STATE_SIGNAL_DONE,
        STATE_IDLE
    } state_t;

    state_t state;

    typedef enum {
        INPUT_PORT_0                    = 0x00,
        INPUT_PORT_1                    = 0x01,
        OUTPUT_PORT_0                   = 0x02,
        OUTPUT_PORT_1                   = 0x03,
        POLARITY_INVERSION_PORT_0       = 0x04,
        POLARITY_INVERSION_PORT_1       = 0x05,
        CONFIGURATION_PORT_0            = 0x06,
        CONFIGURATION_PORT_1            = 0x07,

        OUTPUT_DRIVE_STRENGTH_0_LOW     = 0x40,
        OUTPUT_DRIVE_STRENGTH_0_HIGH    = 0x41,
        OUTPUT_DRIVE_STRENGTH_1_LOW     = 0x42,
        OUTPUT_DRIVE_STRENGTH_1_HIGH    = 0x43,
        INPUT_LATCH_0                   = 0x44,
        INPUT_LATCH_1                   = 0x45,
        PULL_UP_DOWN_ENABLE_0           = 0x46,
        PULL_UP_DOWN_ENABLE_1           = 0x47,
        PULL_UP_DOWN_SELECTION_0        = 0x48,
        PULL_UP_DOWN_SELECTION_1        = 0x49,
        INTERRUPT_MASK_0                = 0x4A,
        INTERRUPT_MASK_1                = 0x4B,
        INTERRUPT_STATUS_0              = 0x4C,
        INTERRUPT_STATUS_1              = 0x4D,
        OUTPUT_PORT_CONFIGURATION       = 0x4F
    } register_t;
};

#endif // __GPIO_PCAL64_H__
