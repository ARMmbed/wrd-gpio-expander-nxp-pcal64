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

#include "wrd-gpio-expander/PCAL64.h"

PCAL64::PCAL64(PinName sda, PinName scl, uint16_t _address, PinName _irq)
    :   i2c(sda, scl),
        address(_address),
        irq(_irq),
        backupStatus(0),
        backupValues(0),
        state(STATE_IDLE)
{
    i2c.frequency(400000);

    if (_irq != NC)
    {
        irq.fall(this, &PCAL64::internalHandlerIRQ);
    }
}

PCAL64::~PCAL64()
{
    irq.fall(NULL);
}

bool PCAL64::bulkRead(FunctionPointer1<void, uint32_t> callback)
{
    bool result = false;

    if (state == STATE_IDLE)
    {
        state = STATE_READ_GET_STATUS;
        externalReadHandler = callback;

        /* Read the interrupt status register before reading the input register.
           This is to prevent accidentally erasing the interrupt status register
           before the interrupt handler has had a chance to read it.

           The status and input values are cached in case the interrupt handler
           needs them.
        */
        FunctionPointer0<void> fp(this, &PCAL64::eventHandler);
        result = i2c.read(address, INTERRUPT_STATUS_0, readBuffer, 2, fp);
    }

    return result;
}

bool PCAL64::bulkWrite(uint32_t _pins, uint32_t directions, uint32_t values, FunctionPointer0<void> callback)
{
    bool result = false;

    if (state == STATE_IDLE)
    {
        state = STATE_WRITE_GET_DIRECTIONS;
        externalDoneHandler = callback;

        pins = _pins;

        /* NOTE: the PCAL64 defines 0 to be output and 1 to be input.
           This is opposite from the gpio-expander API, hence the invesion.
        */
        param1 = ~directions;
        param2 = values;

        FunctionPointer0<void> fp(this, &PCAL64::eventHandler);
        result = i2c.read(address, CONFIGURATION_PORT_0, readBuffer, 2, fp);
    }

    return result;
}

bool PCAL64::bulkToggle(uint32_t _pins, FunctionPointer0<void> callback)
{
    bool result = false;

    if (state == STATE_IDLE)
    {
        state = STATE_TOGGLE_GET_VALUES;
        externalDoneHandler = callback;

        pins = _pins;

        FunctionPointer0<void> fp(this, &PCAL64::eventHandler);
        result = i2c.read(address, OUTPUT_PORT_0, readBuffer, 2, fp);
    }

    return result;
}

bool PCAL64::bulkSetInterrupt(uint32_t _pins, uint32_t values, FunctionPointer0<void> callback)
{
    bool result = false;

    if (state == STATE_IDLE)
    {
        state = STATE_INTERRUPT_GET_DIRECTIONS;
        externalDoneHandler = callback;

        pins = _pins;
        param1 = values;

        FunctionPointer0<void> fp(this, &PCAL64::eventHandler);
        result = i2c.read(address, CONFIGURATION_PORT_0, readBuffer, 2, fp);
    }

    return result;
}

void PCAL64::setInterruptHandler(FunctionPointer3<void, uint16_t, uint32_t, uint32_t> callback)
{
    externalIRQHandler = callback;
}

void PCAL64::clearInterruptHandler(void)
{
    externalIRQHandler.clear();
}

void PCAL64::internalHandlerIRQ(void)
{
    minar::Scheduler::postCallback(this, &PCAL64::internalHandlerTask)
        .tolerance(1);
}

void PCAL64::internalHandlerTask(void)
{
    if (state == STATE_IDLE)
    {
        state = STATE_INTERRUPT_GET_STATUS;

        FunctionPointer0<void> fp(this, &PCAL64::eventHandler);
        i2c.read(address, INTERRUPT_STATUS_0, readBuffer, 2, fp);
    }
    else
    {
        // transfer in progress, repost IRQ handler with 1 ms delay
        minar::Scheduler::postCallback(this, &PCAL64::internalHandlerTask)
            .delay(minar::milliseconds(1))
            .tolerance(1);
    }
}

void PCAL64::eventHandler()
{
    switch (state)
    {
        /*********************************************************************/
        /* bulkRead                                                          */
        /*********************************************************************/
        case STATE_READ_GET_STATUS:
            {
                state = STATE_READ_GET_VALUES;

                uint32_t status = readBuffer[1];
                status = (status << 8) | readBuffer[0];

                backupStatus = status;

                FunctionPointer0<void> fp(this, &PCAL64::eventHandler);
                i2c.read(address, INPUT_PORT_0, readBuffer, 2, fp);
            }
            break;

        case STATE_READ_GET_VALUES:
            {
                state = STATE_IDLE;

                uint32_t values = readBuffer[1];
                values = (values << 8) | readBuffer[0];

                backupValues = values;

                if (externalReadHandler)
                {
                    minar::Scheduler::postCallback(externalReadHandler.bind(values))
                        .tolerance(1);
                }
            }
            break;

        /*********************************************************************/
        /* bulkWrite                                                         */
        /*********************************************************************/
        case STATE_WRITE_GET_DIRECTIONS:
            {
                state = STATE_WRITE_SET_DIRECTIONS;

                uint16_t directions = readBuffer[1];
                directions = (directions << 8) | readBuffer[0];

                /* pins are input when bit is 1 */

                // enable bits
                directions |= (pins & param1);

                // disable bits
                directions &= ~(pins & ~param1);

                uint8_t writeBuffer[2];
                writeBuffer[0] = directions;
                writeBuffer[1] = directions >> 8;

                FunctionPointer0<void> fp(this, &PCAL64::eventHandler);
                i2c.write(address, CONFIGURATION_PORT_0, writeBuffer, 2, fp);
            }
            break;

        case STATE_WRITE_SET_DIRECTIONS:
            {
                state = STATE_WRITE_GET_VALUES;

                FunctionPointer0<void> fp(this, &PCAL64::eventHandler);
                i2c.read(address, OUTPUT_PORT_0, readBuffer, 2, fp);
            }
            break;

        case STATE_WRITE_GET_VALUES:
            {
                state = STATE_SIGNAL_DONE;

                uint16_t values = readBuffer[1];
                values = (values << 8) | readBuffer[0];

                /* pins are high when bit is 1 */

                // enable bits
                values |= (pins & param2);

                // disable bits
                values &= ~(pins & ~param2);

                uint8_t writeBuffer[2];
                writeBuffer[0] = values;
                writeBuffer[1] = values >> 8;

                FunctionPointer0<void> fp(this, &PCAL64::eventHandler);
                i2c.write(address, OUTPUT_PORT_0, writeBuffer, 2, fp);
            }
            break;

        /*********************************************************************/
        /* bulkToggle                                                        */
        /*********************************************************************/
        case STATE_TOGGLE_GET_VALUES:
            {
                state = STATE_SIGNAL_DONE;

                uint16_t values = readBuffer[1];
                values = (values << 8) | readBuffer[0];

                uint16_t original = values;

                // enable bits
                values |= (pins & ~original);

                // disable bits
                values &= ~(pins & original);

                uint8_t writeBuffer[2];
                writeBuffer[0] = values;
                writeBuffer[1] = values >> 8;

                FunctionPointer0<void> fp(this, &PCAL64::eventHandler);
                i2c.write(address, OUTPUT_PORT_0, writeBuffer, 2, fp);
            }
            break;

        /*********************************************************************/
        /* bulkInterrupt                                                     */
        /*********************************************************************/
        case STATE_INTERRUPT_GET_DIRECTIONS:
            {
                state = STATE_INTERRUPT_SET_DIRECTIONS;

                uint16_t directions = readBuffer[1];
                directions = (directions << 8) | readBuffer[0];

                /* pins are input when bit is 1 */

                // enable bits
                directions |= (pins & param1);

                // disable bits
                directions &= ~(pins & ~param1);

                uint8_t writeBuffer[2];
                writeBuffer[0] = directions;
                writeBuffer[1] = directions >> 8;

                FunctionPointer0<void> fp(this, &PCAL64::eventHandler);
                i2c.write(address, CONFIGURATION_PORT_0, writeBuffer, 2, fp);
            }
            break;

        case STATE_INTERRUPT_SET_DIRECTIONS:
            {
                state = STATE_INTERRUPT_GET_LATCH;

                FunctionPointer0<void> fp(this, &PCAL64::eventHandler);
                i2c.read(address, INPUT_LATCH_0, readBuffer, 2, fp);
            }
            break;

        case STATE_INTERRUPT_GET_LATCH:
            {
                state = STATE_INTERRUPT_SET_LATCH;

                uint16_t latch = readBuffer[1];
                latch = (latch << 8) | readBuffer[0];

                /* pins are latched when bit is 1 */

                // enable bits
                latch |= (pins & param1);

                // disable bits
                latch &= ~(pins & ~param1);

                uint8_t writeBuffer[2];
                writeBuffer[0] = latch;
                writeBuffer[1] = latch >> 8;

                FunctionPointer0<void> fp(this, &PCAL64::eventHandler);
                i2c.write(address, INPUT_LATCH_0, writeBuffer, 2, fp);

            }
            break;

        case STATE_INTERRUPT_SET_LATCH:
            {
                state = STATE_INTERRUPT_GET_MASK;

                FunctionPointer0<void> fp(this, &PCAL64::eventHandler);
                i2c.read(address, INTERRUPT_MASK_0, readBuffer, 2, fp);
            }
            break;

        case STATE_INTERRUPT_GET_MASK:
            {
                state = STATE_SIGNAL_DONE;

                uint16_t values = readBuffer[1];
                values = (values << 8) | readBuffer[0];

                /* interrupts are enabled when bit is 0 */

                // enable bits
                values |= (pins & ~param1);

                // disable bits
                values &= ~(pins & param1);

                uint8_t writeBuffer[2];
                writeBuffer[0] = values;
                writeBuffer[1] = values >> 8;

                FunctionPointer0<void> fp(this, &PCAL64::eventHandler);
                i2c.write(address, INTERRUPT_MASK_0, writeBuffer, 2, fp);
            }
            break;

        /*********************************************************************/
        /* IRQ handler                                                       */
        /*********************************************************************/
        case STATE_INTERRUPT_GET_STATUS:
            {
                state = STATE_INTERRUPT_GET_VALUES;

                cache = readBuffer[1];
                cache = (cache << 8) | readBuffer[0];

                FunctionPointer0<void> fp(this, &PCAL64::eventHandler);
                i2c.read(address, INPUT_PORT_0, readBuffer, 2, fp);
            }
            break;

        case STATE_INTERRUPT_GET_VALUES:
            {
                state = STATE_IDLE;

                uint16_t values = readBuffer[1];
                values = (values << 8) | readBuffer[0];

                if (externalIRQHandler)
                {
                    /* A normal read call can clear interrupts if it is already
                       running when an interrupt fires. So all read calls also
                       reads and stores the interrupt status register and the
                       pin values in a cache in the odd event that a read call
                       cleares the status register before the interrupt handler
                       gets to read it.

                       In this particular case the status register will be zero
                       and we can substitute it with the cached version instead.
                    */
                    if (cache)
                    {
                        minar::Scheduler::postCallback(externalIRQHandler.bind(address, cache, values))
                            .tolerance(1);
                    }
                    else
                    {
                        minar::Scheduler::postCallback(externalIRQHandler.bind(address, backupStatus, backupValues))
                            .tolerance(1);
                    }
                }
            }
            break;

        /*********************************************************************/
        /* signal done                                                       */
        /*********************************************************************/
        case STATE_SIGNAL_DONE:
            {
                state = STATE_IDLE;

                if (externalDoneHandler)
                {
                    minar::Scheduler::postCallback(externalDoneHandler)
                        .tolerance(1);
                }
            }
            break;

        default:
            state = STATE_IDLE;
            break;
    }
}
