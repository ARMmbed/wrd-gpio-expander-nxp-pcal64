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

#include "mbed-drivers/mbed.h"
#include "gpio-pcal64/PCAL64.h"

/*****************************************************************************/
/* PCAL64                                                                    */
/*****************************************************************************/

PCAL64 ioexpander0(YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_EXTERNAL_GPIO_GPIO0_I2C_SDA,
                   YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_EXTERNAL_GPIO_GPIO0_I2C_SCL,
                   YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_EXTERNAL_GPIO_GPIO0_I2C_ADDRESS,
                   YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_EXTERNAL_GPIO_GPIO0_IRQ_PIN);

PCAL64 ioexpander1(YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_EXTERNAL_GPIO_GPIO1_I2C_SDA,
                   YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_EXTERNAL_GPIO_GPIO1_I2C_SCL,
                   YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_EXTERNAL_GPIO_GPIO1_I2C_ADDRESS,
                   YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_EXTERNAL_GPIO_GPIO1_IRQ_PIN);


void irqHandler(uint16_t address, uint32_t pins, uint32_t values)
{
    printf("%02X: %lu %lu\r\n", address, pins, values);
}

void writeDone(void)
{

}

void irqDone()
{
    ioexpander1.bulkWrite(PCAL64::P0_6, PCAL64::P0_6, 0, writeDone);
}

/*****************************************************************************/
/* Debug                                                                     */
/*****************************************************************************/

// enable buttons to initiate transfer
static InterruptIn button1(YOTTA_CFG_HARDWARE_WEARABLE_REFERENCE_DESIGN_BUTTON_FORWARD_GPIO_PIN);

void readDone(uint32_t values)
{
    printf("%lu\r\n", values);
}

void toggleDone()
{
    printf("toggle done\r\n");

    ioexpander0.bulkRead(readDone);
}

void button1Task()
{
    ioexpander1.bulkToggle(PCAL64::P0_6, toggleDone);
}

void button1ISR()
{
    minar::Scheduler::postCallback(button1Task);
}

/*****************************************************************************/
/* App start                                                                 */
/*****************************************************************************/

void app_start(int, char *[])
{
    // setup buttons
    button1.fall(button1ISR);

    ioexpander0.setInterruptHandler(irqHandler);
    ioexpander1.setInterruptHandler(irqHandler);

    ioexpander0.bulkSetInterrupt(PCAL64::P0_0, PCAL64::P0_0, irqDone);
}
