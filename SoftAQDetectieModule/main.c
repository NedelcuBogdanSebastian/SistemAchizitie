/*************************************************************************************
    Copyright (C) 2024 Nedelcu Bogdan Sebastian
    This code is free software: you can redistribute it and/or modify it
    under the following conditions:
    1. The use, distribution, and modification of this file are permitted for any
       purpose, provided that the following conditions are met:
    2. Any redistribution or modification of this file must retain the original
       copyright notice, this list of conditions, and the following attribution:
       "Original work by Nedelcu Bogdan Sebastian."
    3. The original author provides no warranty regarding the functionality or fitness
       of this software for any particular purpose. Use it at your own risk.
    By using this software, you agree to retain the name of the original author in any
    derivative works or distributions.
    ------------------------------------------------------------------------
    This code is provided as-is, without any express or implied warranties.
**************************************************************************************/

/*
    We run this on the GOD ol' Matchbox :). Adapted with 10k pull up resistor and diode on USART3,
    to interrogate all possible modules, and see which answer.

    It shows as USB SERIAL on the PC side, and we can use Termite Serial to see the output.
*/

// If we need to store signed values we use 2's complement
// int16_t val = -100;
// uint16_t number = (uint16_t) val

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

/* Private function prototypes -----------------------------------------------*/

// dec -> hex, hex -> dec
#define TO_HEX(i) (i <= 9 ? '0' + i : 'A' - 10 + i)
#define TO_DEC(i) (i <= '9'? i - '0': i - 'A' + 10)
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

void USART3_Putch(unsigned char ch);
void USART3andUSB_Print(char s[]);
void USART3_Init(void);

/*
CRC Generation from the MODBUS Specification V1.02:
1. Load a 16bit register with FFFF hex (all 1’s). Call this the CRC register.
2. Exclusive OR the first 8bit byte of the message with the low order byte of the 16 bit CRC register, putting the result in the
CRC register.
3. Shift the CRC register one bit to the right (toward the LSB), zero filling the MSB.
Extract and examine the LSB.
4. (If the LSB was 0): Repeat Step 3 (another shift).
(If the LSB was 1): Exclusive OR the CRC register with the polynomial value 0xA001 (1010 0000 0000 0001).
5. Repeat Steps 3 and 4 until 8 shifts have been performed. When this is done, a complete 8bit byte will have been processed.
6. Repeat Steps 2 through 5 for the next 8bit byte of the message. Continue doing this until all bytes have been processed.
7. The final content of the CRC register is the CRC value.
8. When the CRC is placed into the message, its upper and lower bytes must be swapped.
*/
uint16_t crc16_update(uint16_t crc, uint8_t a) {
    uint8_t i;

    crc ^= a;
    for (i = 0; i < 8; ++i) {
        if (crc & 1) crc = (crc >> 1) ^ 0xA001;
        else crc = (crc >> 1);
    }

    return crc;
}

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

volatile uint32_t TimingDelay;

/* Extern variables ----------------------------------------------------------*/
volatile uint8_t TestBuffer[27];

extern volatile uint8_t Receive_Buffer[64];
extern volatile uint32_t Receive_length ;
extern volatile uint32_t length ;
uint8_t Send_Buffer[64];
uint32_t packet_sent=1;
uint32_t packet_receive=1;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

void Delay(volatile uint32_t nTime) {
    TimingDelay = nTime;
    while(TimingDelay != 0);
}

void print2usb(char *s) {
    if (s == NULL) {
        return; // Handle null pointer
    }

    int str_len = strlen(s); // Get the length of the string
    CDC_Send_DATA((unsigned char*)s, str_len); // Send the string to USB
    Delay(10);
}

int main(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    uint8_t i, ch;
    uint16_t u16CRC_computed;
    uint8_t RECEIVED_DATA[30] = {0};
    uint8_t data_gathering_active_flag, character_counter;
    int8_t module_counter;
    uint8_t data_size, error_flag;
    uint8_t module_type;


    char *s;

    /************************************************************
    *   Set the Vector Table base adress at 0x8004000
    *************************************************************/
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x4000);

    // Set Systick to interrupt each 1 ms
    if(SysTick_Config(72000)) {
        // Capture error */
        while (1);
    }

    /************************************************************
    *   Init PB0 led
    *************************************************************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /*************************************************************
    *   Init USART3 peripheral
    *************************************************************/
    USART3_Init();
    Delay(500);
    USART3andUSB_Print("START");

    /*************************************************************
    *   Init USB peripheral
    *************************************************************/
    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();

    /************************************************************
    *   Infinite loop
    *************************************************************/
    error_flag = 0;
    module_type = 0;
    module_counter = 0;
    character_counter = 0;
    data_gathering_active_flag = 0;

    while (1) {
        // we have 2 modes:   1. Send ID (data_gathering_active_flag = 0)
        //                    2. Data gathering (data_gathering_active_flag = 1)
        // "if then else" is like this so when flag is active execute only "if" condition
        if (data_gathering_active_flag == 1) {
            // We execute this block each loop if flag is active
            // but only for 100 ms

            // Test if we have received data and store to data buffer
            if (USART3->SR & USART_FLAG_RXNE) {
                // Read current received character from USART buffer
                ch = USART3->DR & 0xff;
                // Store the char in the coresponding buffer, from 0..28 = 29 octeti
                if (character_counter < 29) RECEIVED_DATA[character_counter] = ch;
                character_counter++;
            }

            // If we have timeout condition deactivate this block and
            // testset error flag
            // Else test if we have another char received
            if (TimingDelay == 0) {
                data_gathering_active_flag = 0;
                character_counter = 0;
                error_flag = 0;

                // All analog values are represented by 2 octets, except PT100 on 4 octets
                // for 1 = digital, length of data is 10,
                //     4 octets DATA + 4 octets CRC
                //     16 values as 1 bit each
                // for 2 = 4-20mA, length of data is 22,
                //     16 octets DATA + 4 octets CRC
                //     8 values as 2 octets each
                // for 3 = 0-10V, length of data is 22,
                //     16 octets DATA + 4 octets CRC
                //     8 values as 2 octets each
                // for 4 = PT100RTD cu MAX31865, length of data is 22,
                //     16 octets DATA + 4 octets CRC
                //     4 values as 4 octets each
                // for 5 = CRS IMPULSE COUNTER
                //     4 octets DATA + 4 octets CRC
                //     1 value as 4 octets

                if (module_type > 1) data_size = 22;
                else data_size = 10;
                // CRS timming is making an exception
                if (module_type == 5) data_size = 10;

                // Reset CRC
                u16CRC_computed = 0xFFFF;

                // Compute CRC of data, set top at the beginning of stored CRC
                // Starts from 1 because at position 0 we have '/'   "  /;0000DEDA?  "
                for (i = 1; i < data_size-4; i++) {
                    u16CRC_computed = crc16_update(u16CRC_computed, RECEIVED_DATA[i]);
                }

                // Test if computed CRC = received CRC (byte by byte)
                // If we have an error set error flag
                if (TO_HEX(((u16CRC_computed & 0xF000) >> 12)) != RECEIVED_DATA[data_size-4]) error_flag = 1;
                if (TO_HEX(((u16CRC_computed & 0x0F00) >> 8))  != RECEIVED_DATA[data_size-3]) error_flag = 1;
                if (TO_HEX(((u16CRC_computed & 0x00F0) >> 4))  != RECEIVED_DATA[data_size-2]) error_flag = 1;
                if (TO_HEX(((u16CRC_computed & 0x000F)))       != RECEIVED_DATA[data_size-1]) error_flag = 1;

                // Set for each module an error flag in modbus from adress 90
                // 90..99 used to store each module comunication status
                // writeHoldingRegister(90 + module_counter, error_flag); // ==== DEACTIVATED

                // Even if we have CRC error set for any of the modules, we need to increment
                // coresponding register indexes, so we don't overide another module registers
                if (error_flag != 0) {

                    // ambient temperature reading
                    if (module_type == 0) {
                                            // ==== DEACTIVATED
                    }

                    // Module type 1 is digital and goes to coils in modbus stack and
                    // the rest of the modules go to holding registers
                    // so we must use different indexes
                    if (module_type == 1) {
                        print2usb("\n");
                    }

                    // Module type 2 = 4-20mA, 3 = 0-10V
                    if ((module_type == 2) || (module_type == 3)) {
                        print2usb("\n");
                    }

                    // Module type 4 = PT100RTD MAX31865
                    if (module_type == 4) {
                        print2usb("\n");
                    }

                    // Module type 5 = CRS IMPULSE COUNTER
                    if (module_type == 5) {
                        print2usb("\n");
                    }
                }

                // If CRC check is ok then proceed analizing data and storing to modbus stack
                if (error_flag == 0) {
                    // Blink to show modules respond OK
                    GPIOB->ODR ^= GPIO_Pin_0;
                    GPIOC->ODR ^= GPIO_Pin_13;   // for BluePill

                    // Ambient temperature reading
                    if (module_type == 0) {
                        // ==== DEACTIVATED
                    }

                    // Module type 1 is digital and goes to coils in modbus stack and
                    // the rest of the modules go to holding registers
                    // so we must use different indexes
                    if (module_type == 1) {
                        print2usb("==== DIGITAL MODULE OK\n");
                    }

                    // Module type 2 = 4-20mA, 3 = 0-10V
                    if ((module_type == 2) || (module_type == 3)) {
                        print2usb("==== 4-20mA MODULE OK\n");
                    }

                    // Module type 4 = PT100RTD MAX31865
                    if (module_type == 4) {
                        print2usb("==== PT100 MODULE OK\n");
                    }

                    // Module type 5 = CRS IMPULSE COUNTER
                    if (module_type == 5) {
                        print2usb("==== CRS COUNTER MODULE OK\n");
                    }
                }
            }
        } else {
            // Clear received data buffer
            for (i = 1; i < 30; i++)
                RECEIVED_DATA[i] = 0;

            // Module type 0 = ambient temperature,
            //             1 = digital, 2 = 4-20mA,
            //             3 = 0-10V, 4 = PT100RTD,
            //             5 = CRS IMPULSE COUNTER

            // Module type is used above to know what type is current active module
            switch (module_counter) {
                case 0:
                    USART3andUSB_Print((char *)"/IA0\\");
                    module_type = 2;
                    break;
                case 1:
                    USART3andUSB_Print((char *)"/IA1\\");
                    module_type = 2;
                    break;
                case 2:
                    USART3andUSB_Print((char *)"/IA2\\");
                    module_type = 2;
                    break;
                case 3:
                    USART3andUSB_Print((char *)"/IA3\\");
                    module_type = 2;
                    break;
                case 4:
                    USART3andUSB_Print((char *)"/IA4\\");
                    module_type = 2;
                    break;
                case 5:
                    USART3andUSB_Print((char *)"/IA5\\");
                    module_type = 2;
                    break;
                case 6:
                    USART3andUSB_Print((char *)"/IA6\\");
                    module_type = 2;
                    break;
                case 7:
                    USART3andUSB_Print((char *)"/IA7\\");
                    module_type = 2;
                    break;
                case 8:
                    USART3andUSB_Print((char *)"/IA8\\");
                    module_type = 2;
                    break;
                case 9:
                    USART3andUSB_Print((char *)"/IA9\\");
                    module_type = 2;
                    break;

                case 10:
                    USART3andUSB_Print((char *)"/DA0\\");
                    module_type = 1;
                    break;
                case 11:
                    USART3andUSB_Print((char *)"/DA1\\");
                    module_type = 1;
                    break;
                case 12:
                    USART3andUSB_Print((char *)"/DA2\\");
                    module_type = 1;
                    break;
                case 13:
                    USART3andUSB_Print((char *)"/DA3\\");
                    module_type = 1;
                    break;
                case 14:
                    USART3andUSB_Print((char *)"/DA4\\");
                    module_type = 1;
                    break;
                case 15:
                    USART3andUSB_Print((char *)"/DA5\\");
                    module_type = 1;
                    break;
                case 16:
                    USART3andUSB_Print((char *)"/DA6\\");
                    module_type = 1;
                    break;
                case 17:
                    USART3andUSB_Print((char *)"/DA7\\");
                    module_type = 1;
                    break;
                case 18:
                    USART3andUSB_Print((char *)"/DA8\\");
                    module_type = 1;
                    break;
                case 19:
                    USART3andUSB_Print((char *)"/DA9\\");
                    module_type = 1;
                    break;

                case 20:
                    USART3andUSB_Print((char *)"/TA0\\");
                    module_type = 4;
                    break;
                case 21:
                    USART3andUSB_Print((char *)"/TA1\\");
                    module_type = 4;
                    break;
                case 22:
                    USART3andUSB_Print((char *)"/TA2\\");
                    module_type = 4;
                    break;
                case 23:
                    USART3andUSB_Print((char *)"/TA3\\");
                    module_type = 4;
                    break;
                case 24:
                    USART3andUSB_Print((char *)"/TA4\\");
                    module_type = 4;
                    break;
                case 25:
                    USART3andUSB_Print((char *)"/TA5\\");
                    module_type = 4;
                    break;
                case 26:
                    USART3andUSB_Print((char *)"/TA6\\");
                    module_type = 4;
                    break;
                case 27:
                    USART3andUSB_Print((char *)"/TA7\\");
                    module_type = 4;
                    break;
                case 28:
                    USART3andUSB_Print((char *)"/TA8\\");
                    module_type = 4;
                    break;
                case 29:
                    USART3andUSB_Print((char *)"/TA9\\");
                    module_type = 4;
                    break;

                // CRS impulse counter address will always be 0
                case 30:
                    USART3andUSB_Print((char *)"/DC0\\");
                    module_type = 4;
                    break;

                default:
                    break;
            }

            if (module_counter > 29) {
                // Reset module counter used to start over
                // -1 because below it is incremented to zero
                module_counter = -1;
                // Display start over indicator
                print2usb("\n==================================\n");
            } else {
                // Activate data gathering mode
                data_gathering_active_flag = 1;
            }

            // Increment modules counter
            module_counter++;

            // Set timeout
            TimingDelay = 100;
        }
    }
}

void USART3_Putch(unsigned char ch) {
    USART_SendData( USART3, ch);
    // Wait until the end of transmision
    while( USART_GetFlagStatus( USART3, USART_FLAG_TC) == RESET){}
}


void USART3andUSB_Print(char s[]) {
    int i=0;

    //s = "==================================\n";
    //   /;0|0|0|0|0|0|0|0|0|0|0|0|0|0|0|0?\n";
    CDC_Send_DATA ((unsigned char*)s, 5);Delay(10);
    while( i < 64) {
        if( s[i] == '\0') break;
        USART3_Putch( s[i++]);
    }
}

void USART3_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    // Configure USART3 Tx (PB10) as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Configure USART3 Rx (PB11) as input floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // USART3 configuration
    USART_InitStructure.USART_BaudRate = 19200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART3, &USART_InitStructure);

    USART_Cmd(USART3, ENABLE);
}

/*
#define TO_HEX(i) (i <= 9 ? '0' + i : 'A' - 10 + i)

void USBSERIAL_Print_Hex(uint16_t x)
{
    uint8_t res[5];

    if (x <= 0xFFFF)
    {
        res[0] = TO_HEX(((x & 0xF000) >> 12));
        res[1] = TO_HEX(((x & 0x0F00) >> 8));
        res[2] = TO_HEX(((x & 0x00F0) >> 4));
        res[3] = TO_HEX((x & 0x000F));
        res[4] = '\n';
    }
    CDC_Send_DATA ((uint8_t *)res,5);
}
*/
