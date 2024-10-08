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
    HEARTBEAT signal is on PB1 at 100 ms (down for 500 Us, then up).

    Connection with modules is over serial USART3 PB10(TX), PB11(RX).

    The DS18B20 digital thermometer is connected to pin PB5.

    The signalling LED is on pin PC13. (Usual on the BluePill).

    Is implemented the reading of CRS switching time from digital module (CRS signal counter)

    The CRC16 is implemented as a look-up table.

    To run a serial Modbus server, we use:
        diagslave.exe -m rtu -p none -a 8 COM14 (only to know how)

    If we need to store signed values we use 2's complement:
        int16_t val = -100;
        uint16_t number = (uint16_t) val
*/

#include "string.h"
#include "stm32f10x.h"
#include "main.h"
#include "mbutils.h"
#include "mb.h"

#define LED_ON       GPIOC->BRR = GPIO_Pin_13
#define LED_OFF      GPIOC->BSRR = GPIO_Pin_13
#define LED_TOGGLE   GPIOC->ODR ^= GPIO_Pin_13

// Lookup table for TO_DEC function for '0'-'9' and 'A'-'F'
uint8_t TO_DEC_LOOKUP[256];

void generateTODECTable (void) {
    // Initialize lookup for digits '0'-'9' (ASCII 48-57)
    for (char c = '0'; c <= '9'; c++) {
        TO_DEC_LOOKUP[(uint8_t)c] = c - '0';
    }

    // Initialize lookup for letters 'A'-'F' (ASCII 65-70)
    for (char c = 'A'; c <= 'F'; c++) {
        TO_DEC_LOOKUP[(uint8_t)c] = c - 'A' + 10;
    }

    // Optionally handle lowercase 'a'-'f' as well, if needed (ASCII 97-102)
    for (char c = 'a'; c <= 'f'; c++) {
        TO_DEC_LOOKUP[(uint8_t)c] = c - 'a' + 10;
    }
}
// Now, TO_DEC_LOOKUP['0'] = 0, TO_DEC_LOOKUP['A'] = 10

// dec -> hex, hex -> dec
#define TO_HEX(i) (i <= 9 ? '0' + i : 'A' - 10 + i)
#define TO_DEC(i) TO_DEC_LOOKUP[i] // (i <= '9'? i - '0': i - 'A' + 10)
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

volatile uint32_t TimingDelay;

void Delay (volatile uint32_t nTime) {
    TimingDelay = nTime;
    while(TimingDelay != 0);
}
void USART3_Putch (unsigned char ch);
void USART3_Print (char s[]);
void USART3_Print_Int (int number);
void USART3_Init (void);

volatile uint8_t Modbus_Request_Flag;

u16 usRegHoldingBuf[100+1]; // 0..99 holding registers
u8  usRegCoilBuf[64/8+1];   // 0..64  coils

void writeCoil (uint8_t coil_index, uint8_t state) {
    uint8_t coil_offset=coil_index/8;
    if (state == 1)
        usRegCoilBuf[coil_offset] |= (1<<(coil_index%8));
    else usRegCoilBuf[coil_offset] &= ~(1<<(coil_index%8));
}

uint8_t getCoil (uint8_t coil_index) {
    uint8_t coil_byte=usRegCoilBuf[coil_index/8];
    if (coil_byte & (1<<(coil_index%8))) return 1;
    else return 0;
}

void writeHoldingRegister (uint8_t reg_index, uint16_t reg_val) {
    usRegHoldingBuf[reg_index] = reg_val;
}

uint16_t readHoldingRegister (uint8_t reg_index) {
    return usRegHoldingBuf[reg_index];
}

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

uint16_t crc16_update(uint16_t crc, uint8_t a) {
    uint8_t i;

    crc ^= a;
    for (i = 0; i < 8; ++i) {
        if (crc & 1) crc = (crc >> 1) ^ 0xA001;
        else crc = (crc >> 1);
    }

    return crc;
}
 We use look-up table for speed
 */
#define POLY 0xA001  // Polynomial used in CRC-16

uint16_t crc16_table[256];

// Generate the CRC-16 lookup table
void generateCRC16Table (void) {
    uint16_t crc;
    for (int i = 0; i < 256; i++) {
        crc = i;
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ POLY;
            } else {
                crc = crc >> 1;
            }
        }
        crc16_table[i] = crc;
    }
}

uint16_t crc16_update (uint16_t crc, uint8_t data) {
    uint8_t tableIndex;
    // XOR the data byte with the lower byte of the CRC and find the table index
    tableIndex = (uint8_t)(crc ^ data);
    // Shift the CRC right and XOR with the table value for the current byte
    crc = (crc >> 8) ^ crc16_table[tableIndex];
    return crc;
}

// https://www.h-schmidt.net/FloatConverter/IEEE754.html
// from : http://www.chipkin.com/how-real-floating-point-and-32-bit-data-is-encoded-in-modbus-rtu-messages/
// result that the encoding scheme is:
// 2.i16-1.ifloat-sb	byte swap	[ a b ] [ c d ]	=> [ b a d c ]
// [ d c ] - first Modbus register
// [ b a ] - second Modbus register
void writeFloat2HoldingRegisters (uint8_t reg_index, float f) {
    uint8_t * v = (uint8_t *) &f;

    uint8_t a, b, c, d;
    uint16_t val16;

    a = v[0];
    b = v[1];
    c = v[2];
    d = v[3];

    // To change encoding scheme to
    // 2.i16-1.ifloat-sw	byte and word swap	[ a b ] [ c d ]	=> [ d c b a ]
    // just change as commented below

    val16 = (b & 0xFF);
    val16 = (val16 << 8) | (a & 0xFF);
    writeHoldingRegister(reg_index, val16); // reg_index + 1

    val16 = (d & 0xFF);
    val16 = (val16 << 8) | (c & 0xFF);
    writeHoldingRegister(reg_index+1, val16);  // reg_index
}

// Testing float values in the Modbus server
// writeFloat2HoldingRegisters(3, 0.4956);  // Store a float in registers 2 & 3
// float t = get_float(3);

float get_float (uint8_t reg_index) {
    float f = 0.0;
    // Creates a pointer v to the memory location of the variable f,
    // but it casts it to a uint8_t *, which means v will point to the
    // byte-level representation of the f float
    uint8_t * v = (uint8_t *) &f;
    uint8_t a, b, c, d;
    uint16_t val16;
    // Get the first byte from Modbus data space
    val16 = usRegHoldingBuf[reg_index];
    b = (val16 >> 8) & 0xFF;
    a = val16 & 0xFF;
    // Get the second byte
    val16 = usRegHoldingBuf[reg_index+1];
    d = (val16 >> 8) & 0xFF;
    c = val16 & 0xFF;

    v[0] = a;
    v[1] = b;
    v[2] = c;
    v[3] = d;

    return f;
}

// Initialise DWT counter
volatile uint32_t *DWT_CONTROL = (volatile uint32_t *)0xE0001000;
volatile uint32_t *DWT_CYCCNT  = (volatile uint32_t *)0xE0001004;
volatile uint32_t *SCB_DEMCR   = (volatile uint32_t *)0xE000EDFC;

void DWT_Enable (void) {
    // Enable the use of DWT
    *SCB_DEMCR = *SCB_DEMCR | 0x01000000;
    // Reset the counter
    *DWT_CYCCNT = 0;
    // Enable cycle counter
    *DWT_CONTROL = *DWT_CONTROL | 1;
}

/*#pragma GCC push_options
#pragma GCC optimize ("O3")
void DWT_Delay(uint32_t us) {
	volatile uint32_t delayTicks = (SystemCoreClock/1000000L)*us;
	volatile uint32_t startTick = DWT->CYCCNT;
	do  {
	} while(DWT->CYCCNT - startTick < delayTicks);  // Execute at least once
}
#pragma GCC pop_options
*/

void DWT_Delay (uint32_t us) {  // Microseconds
	uint32_t delayTicks = us * (SystemCoreClock / 1000000);
	uint32_t startTick = *DWT_CYCCNT;
    while (*DWT_CYCCNT - startTick < delayTicks);
}

uint8_t DS18B20_Init (void) {
	GPIOB->BRR = GPIO_Pin_5;   // Pull bus down for 500 Us (min. 480 Us)
	DWT_Delay(500);
	GPIOB->BSRR = GPIO_Pin_5;  // Release bus

    // Wait 70 Us, bus should be pulled up by resistor and then
    // pulled down by slave (15-60 Us after detecting rising edge)
	DWT_Delay(70);

	if (!(GPIOB->IDR & GPIO_Pin_5)) {  // If the pin is low i.e the presence pulse is there
		DWT_Delay(430);  // Wait additional 430 Us until slave keeps bus down (total 500 Us, min. 480 Us)
		return 0;
	} else {
        DWT_Delay(430);  // Wait additional 430 Us until slave keeps bus down (total 500 Us, min. 480 Us)
		return 1;
	}
}

void DS18B20_Write (uint8_t data) {
	for (int i=0; i<8; i++) {
		if ((data & (1<<i)) != 0) {   // If the bit is high write 1
			GPIOB->BRR = GPIO_Pin_5;  // Pull bus down for 15 Us
			DWT_Delay(15);
			GPIOB->BSRR = GPIO_Pin_5; // Release bus
			DWT_Delay(60);            // Wait until end of time slot (60 Us) + 5 Us for recovery
		} else {                      // If the bit is low write 0
			GPIOB->BRR = GPIO_Pin_5;  // Pull bus down for 60 Us
			DWT_Delay(60);
			GPIOB->BSRR = GPIO_Pin_5; // Release bus
			DWT_Delay(5);             // Wait until end of time slot (60 Us) + 5 Us for recovery
		}
	}
}

uint8_t DS18B20_Read (void) {
	uint8_t value=0;

	for (int i=0;i<8;i++) {
		GPIOB->BRR = GPIO_Pin_5;       // Pull bus down for 5 Us
		DWT_Delay(5);
		GPIOB->BSRR = GPIO_Pin_5;      // Release bus
		DWT_Delay(5);                  // Wait 5 Us and check bus state

		if (GPIOB->IDR & GPIO_Pin_5) { // If the pin is HIGH
			value |= 1<<i;             // Read = 1
		}
		DWT_Delay(55);                 // Wait until end of time slot (60 Us) + 5 Us for recovery
	}
	return value;
}

uint16_t DS18B20_GetTemperature (void) {
	uint8_t check=2, temp_l=0, temp_h=0;

    check = DS18B20_Init();
    if (check == 1) return 5555;   // If initialisation returned error then return 5555 as error code
    DS18B20_Write(0xCC);  // skip ROM
    DS18B20_Write(0x44);  // convert t
    DWT_Delay(800);

    DS18B20_Init();
    DS18B20_Write(0xCC);  // skip ROM
    DS18B20_Write(0xBE);  // Read Scratch pad

	temp_l = DS18B20_Read();
	temp_h = DS18B20_Read();
	return (temp_h<<8)|temp_l;
}

void HEARTBEAT(void) {
	GPIOB->BRR = GPIO_Pin_1;   // Pull heart beat pin down for 500 us
	DWT_Delay(500);
	GPIOB->BSRR = GPIO_Pin_1;  // Release heart beat pin
}

int main(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    uint8_t i, j, k, val, ch;
    uint16_t CRC16_val;
    uint16_t temperature;
    uint8_t RECEIVED_DATA[30] = {0};
    uint8_t data_gathering_active_flag, character_counter;
    int8_t module_counter;
    uint8_t data_size, error_flag;
    uint8_t co_index, hr_index;
    uint8_t module_type;
    uint16_t pt100_temperature;
    uint16_t signal_counter;

    // --->>> Vectors position was set in system_stm32f10x.c, line 128
    // Set the Vector Table base address at 0x8004000
    // NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x4000);

    if(SysTick_Config(72000)) { // 1 Ms interrupt 72MHz/72000 = 1000
        // Capture error
        while (1);
    }

    /************************************************************
    *   Enable the use of internal DWT for counting
    *************************************************************/
    DWT_Enable();

    /************************************************************
    *   Initialise led on PC13 (BluePill is used)
    *************************************************************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /************************************************************
    *   Initialise PB1 as heart beat bus for RESETTER module
    *************************************************************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    HEARTBEAT();

    /************************************************************
    *   Initialise PB5 as control pin for DS18B20
    *************************************************************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /************************************************************
    *   Initialise USART3 peripheral
    *************************************************************/
    USART3_Init();

    /************************************************************
    *   Generate CRC16 look-up table
    *************************************************************/
    generateCRC16Table();

    /************************************************************
    *   Generate TO_DEC look-up table
    *************************************************************/
    generateTODECTable();

    /************************************************************
    *   Initialise protocol stack in RTU mode
    *************************************************************/
	// MB_RTU, Device ID: 2, USART port: 1 (it's configured in portserial.h, Baud rate: 19200, Parity: NONE)
    eMBInit(MB_RTU, 2, 1, 19200, MB_PAR_NONE);
	// Enable the Modbus Protocol Stack.
    eMBEnable();

    /************************************************************
    *   Config IWDG watch dog
    *************************************************************/
    // The dedicated separate clock of the IWDG hardware comes from Low Speed Internal (LSI) clock.
    // The LSI is not accurate due to the fact that it is a RC oscillator. It has an oscillation frequency of somewhere between 30 – 60 kHz.
    // For most applications it is assumed to have a mean frequency of 45 kHz though it is supposed to be around 32 kHz.
    // Prescaler value can be all powers of two ranging from pow(2, 2) up to pow(2, 8)
    // Reload specifies the timer period, that is the auto-reload value when the timer is refreshed. It can range from 0x0 up to 0xFFF
    // Ex:
    //     If prescaler is set to IWDG_PRESCALER_4 and reload value is set to 4096 then timeout period is about 512 Ms.
    //     If prescaler is set to IWDG_PRESCALER_64 and reload value is set to 4096 then timeout period is about 8192 Ms approx. 8 Sec.

    // Enable write access to IWDG_PR and IWDG_RLR registers (disable write protection of IWDG registers)
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    // IWDG counter clock: 32KHz(LSI) / 32 = 1KHz
    IWDG_SetPrescaler(IWDG_Prescaler_32);
    // Set counter reload value to 500 IWDG timeout equal to 500ms (the timeout may varies due to LSI frequency dispersion)
    IWDG_SetReload(500);
    // Reload IWDG counter
    IWDG_ReloadCounter();
    // Enable IWDG (the LSI oscillator will be enabled by hardware)
    IWDG_Enable();

    co_index = 1;  // Coils start at zero in RMMS
    hr_index = 2;  // HR start at 1, because at 0 we have ambient temperature
    error_flag = 0;
    module_counter = 0;  // Start at first module index
    character_counter = 0;
    data_gathering_active_flag = 0;  // Start in `Send ID` mode

    while(1) {
    	// Reload IWDG counter
        IWDG_ReloadCounter();

        eMBPoll();

        // We have 2 modes:   1. Send ID        ( data_gathering_active_flag = 0 )
        //                    2. Data gathering ( data_gathering_active_flag = 1 )
        // "if then else" is like this so when flag is active execute only "if" condition
        if (data_gathering_active_flag == 1) {
            // We execute this block each loop if flag is active
            // but only for 100 Ms

            // Test if we have received data and store to data buffer
            if (USART3->SR & USART_FLAG_RXNE) {
                // Read current received character from USART buffer
                ch = USART3->DR & 0xff;
                // Store the char in the corresponding buffer, from 0..28 = 29 bytes
                if (character_counter < 29) RECEIVED_DATA[character_counter] = ch;
                character_counter++;
            }

            if (TimingDelay == 0) { // 100 Ms have past
            	HEARTBEAT();        // So send the heart beat

				// All analogue values are represented by 2 bytes, except PT100 on 4 bytes
				// for 1 = digital, length of data is 10,
				//     4 bytes DATA + 4 bytes CRC
				//     16 values as 1 bit each
				// for 2 = 4-20mA, length of data is 22,
				//     16 bytes DATA + 4 bytes CRC
				//     8 values as 2 bytes each
				// for 3 = 0-10V, length of data is 22,
				//     16 bytes DATA + 4 bytes CRC
				//     8 values as 2 bytes each
				// for 4 = PT100RTD with MAX31865, length of data is 22,
				//     16 bytes DATA + 4 bytes CRC
				//     4 values as 4 bytes each
				// for 5 = CRS IMPULSE COUNTER
				//     4 bytes DATA + 4 bytes CRC
				//     1 value as 4 bytes

                if (module_type > 1) data_size = 22;
				else data_size = 10;
                // CRS timing is making an exception
                if (module_type == 5) data_size = 10;

                data_gathering_active_flag = 0;
                character_counter = 0;
                error_flag = 0;

				// Reset CRC
				CRC16_val = 0xFFFF;

				// Compute CRC of data, set top at the beginning of stored CRC
				// Starts from 1 because at position 0 we have '/'   "  /;0000DEDA?  "
				for (i = 1; i < data_size-4; i++) {
					CRC16_val = crc16_update(CRC16_val, RECEIVED_DATA[i]);
				}

				// Test if computed CRC = received CRC (byte by byte)
				// If we have an error set error flag
				if (TO_HEX(((CRC16_val & 0xF000) >> 12)) != RECEIVED_DATA[data_size-4]) error_flag = 1;
				if (TO_HEX(((CRC16_val & 0x0F00) >> 8))  != RECEIVED_DATA[data_size-3]) error_flag = 1;
				if (TO_HEX(((CRC16_val & 0x00F0) >> 4))  != RECEIVED_DATA[data_size-2]) error_flag = 1;
				if (TO_HEX(((CRC16_val & 0x000F)))       != RECEIVED_DATA[data_size-1]) error_flag = 1;

				// Set for each module an error flag in Modbus from address 90
				// 90..99 used to store each module communication status
				writeHoldingRegister(90 + module_counter, error_flag);

				// Even if we have CRC error set for any of the modules, we need to increment
				// corresponding register indexes, so we don't override another module registers
				if (error_flag == 1) {
					// Module type 1 is digital and goes to coils in Modbus stack
					// the rest of the modules go to holding registers
					// so we must use different indexes
					if (module_type == 1) {
						// We have data at index 2, 3, 4, 5
						for (i = 2; i < 6; i++) {
							// We check bit's from 8 bit "val" at positions 3,2,1,0 (one hex character)
							for (j = 0; j < 4; j++) {
								// Write zero for this module in the Modbus server
								writeCoil(co_index, 0);
								// Only 0..63 coils allowed
								if (co_index < 63) co_index++;
							}
						}
					}

					// Module type 2 = 4-20mA, 3 = 0-10V
					if ((module_type == 2) || (module_type == 3)) {
						// We must get all the 8 values for all 8 inputs of the module, from the data received
						for (i = 0; i < 8; i++) {
							// Write zero for this module in the Modbus server
							writeHoldingRegister(hr_index, 0);
							// Only 0..89 holding registers allowed
							// 90..99 used to store each module communication status
							if (hr_index < 89) hr_index++;
						}
					}

					// Module type 4 = PT100RTD MAX31865
					if (module_type == 4) {
						// We must get all the 4 values for all 4 inputs of the module, from the data received
						for (i = 0; i < 4; i++) {
							// Write zero for this module in the Modbus server
							writeHoldingRegister(hr_index, 0);
							// Only 0..89 holding registers allowed
							// 90..99 used to store each module communication status
							if (hr_index < 89) hr_index++;
						}
					}

					// Module type 5 = CRS IMPULSE COUNTER
					if (module_type == 5) {
						// Write zero for this module in the Modbus server
						writeHoldingRegister(hr_index, 0);
						// Only 0..89 holding registers allowed
						// 90..99 used to store each module communication status
						if (hr_index < 89) hr_index++;
					}
				}

				// If CRC check is OK then proceed analysing data and storing to Modbus stack
				if (error_flag == 0) {
					// Module type 1 is digital and goes to coils in Modbus stack
					// the rest of the modules go to holding registers so we must use different indexes
					if (module_type == 1) {
						// We have data at index 2, 3, 4, 5
						for (i = 2; i < 6; i++) {
							// Convert hex value to uint8_t
							val = TO_DEC(RECEIVED_DATA[i]);
							// We check bit's from 8 bit "val" at positions 3,2,1,0 (one hex character)
							for (j = 0; j < 4; j++) {
								if (val & (1<<(3-j))) writeCoil(co_index, 1);
								else writeCoil(co_index, 0);
								// only 0..63 coils alowed
								if (co_index < 63) co_index++;
							}
						}
					}

					// Module type 2 = 4-20mA, 3 = 0-10V
					if ((module_type == 2) || (module_type == 3)) {
						k = 2;
						// We must get all the 8 values for all 8 inputs of the module, from the data received
						for (i = 0; i < 8; i++) {
							val = 0;
							// Convert 4 bit hex value to 8 bit unsigned int
							ch = TO_DEC(RECEIVED_DATA[k]);
							// Shift 4 to make space for new digit, and add the 4 bits of the new digit
							val = (val << 4) | (ch & 0xF);
							// Convert 4 bit hex value to 8 bit unsigned int
							ch = TO_DEC(RECEIVED_DATA[k+1]);
							// Shift 4 to make space for new digit, and add the 4 bits of the new digit
							val = (val << 4) | (ch & 0xF);
							// Follows next 2 chars
							k += 2;
							// Store data to modbus stack
							writeHoldingRegister(hr_index, val);
							// Only 0..89 holding registers alowed
							// 90..99 used to store each module comunication status
							if (hr_index < 89) hr_index++;
						}
					}

					// Module type 4 = PT100RTD MAX31865
					if (module_type == 4) {
						pt100_temperature = 0;
						ch = TO_DEC(RECEIVED_DATA[2]);
						pt100_temperature = (pt100_temperature << 4) | (ch & 0xF);
						ch = TO_DEC(RECEIVED_DATA[3]);
						pt100_temperature = (pt100_temperature << 4) | (ch & 0xF);
						ch = TO_DEC(RECEIVED_DATA[4]);
						pt100_temperature = (pt100_temperature << 4) | (ch & 0xF);
						ch = TO_DEC(RECEIVED_DATA[5]);
						pt100_temperature = (pt100_temperature << 4) | (ch & 0xF);
						writeHoldingRegister(hr_index, pt100_temperature);
						// Only 0..89 holding registers alowed
						// 90..99 used to store each module comunication status
						if (hr_index < 89) hr_index++;

						pt100_temperature = 0;
						ch = TO_DEC(RECEIVED_DATA[6]);
						pt100_temperature = (pt100_temperature << 4) | (ch & 0xF);
						ch = TO_DEC(RECEIVED_DATA[7]);
						pt100_temperature = (pt100_temperature << 4) | (ch & 0xF);
						ch = TO_DEC(RECEIVED_DATA[8]);
						pt100_temperature = (pt100_temperature << 4) | (ch & 0xF);
						ch = TO_DEC(RECEIVED_DATA[9]);
						pt100_temperature = (pt100_temperature << 4) | (ch & 0xF);
						writeHoldingRegister(hr_index, pt100_temperature);
						// Only 0..89 holding registers alowed
						// 90..99 used to store each module comunication status
						if (hr_index < 89) hr_index++;

						pt100_temperature = 0;
						ch = TO_DEC(RECEIVED_DATA[10]);
						pt100_temperature = (pt100_temperature << 4) | (ch & 0xF);
						ch = TO_DEC(RECEIVED_DATA[11]);
						pt100_temperature = (pt100_temperature << 4) | (ch & 0xF);
						ch = TO_DEC(RECEIVED_DATA[12]);
						pt100_temperature = (pt100_temperature << 4) | (ch & 0xF);
						ch = TO_DEC(RECEIVED_DATA[13]);
						pt100_temperature = (pt100_temperature << 4) | (ch & 0xF);
						writeHoldingRegister(hr_index, pt100_temperature);
						// Only 0..89 holding registers alowed
						// 90..99 used to store each module comunication status
						if (hr_index < 89) hr_index++;

						pt100_temperature = 0;
						ch = TO_DEC(RECEIVED_DATA[14]);
						pt100_temperature = (pt100_temperature << 4) | (ch & 0xF);
						ch = TO_DEC(RECEIVED_DATA[15]);
						pt100_temperature = (pt100_temperature << 4) | (ch & 0xF);
						ch = TO_DEC(RECEIVED_DATA[16]);
						pt100_temperature = (pt100_temperature << 4) | (ch & 0xF);
						ch = TO_DEC(RECEIVED_DATA[17]);
						pt100_temperature = (pt100_temperature << 4) | (ch & 0xF);
						writeHoldingRegister(hr_index, pt100_temperature);
						// Only 0..89 holding registers alowed
						// 90..99 used to store each module comunication status
						if (hr_index < 89) hr_index++;
					}

					// Module type 5 = CRS SIGNAL COUNTER
					if (module_type == 5) {
						signal_counter = 0;
						ch = TO_DEC(RECEIVED_DATA[2]);
						signal_counter = (signal_counter << 4) | (ch & 0xF);
						ch = TO_DEC(RECEIVED_DATA[3]);
						signal_counter = (signal_counter << 4) | (ch & 0xF);
						ch = TO_DEC(RECEIVED_DATA[4]);
						signal_counter = (signal_counter << 4) | (ch & 0xF);
						ch = TO_DEC(RECEIVED_DATA[5]);
						signal_counter = (signal_counter << 4) | (ch & 0xF);
						writeHoldingRegister(hr_index, signal_counter);
						// Only 0..89 holding registers alowed
						// 90..99 used to store each module comunication status
						if (hr_index < 89) hr_index++;
					}
				}
            }
        } else {
        	LED_TOGGLE;

            // Clear received data buffer
            memset(RECEIVED_DATA, 0, 30);

            // Call the modules id (random modules at home :))
            switch (module_counter) {
                case 1:
                    USART3_Print((char *)"/IA8\\");
                    module_type = 2;
                    break;
                case 2:
                    USART3_Print((char *)"/DA8\\");
                    module_type = 1;
                    break;
                case 3:
                    USART3_Print((char *)"/TA6\\");
                    module_type = 4;
                    break;
                case 4:
                    USART3_Print((char *)"/TA1\\");
                    module_type = 4;
                    break;
                default:
                    break;
            }

            // Module number 5 is where we read the ambiant temperature sensor and reset counters to restart
            if (module_counter == 5) {
                // reset module counter used to start over
                module_counter = 0;
                // Also because we start again, we must reset the indexes
                // hr_index = 1 is ambient temperature
                // used to store data in the modbus stack
                co_index = 1; hr_index = 2;

            	// Temperature reading is hard coded, address is fixed 1
            	temperature = DS18B20_GetTemperature();

            	if (temperature == 5555) {
                	// Write zero for this module in the modbus server
                    writeHoldingRegister(1, 0);  // hr_index = 1
            	} else {
                    // Store data to modbus stack
            		writeHoldingRegister(1, temperature);  // hr_index = 1
            	}
            	// To get real value we need to divide => temperature / 16.0
            } else {
                // Activate data gathering mode
                data_gathering_active_flag = 1;
            }

            // Increment modules counter
            module_counter++;

            // Reset timeout
            TimingDelay = 100;
        }
    }
}

void USART3_Putch (unsigned char ch) {
	USART_SendData( USART3, ch);
	// Wait until the end of transmission
	while( USART_GetFlagStatus( USART3, USART_FLAG_TC) == RESET){}
}

void USART3_Print (char s[]) {
    int i=0;

    while( i < 64) {
	    if( s[i] == '\0') break;
        USART3_Putch( s[i++]);
    }
}

void USART3_Print_Int (int number) {
	unsigned char s[5], i=1, j=0;

    if( number < 0) {
    	USART3_Putch( '-');
		number = -number;
	}

    while( number >= 10) {
	    s[i++] = number % 10;
	    number /= 10;
    }

    s[i] = number;
    j = i;
    for( i=j; i>0; i--) USART3_Putch( '0' + s[i]);
}

void USART3_Init (void) {
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
