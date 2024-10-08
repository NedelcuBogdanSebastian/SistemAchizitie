
/*
 * SOFT PT100 RTD (T)
 * 
 * FUSES
 * --------
 * EFUSE = 0xff     // = 0x07
 * HFUSE = 0xde
 * LFUSE = 0xe2
 * 
 *  aceasta varianta este pentru modulul cu max31865 4 module
 */

#include <SPI.h>
#include <MAX31865.h>

#define ID   "TA0"

#define LEDON    PORTD |= (1<<PIND5)
#define LEDOFF   PORTD &= ~(1<<PIND5)

#define TO_HEX(i)   (i <= 9 ? '0' + i : 'A' - 10 + i)
#define HI(x)       TO_HEX(((x & 0xF0) >> 4));
#define LO(x)       TO_HEX((x & 0x0F));

uint16_t u16CRC;

String str;

#define RTD0_CS_PIN   A3 // A0
#define RTD1_CS_PIN   A2 // A1
#define RTD2_CS_PIN   A0 // A2
#define RTD3_CS_PIN   A1 // A3

uint8_t  configuration_control_bits;
uint16_t configuration_low_threshold;
uint16_t configuration_high_threshold;
  
/* Values read from the device. */
uint8_t  measured_configuration;
uint16_t measured_resistance;
uint16_t measured_high_threshold;
uint16_t measured_low_threshold;
uint8_t  measured_status;

void MAX31865_configure( bool v_bias, bool conversion_mode, bool one_shot,
                              bool three_wire, uint8_t fault_cycle, bool fault_clear,
                              bool filter_50hz, uint16_t low_threshold,
                              uint16_t high_threshold, uint8_t cs_pin )
{
  uint8_t control_bits = 0;

  /* Assemble the control bit mask. */
  control_bits |= ( v_bias ? 0x80 : 0 );
  control_bits |= ( conversion_mode ? 0x40 : 0 );
  control_bits |= ( one_shot ? 0x20 : 0 );
  control_bits |= ( three_wire ? 0x10 : 0 );
  control_bits |= fault_cycle & 0b00001100;
  control_bits |= ( fault_clear ? 0x02 : 0 );
  control_bits |= ( filter_50hz ? 0x01 : 0 );

  /* Store the control bits and the fault threshold limits for reconfiguration
     purposes. */
  configuration_control_bits   = control_bits;
  configuration_low_threshold  = low_threshold;
  configuration_high_threshold = high_threshold;
  
  // Perform an full reconfiguration
  MAX31865_reconfigure(cs_pin);
}

/**
 * Reconfigure the MAX31865 by writing the stored control bits and the stored fault
 * threshold values back to the chip.
 *
 * @param [in] full true to send also threshold configuration
 */ 
void MAX31865_reconfigure(uint8_t cs_pin)
{
  /* Perform an full reconfiguration */
  /* Write the threshold values. */
  uint16_t threshold ;

  digitalWrite( cs_pin, LOW );
  SPI.transfer( 0x83 );
  threshold = configuration_high_threshold ;
  SPI.transfer( ( threshold >> 8 ) & 0x00ff );
  SPI.transfer(   threshold        & 0x00ff );
  threshold = configuration_low_threshold ;
  SPI.transfer( ( threshold >> 8 ) & 0x00ff );
  SPI.transfer(   threshold        & 0x00ff );
  digitalWrite( cs_pin, HIGH );

  /* Write the configuration to the MAX31865. */
  digitalWrite( cs_pin, LOW );
  SPI.transfer( 0x80 );
  SPI.transfer( configuration_control_bits );
  digitalWrite( cs_pin, HIGH );
}

uint8_t MAX31865_read_all(uint8_t cs_pin)
{
  uint16_t combined_bytes;

  /* Start the read operation. */
  digitalWrite( cs_pin, LOW );
  /* Tell the MAX31865 that we want to read, starting at register 0. */
  SPI.transfer( 0x00 );

  /* Read the MAX31865 registers in the following order:
       Configuration
       RTD
       High Fault Threshold
       Low Fault Threshold
       Fault Status */

  measured_configuration = SPI.transfer( 0x00 );

  combined_bytes  = SPI.transfer( 0x00 ) << 8;
  combined_bytes |= SPI.transfer( 0x00 );
  measured_resistance = combined_bytes >> 1;

  combined_bytes  = SPI.transfer( 0x00 ) << 8;
  combined_bytes |= SPI.transfer( 0x00 );
  measured_high_threshold = combined_bytes ;

  combined_bytes  = SPI.transfer( 0x00 ) << 8;
  combined_bytes |= SPI.transfer( 0x00 );
  measured_low_threshold = combined_bytes ;

  measured_status = SPI.transfer( 0x00 );

  digitalWrite( cs_pin, HIGH );

  /* Reset the configuration if the measured resistance is
     zero or a fault occurred. */
  if(  measured_resistance == 0 || measured_status != 0  )
    MAX31865_reconfigure(cs_pin);

  return( measured_status );
}

uint16_t MAX31865_getTemperature(uint8_t cs_pin)
{
    uint8_t operation_state;
    uint16_t T;
    
    operation_state = MAX31865_read_all(cs_pin);  
    if (operation_state == 0)
        return (10.0 * (( -0.003908 + sqrt( 0.000015272464 - ( -0.000002348 * ( 1.0 - measured_resistance * 0.0001312255859375 ) ) ) ) / -0.000001174));        
    else
        return 0;
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
*/
uint16_t crc16_update(uint16_t crc, uint8_t a)
{
  int i;

  crc ^= a;
  for (i = 0; i < 8; ++i)
  {
    if (crc & 1)
      crc = (crc >> 1) ^ 0xA001;
    else
      crc = (crc >> 1);
  }

  return crc;
}

void setup() {  
  /* Initialize SPI communication. */
  SPI.begin( );
  SPI.setClockDivider( SPI_CLOCK_DIV4 );
  SPI.setDataMode( SPI_MODE1 );

  pinMode(A0, OUTPUT);        
  pinMode(A1, OUTPUT);        
  pinMode(A2, OUTPUT);        
  pinMode(A3, OUTPUT);  
   
  // initialize serial port
  Serial.begin(19200);

  // PD5 pin as LED output
  DDRD |= (1<<PIND5);
  LEDOFF;

  for (int i=0; i < 10; i++)
  {
    delay(30);    
    LEDON;
    delay(30);
    LEDOFF;
  } 
  
  /* Configure:

       V_BIAS enabled
       Auto-conversion
       1-shot disabled
       3-wire enabled
       Fault detection:  automatic delay
       Fault status:  auto-clear
       50 Hz filter
       Low threshold:   0x2690  // -100C  MAX31865 datasheet page 20/21 
       High threshold:  0x9304  // +350C  Table 9. Temperature Example for PT100 with 400Ω RREF
  */
  
    MAX31865_configure( true, true, false, true, MAX31865_FAULT_DETECTION_NONE,
                        true, true, 0x2690, 0x9304, RTD0_CS_PIN);                   
    MAX31865_configure( true, true, false, true, MAX31865_FAULT_DETECTION_NONE,
                        true, true, 0x2690, 0x9304, RTD1_CS_PIN);                   
    MAX31865_configure( true, true, false, true, MAX31865_FAULT_DETECTION_NONE,
                        true, true, 0x2690, 0x9304, RTD2_CS_PIN);                   
    MAX31865_configure( true, true, false, true, MAX31865_FAULT_DETECTION_NONE,
                        true, true, 0x2690, 0x9304, RTD3_CS_PIN);                    
}


void loop() 
{
  // if we receive one char then enter this block
  if (Serial.available())
  {    
    // if we have '/' char then proceed processing more incoming chars
    if (Serial.read() == '/')
    {      
      str = "";
      Serial.setTimeout(100);      
      str = Serial.readStringUntil('\\');
      
      unsigned long now = millis ();
      while (millis () - now < 50)
        if (Serial.available())
          Serial.read();  // read and discard any input        
        
      if (str == ID) send_inputs();      
    }
  }
}

void send_inputs(void)
{
  uint16_t T0, T1, T2, T3;
  uint8_t stat;
  char buf[30];  
  
    LEDON;
         
    T0 = MAX31865_getTemperature(RTD0_CS_PIN);
    T1 = MAX31865_getTemperature(RTD1_CS_PIN);
    T2 = MAX31865_getTemperature(RTD2_CS_PIN);
    T3 = MAX31865_getTemperature(RTD3_CS_PIN);
    
    
/*    T0 = 0;
    T1 = 0;
    T2 = 0;
    T3 = 0;
   
    stat = MAX31865_read_all(RTD0_CS_PIN);  
    if( stat == 0 )
        T0 = 10.0 * (( -0.003908 + sqrt( 0.000015272464 - ( -0.000002348 * ( 1.0 - measured_resistance * 0.0001312255859375 ) ) ) ) / -0.000001174);        
    stat = MAX31865_read_all(RTD1_CS_PIN);  
    if( stat == 0 )
        T1 = 10.0 * (( -0.003908 + sqrt( 0.000015272464 - ( -0.000002348 * ( 1.0 - measured_resistance * 0.0001312255859375 ) ) ) ) / -0.000001174);        
    stat = MAX31865_read_all(RTD2_CS_PIN);  
    if( stat == 0 )
        T2 = 10.0 * (( -0.003908 + sqrt( 0.000015272464 - ( -0.000002348 * ( 1.0 - measured_resistance * 0.0001312255859375 ) ) ) ) / -0.000001174);        
    stat = MAX31865_read_all(RTD3_CS_PIN);  
    if( stat == 0 )
        T3 = 10.0 * (( -0.003908 + sqrt( 0.000015272464 - ( -0.000002348 * ( 1.0 - measured_resistance * 0.0001312255859375 ) ) ) ) / -0.000001174);        
*/
    
    if (T0 > 1000) T0 = 0; // 100 grade max
    if (T1 > 1000) T1 = 0; // 100 grade max
    if (T2 > 1000) T2 = 0; // 100 grade max
    if (T3 > 1000) T3 = 0; // 100 grade max
    
    

  // module top side
  uint8_t i = 0;
  buf[i++] = ';';
  
  buf[i++] = TO_HEX(((T0 & 0xF000) >> 12));  
  buf[i++] = TO_HEX(((T0 & 0x0F00) >> 8));
  buf[i++] = TO_HEX(((T0 & 0x00F0) >> 4));  
  buf[i++] = TO_HEX((T0 & 0x000F));
  
  buf[i++] = TO_HEX(((T1 & 0xF000) >> 12));  
  buf[i++] = TO_HEX(((T1 & 0x0F00) >> 8));
  buf[i++] = TO_HEX(((T1 & 0x00F0) >> 4));  
  buf[i++] = TO_HEX((T1 & 0x000F));
  
  buf[i++] = TO_HEX(((T2 & 0xF000) >> 12));  
  buf[i++] = TO_HEX(((T2 & 0x0F00) >> 8));
  buf[i++] = TO_HEX(((T2 & 0x00F0) >> 4));  
  buf[i++] = TO_HEX((T2 & 0x000F));
  
  buf[i++] = TO_HEX(((T3 & 0xF000) >> 12));  
  buf[i++] = TO_HEX(((T3 & 0x0F00) >> 8));
  buf[i++] = TO_HEX(((T3 & 0x00F0) >> 4));  
  buf[i++] = TO_HEX((T3 & 0x000F));
  
  u16CRC = 0xFFFF;
  for (uint8_t j=0; j<i; j++)
    u16CRC = crc16_update(u16CRC, buf[j]);

  buf[i++] = TO_HEX(((u16CRC & 0xF000) >> 12));
  buf[i++] = TO_HEX(((u16CRC & 0x0F00) >> 8));
  buf[i++] = TO_HEX(((u16CRC & 0x00F0) >> 4));
  buf[i++] = TO_HEX((u16CRC & 0x000F));;

  buf[i++] = '?';
    
  buf[i++] = 0;   // string ending
  
  Serial.print((char*)buf);
 
  LEDOFF;
}
