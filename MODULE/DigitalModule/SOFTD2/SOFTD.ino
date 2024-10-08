

/*
 * SOFT DIGITALE
 * 
 * FUSES
 * --------
 * EFUSE = 0xff     // = 0x07
 * HFUSE = 0xde
 * LFUSE = 0xe2
 * 
 */
 
#define ID   "DA9"

#define LEDON    PORTB |= (1<<PB5)
#define LEDOFF   PORTB &= ~(1<<PB5)

#define TO_HEX(i)   (i <= 9 ? '0' + i : 'A' - 10 + i)

uint16_t u16CRC;

String str;

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
  // initialize serial port
  Serial.begin(19200);

  // led pin as output
  DDRB |= (1<<PB5);
  LEDOFF;

  // activate pull-ups 
  PORTB |= (1<<PB7)|(1<<PB6)|(1<<PB4)|(1<<PB3)|(1<<PB2)|(1<<PB1)|(1<<PB0);  
  PORTC |= (1<<PC5)|(1<<PC4)|(1<<PC3)|(1<<PC2)|(1<<PC1)|(1<<PC0);
  PORTD |= (1<<PD7)|(1<<PD6)|(1<<PD5);   

  // initialize pins as input
  DDRB &= ~((1<<PB7)&(1<<PB6)&(1<<PB4)&(1<<PB3)&(1<<PB2)&(1<<PB1)&(1<<PB0));  
  DDRC &= ~((1<<PC5)&(1<<PC4)&(1<<PC3)&(1<<PC2)&(1<<PC1)&(1<<PC0));
  DDRD &= ~((1<<PD7)&(1<<PD6)&(1<<PD5));
   
  for (int i=0; i < 10; i++)
  {
    delay(30);    
    LEDON;
    delay(30);
    LEDOFF;
  }            
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

uint8_t b2c(uint8_t nr)
{
  if (nr == 0)
      return 1;
  else
      return 0;
}

void send_inputs(void)
{
  uint8_t octet, a, b, c, d;
  char buf[30];  
  
  LEDON;

  unsigned long now = millis();

  uint8_t i = 0;
  buf[i++] = ';';
 
  // top side
  
  a = b2c(PINC & (1<<PC5));
  b = b2c(PINC & (1<<PC4));
  c = b2c(PINC & (1<<PC3));
  d = b2c(PINC & (1<<PC2));
  octet = (a<<3)|(b<<2)|(c<<1)|(d<<0);  
  buf[i++] = TO_HEX(octet);

  a = b2c(PINC & (1<<PC1));
  b = b2c(PINC & (1<<PC0));
  c = b2c(PINB & (1<<PB6));
  d = b2c(PINB & (1<<PB7));
  octet = (a<<3)|(b<<2)|(c<<1)|(d<<0);  
  buf[i++] = TO_HEX(octet);

  // bottom side

  a =  b2c(PIND & (1<<PD5));
  b =  b2c(PIND & (1<<PD6));
  c =  b2c(PIND & (1<<PD7));
  d =  b2c(PINB & (1<<PB0));
  octet = (a<<3)|(b<<2)|(c<<1)|(d<<0);  
  buf[i++] = TO_HEX(octet);
  
  a = b2c(PINB & (1<<PB1));
  b = b2c(PINB & (1<<PB2));
  c = b2c(PINB & (1<<PB3));
  d = b2c(PINB & (1<<PB4));
  octet = (a<<3)|(b<<2)|(c<<1)|(d<<0);  
  buf[i++] = TO_HEX(octet);

  u16CRC = 0xFFFF;
  for (uint8_t j=0; j<i; j++)
    u16CRC = crc16_update(u16CRC, buf[j]);

  buf[i++] = TO_HEX(((u16CRC & 0xF000) >> 12));
  buf[i++] = TO_HEX(((u16CRC & 0x0F00) >> 8));
  buf[i++] = TO_HEX(((u16CRC & 0x00F0) >> 4));
  buf[i++] = TO_HEX( (u16CRC & 0x000F));

  buf[i++] = '?';
    
  buf[i++] = 0;   // string ending
  
  Serial.print((char*)buf);

  // wait at least 12 ms
  while (millis() - now < 12);
  
  LEDOFF;   
}

