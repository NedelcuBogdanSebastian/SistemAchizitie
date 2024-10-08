
/*
 * SOFT 4-20mA (I)
 * 
 * FUSES
 * --------
 * EFUSE = 0xff     // = 0x07
 * HFUSE = 0xde
 * LFUSE = 0xe2
 * 
 */
 
#define ID   "IA9"

#define LEDON    PORTB |= (1<<PB5)
#define LEDOFF   PORTB &= ~(1<<PB5)

#define TO_HEX(i)   (i <= 9 ? '0' + i : 'A' - 10 + i)
#define HI(x)       TO_HEX(((x & 0xF0) >> 4));
#define LO(x)       TO_HEX((x & 0x0F));

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

void send_inputs(void)
{
  uint16_t I0, I1, I2, I3, I4, I5, I6, I7;
  char buf[30];  
  
  LEDON;
  
  // A5, A4, A3, A2
  // A6, A7, A0, A1
  I0 = compute_current(A5); // PC5
  I1 = compute_current(A4); // PC4
  I2 = compute_current(A3); // PC3
  I3 = compute_current(A2); // PC2

  I4 = compute_current(A6); // ADC6
  I5 = compute_current(A7); // ADC7
  I6 = compute_current(A0); // PC0
  I7 = compute_current(A1); // PC1

  // module top side
  uint8_t i = 0;
  buf[i++] = ';';
  buf[i++] = HI(I0);  buf[i++] = LO(I0)
  buf[i++] = HI(I1);  buf[i++] = LO(I1)
  buf[i++] = HI(I2);  buf[i++] = LO(I2)
  buf[i++] = HI(I3);  buf[i++] = LO(I3)

  // module bottom side
  buf[i++] = HI(I4);  buf[i++] = LO(I4)
  buf[i++] = HI(I5);  buf[i++] = LO(I5)
  buf[i++] = HI(I6);  buf[i++] = LO(I6)
  buf[i++] = HI(I7);  buf[i++] = LO(I7)

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

uint16_t compute_current(int channel)
{
  float current=0;
  uint16_t I;
  
  for (int i=0; i<10; i++) 
    current += analogRead(channel) * 0.26881720430107526881720430107527; // (adc_value * (3.3 / 1023.0)) / 120.0
    
  I = uint16_t(current/10.0);

  // make number represent on 8 bits => 00..FF (0mA..25.5mA <=> 00..FF)  (value is *10 !!!)
  if (I > 255) I = 255;
  return I;
}


