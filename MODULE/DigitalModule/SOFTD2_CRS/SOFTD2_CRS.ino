/*
 * SOFT DIGITALE + IMPULSE TIME COUTER ON LAST INPUT 
 * 
 * FUSES
 * --------
 * EFUSE = 0xff     // = 0x07
 * HFUSE = 0xde
 * LFUSE = 0xe2
 * 
 */

#define PCINT_PIN 12    // PB4 - MOSI - PCINT4
#define PCINT_MODE CHANGE
#define PCINT_FUNCTION activeTimeCounter

#define PCMSK *digitalPinToPCMSK(PCINT_PIN)
#define PCINT digitalPinToPCMSKbit(PCINT_PIN)
#define PCIE  digitalPinToPCICRbit(PCINT_PIN)
#define PCPIN *portInputRegister(digitalPinToPort(PCINT_PIN))

#if (PCIE == 0)
#define PCINT_vect PCINT0_vect
#elif (PCIE == 1)
#define PCINT_vect PCINT1_vect
#elif (PCIE == 2)
#define PCINT_vect PCINT2_vect
#else
#error This board doesnt support PCINT ?
#endif

#define INPUT_PIN_LOW    !(PINB & (1<<PB4))
#define INPUT_PIN_HIGH    (PINB & (1<<PB4))

uint16_t SIGNAL_PERIOD = 0;
uint16_t LAST_SIGNAL_PERIOD = 0;
uint32_t TimeStamp = 0;
uint8_t flagInOut = 0;

volatile uint8_t oldPort = 0x00;
 
#define ID1   "DA6"
#define ID2   "DC6"

#define LEDON    PORTB |= (1<<PB5)
#define LEDOFF   PORTB &= ~(1<<PB5)

volatile uint16_t timeCounter = 0; // start from somewere
volatile uint8_t flagCounterStarted = 0;


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

  // ==== attachPinChangeInterrupt
  // update the old state to the actual state
  oldPort = PCPIN;
  // pin change mask registers decide which pins are enabled as triggers
  PCMSK |= (1 << PCINT);
  // PCICR: Pin Change Interrupt Control Register - enables interrupt vectors
  PCICR |= (1 << PCIE);
}

/*

 void impulseTimeCounter(void)
{
  // If impulse detected
  if (!(PINB & (1<<PB4)))
  {
    // If first time
    if (flagCounterStarted == 0)
    { 
      // Save time stamp
      timeStamp = millis();
      // Disable this block until impulse end
      flagCounterStarted = 1;
    }
    //LEDON; ==========================
  }
  else
  {
    // Calculate impulse duration
    timeCounter = millis() - timeStamp;
    // Reset flag for the next impulse
    flagCounterStarted = 0;
    //LEDOFF; ===================
  }
}
*/

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
        
      if (str == ID1) send_inputs1(); 
      if (str == ID2) send_inputs2();
    }
  }

  if (flagInOut == 1)
      if ((millis() - TimeStamp) > 10000)
      {
          flagInOut = 0;
          digitalWrite(LED_BUILTIN, LOW);
      }

}

/*
void activeTimeCounter(void) 
{
  // IF LOW
  if (INPUT_PIN_LOW) 
  {  
      TimeStamp = millis();
      digitalWrite(LED_BUILTIN, HIGH);
  }
  else 
  {
      SIGNAL_PERIOD = uint16_t(millis() - TimeStamp);
      digitalWrite(LED_BUILTIN, LOW);
  }
}
*/

void activeTimeCounter(void) 
{
    if (INPUT_PIN_LOW && (flagInOut == 0)) 
    {  
        flagInOut = 1;
        TimeStamp = millis();
        digitalWrite(LED_BUILTIN, HIGH);
    }

    if (INPUT_PIN_HIGH && (flagInOut == 1)) 
    {
        SIGNAL_PERIOD = uint16_t(millis() - TimeStamp);
        if (SIGNAL_PERIOD > 1000)
        {
            flagInOut = 0;
            digitalWrite(LED_BUILTIN, LOW);
        }
    }
}


ISR(PCINT_vect) 
{  
    PCINT_FUNCTION();
}

/*
ISR(PCINT_vect) {
  // get the new and old pin states for port
  uint8_t newPort = PCPIN;

  // compare with the old value to detect a rising or falling
  uint8_t change = newPort ^ oldPort;

  // check which pins are triggered, compared with the settings
  uint8_t trigger = 0x00;
#if (PCINT_MODE == RISING) || (PCINT_MODE == CHANGE)
  uint8_t rising = change & newPort;
  trigger |= (rising & (1 << PCINT));
#endif
#if (PCINT_MODE == FALLING) || (PCINT_MODE == CHANGE)
  uint8_t falling = change & oldPort;
  trigger |= (falling & (1 << PCINT));
#endif

  // save the new state for next comparison
  oldPort = newPort;

  // if our needed pin has changed, call the IRL interrupt function
  if (trigger & (1 << PCINT))
    PCINT_FUNCTION();
}

 */
uint8_t b2c(uint8_t nr)
{
  if (nr == 0)
      return 1;
  else
      return 0;
}

void send_inputs1(void)
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


void send_inputs2(void)
{
  char buf[30];  
  
  LEDON; 
  unsigned long now = millis();

  // update last value only if new data is reliable
  if (SIGNAL_PERIOD > 1000) LAST_SIGNAL_PERIOD = SIGNAL_PERIOD;

  uint8_t i = 0;
  buf[i++] = ';';
 
  buf[i++] = TO_HEX(((LAST_SIGNAL_PERIOD & 0xF000) >> 12));  
  buf[i++] = TO_HEX(((LAST_SIGNAL_PERIOD & 0x0F00) >> 8));
  buf[i++] = TO_HEX(((LAST_SIGNAL_PERIOD & 0x00F0) >> 4));  
  buf[i++] = TO_HEX((LAST_SIGNAL_PERIOD & 0x000F));
  
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


/*
#define HI   digitalWrite(LED_BUILTIN, HIGH)
#define LO   digitalWrite(LED_BUILTIN, LOW)

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    pinMode(A0, INPUT_PULLUP);
    pinMode(A1, INPUT_PULLUP);
    pinMode(A2, INPUT_PULLUP);
}

void loop() {
    if (digitalRead(A0) == LOW) {
    HI; delay(23);
    LO; delay(28);
    HI; delay(23);    
    LO; delay(28);
    HI; delay(2);
    LO; delay(1);
    HI; delay(5);
    LO; delay(1);
    HI; delay(2873); // + 111
    LO;
    }
    
    if (digitalRead(A1) == LOW) {
    HI; delay(2);
    LO; delay(2);
    HI; delay(3);    
    LO; delay(5);
    HI; delay(10);
    LO; delay(50);
    HI; delay(3);
    LO; delay(1);
    HI; delay(5638); // + 76
    LO;
    }
    
    if (digitalRead(A2) == LOW) {
    HI; delay(5);
    LO; delay(10);
    HI; delay(10);    
    LO; delay(5);
    HI; delay(20);
    LO; delay(10);
    HI; delay(5);
    LO; delay(15);
    HI; delay(3243); // + 80
    LO;
    }
}
*/
