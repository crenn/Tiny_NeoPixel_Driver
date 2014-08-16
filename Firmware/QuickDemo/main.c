#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//Timer 0 used for UART handling

//Timer 1 used for timing and debouncing
#define TIM1PEROID 5 //ms
#define TURNOFFAFTER	(1000/TIM1PEROID) //ms
#define STAYONAFTER		(500/TIM1PEROID) //ms
#define MODESWAFTER		(50/TIM1PEROID) //ms

#define NEOPIX_NUM  8
#define NEOPIX_BYTE  (NEOPIX_NUM*3)

const uint16_t numBytes = NEOPIX_BYTE;
volatile uint8_t hi = 0; // PORT w/output bit set high
volatile uint8_t lo = 0; // PORT w/output bit set low
volatile uint16_t time = TURNOFFAFTER;
volatile uint8_t state = 0;
uint8_t pixels[NEOPIX_BYTE] = {0x00};

uint8_t colour[3] = {0x00};

void ioinit(void);
void setupNeopixels(void);
void writeNeoPixel(uint8_t r, uint8_t g, uint8_t bl);
void writeNeoPixelByte(void);
void showLEDs(void);
void colourWipe(uint8_t r, uint8_t g, uint8_t bl);
void rainbow(void);
void Wheel(uint8_t WheelPos);

ISR(TIMER1_COMPA_vect) {
	uint8_t temp = PINB;
	temp &= 0x08;
	if (temp != 0) {
		time = TURNOFFAFTER;
	} else {
		if (time > 0) {
			if (--time == 0) {
				state = 1;
			}
		}
	}
	TIFR &= ~(0x40);
}

int main (void) {
  uint8_t i = 0;
  ioinit();
  
  setupNeopixels();
  
  do {
      writeNeoPixelByte();
  } while (++i < NEOPIX_NUM);
  i = 0;
  // for (i = 0; i < NEOPIX_NUM; i++) {
	// writeNeoPixelByte();
  // }
  // i = 0;

  while (1) {
    while (1) {
      if (i != 0) {
        pixels[i-1] = 0x00;
      } else {
        pixels[NEOPIX_BYTE-1] = 0x00;
      }
      pixels[i++] = 0xFF;
      if (i >= NEOPIX_BYTE) {
        i = 0;
      }
      _delay_ms(500);
      showLEDs();
      if (i == 0)
        break;
    }
    do {
      rainbow();
    } while (++i < 3);
    i = 0;
	do {
      colourWipe(0xFF,0x00,0x00);
	  colourWipe(0x00,0xFF,0x00);
	  colourWipe(0x00,0x00,0xFF);
    } while (++i < 3);
    i = 0;
	
	if (state == 1)
		break;
  }
  
  i = 0;
  colour[0] = 0;
  colour[1] = 0;
  colour[2] = 0;
  do {
      writeNeoPixelByte();
  } while (++i < NEOPIX_NUM);
  
  TCCR1 = 0x00;
  PORTB &= ~(0x08);
  DDRB |= 0x08;

  while(1); // switch to sleep here

  return 0;
}

void ioinit(void) {
  DDRB = 0x06;
  _delay_ms(50);
  PORTB = 0x19;
  OCR1A = 156;
  TIMSK = (1 << OCIE1A);
  TCCR1 = 0x8A;
  sei();
}

void setupNeopixels(void) {
  hi = PORTB | 0x02;
  lo = PORTB & ~0x02;
}

void showLEDs(void) {
  uint8_t currNeo = 0;
  uint8_t temp = 0;
  cli();
  while (currNeo < NEOPIX_BYTE) {
    colour[0] = pixels[currNeo++];
    colour[1] = pixels[currNeo++];
    colour[2] = pixels[currNeo++];
    writeNeoPixelByte();
  }
  sei();
}

void colourWipe(uint8_t r, uint8_t g, uint8_t bl) {
  uint8_t i, j;
  colour[0] = g;
  colour[1] = r;
  colour[2] = bl;
  for (i = 1; i <= NEOPIX_NUM; i++) {
    cli();
    for (j=0; j < i; j++) {
		writeNeoPixelByte();
	}
	sei();
	_delay_ms(50);
  }
}

void rainbow(void) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
  Wheel(j);
  cli();
  for (i=0; i < NEOPIX_NUM; i++) {
    writeNeoPixelByte();
  }
  sei();
    _delay_ms(50);
  }
}

void Wheel(uint8_t WheelPos) {
  uint8_t temp;
  if(WheelPos < 85) {
    temp = WheelPos * 3;
    colour[0] = 255 - temp;
    colour[1] = temp;
    colour[2] = 0;
  } else if(WheelPos < 170) {
    WheelPos -= 85;
    temp = WheelPos * 3;
    colour[0] = 0;
    colour[1] = 255 - temp;
    colour[2] = temp;
  } else {
    WheelPos -= 170;
    temp = WheelPos * 3;
    colour[0] = temp;
    colour[1] = 0;
    colour[2] = 255 - temp;
  }
}

void writeNeoPixel(uint8_t r, uint8_t g, uint8_t bl) {
  colour[0] = g;
  colour[1] = r;
  colour[2] = bl;
  writeNeoPixelByte();
}

// Assembly is derived from Adafruit's NeoPixel Libraries available here:
// https://github.com/adafruit/Adafruit_NeoPixel
void writeNeoPixelByte(void) {
  volatile uint16_t
    i   = 3; // Loop counter
  volatile uint8_t
   *ptr = colour,   // Pointer to next byte
    b   = *ptr++;   // Current byte value
  volatile uint8_t
    next = lo,
    bit  = 8;

    asm volatile(
     "head20:"                   "\n\t" // clk  pseudocode    (t =  0)
      "out  %[port] , %[hi]"     "\n\t" // 1    port = hi     (t =  1)
      "nop"                      "\n\t" // 1    nop           (t =  2)
      "sbrc %[byte],  7"         "\n\t" // 1-2  if(b & 128)
       "mov  %[next], %[hi]"     "\n\t" // 0-1   next = hi    (t =  4)
      "dec  %[bit]"              "\n\t" // 1    bit--         (t =  5)
      "out  %[port] , %[next]"   "\n\t" // 1    port = next   (t =  6)
      "nop"                      "\n\t" // 1    nop           (t =  7)
      "mov  %[next] ,  %[lo]"    "\n\t" // 1    next = lo     (t =  8)
      "breq nextbyte20"          "\n\t" // 1-2  if(bit == 0) (from dec above)
      "rol  %[byte]"             "\n\t" // 1    b <<= 1       (t = 10)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (t = 12)
      "nop"                      "\n\t" // 1    nop           (t = 13)
      "out  %[port] , %[lo]"     "\n\t" // 1    port = lo     (t = 14)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (t = 16)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (t = 18)
      "rjmp head20"              "\n\t" // 2    -> head20 (next bit out)
     "nextbyte20:"               "\n\t" //                    (t = 10)
      "ldi  %[bit]  ,  8"        "\n\t" // 1    bit = 8       (t = 11)
      "ld   %[byte] ,  %a[ptr]+" "\n\t" // 2    b = *ptr++    (t = 13)
      "out  %[port] , %[lo]"     "\n\t" // 1    port = lo     (t = 14)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (t = 16)
      "sbiw %[count], 1"         "\n\t" // 2    i--           (t = 18)
       "brne head20"             "\n"   // 2    if(i != 0) -> (next byte)
      : [byte]  "+r" (b),
        [bit]   "+r" (bit),
        [next]  "+r" (next),
        [count] "+w" (i)
      : [port] "I" (_SFR_IO_ADDR(PORTB)),
        [ptr]    "e" (ptr),
        [hi]     "r" (hi),
        [lo]     "r" (lo));
}