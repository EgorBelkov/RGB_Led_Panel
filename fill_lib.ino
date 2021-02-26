// библиотека для работы с матрицей
//#include <RGBmatrixPanel.h>

// управляющие пины матрицы
#define CLK   11
#define OE    9
#define LAT   10
#define A     A0
#define B     A1
#define C     A2
#define D     A3

// объявляем объект для работы с матрицей 64х32
// включаем двойную буферизацию
//RGBmatrixPanel matrix(A, B, C, D, CLK, LAT, OE, true, 64);
#define nPlanes 4  
#define DATAPORT PORTA ///< RGB data PORT register
#define DATADIR DDRA   ///< RGB data direction register
#define CLKPORT PORTB  ///< RGB clock PORT register
#define WIDTH 64
#define HEIGHT 32
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  } ///< 16-bit var swap
  
  typedef uint8_t PortType;

  int color;
  uint8_t *matrixbuff[2];     ///< Buffer pointers for double-buffering
  uint8_t nRows;              ///< Number of rows (derived from A/B/C/D pins)
  volatile uint8_t backindex; ///< Index (0-1) of back buffer
  volatile boolean swapflag;  ///< if true, swap on next vsync
  uint8_t _clk;       ///< RGB clock pin number
  uint8_t _lat;       ///< RGB latch pin number
  uint8_t _oe;        ///< Output enable pin number
  uint8_t _a;         ///< Address/row-select A pin number
  uint8_t _b;         ///< Address/row-select B pin number
  uint8_t _c;         ///< Address/row-select C pin number
  uint8_t _d;         ///< Address/row-select D pin number
  PortType clkmask;   ///< RGB clock pin bitmask
  PortType latmask;   ///< RGB latch pin bitmask
  PortType oemask;    ///< Output enable pin bitmask
  PortType addramask; ///< Address/row-select A pin bitmask
  PortType addrbmask; ///< Address/row-select B pin bitmask
  PortType addrcmask; ///< Address/row-select C pin bitmask
  PortType addrdmask; ///< Address/row-select D pin bitmask
  // PORT register pointers (CLKPORT is hardcoded on AVR)
  volatile PortType *latport;   ///< RGB latch PORT register
  volatile PortType *oeport;    ///< Output enable PORT register
  volatile PortType *addraport; ///< Address/row-select A PORT register
  volatile PortType *addrbport; ///< Address/row-select B PORT register
  volatile PortType *addrcport; ///< Address/row-select C PORT register
  volatile PortType *addrdport; ///< Address/row-select D PORT register
  volatile uint8_t row;      ///< Row counter for interrupt handler
  volatile uint8_t plane;    ///< Bitplane counter for interrupt handler
  volatile uint8_t *buffptr; ///< Current RGB pointer for interrupt handler  

  uint8_t *buffer; 

void init(uint8_t rows, uint8_t a, uint8_t b, uint8_t c, uint8_t clk, uint8_t lat, uint8_t oe, boolean dbuf, uint8_t width) 
{

  nRows = rows; // Number of multiplexed rows; actual height is 2X this

  // Allocate and initialize matrix buffer:
  int buffsize = width * nRows * 3, // x3 = 3 bytes holds 4 planes "packed"
      allocsize = (dbuf == true) ? (buffsize * 2) : buffsize;
  if (NULL == (matrixbuff[0] = (uint8_t *)malloc(allocsize)))
    return;
  memset(matrixbuff[0], 0, allocsize);
  // If not double-buffered, both buffers then point to the same address:
  matrixbuff[1] = (dbuf == true) ? &matrixbuff[0][buffsize] : matrixbuff[0];

  // Save pin numbers for use by begin() method later.
  _a = a;
  _b = b;
  _c = c;
  _clk = clk;
  _lat = lat;
  _oe = oe;

  // Look up port registers and pin masks ahead of time,
  // avoids many slow digitalWrite() calls later.
  clkmask = digitalPinToBitMask(clk);
  latport = portOutputRegister(digitalPinToPort(lat));
  latmask = digitalPinToBitMask(lat);
  oeport = portOutputRegister(digitalPinToPort(oe));
  oemask = digitalPinToBitMask(oe);
  addraport = portOutputRegister(digitalPinToPort(a));
  addramask = digitalPinToBitMask(a);
  addrbport = portOutputRegister(digitalPinToPort(b));
  addrbmask = digitalPinToBitMask(b);
  addrcport = portOutputRegister(digitalPinToPort(c));
  addrcmask = digitalPinToBitMask(c);
  plane = nPlanes - 1;
  row = nRows - 1;
  swapflag = false;
  backindex = 0; // Array index of back buffer
}

void setup() 
{
  init(16, A, B, C, CLK, LAT, OE, true, 64);
  // Init a few extra 32x32-specific elements:
  _d = D;
  addrdport = portOutputRegister(digitalPinToPort(D));
  addrdmask = digitalPinToBitMask(D);  // инициируем работу с матрицей

  backindex = 0;                       // Back buffer
  buffptr = matrixbuff[1 - backindex]; // -> front buffer
  //activePanel = this;                  // For interrupt hander

  // Enable all comm & address pins as outputs, set default states:
  pinMode(_clk, OUTPUT);
  digitalWrite(_clk, LOW); // Low
  delayMicroseconds(1);
  pinMode(_lat, OUTPUT);
  *latport &= ~latmask; // Low
  delayMicroseconds(1);
  pinMode(_oe, OUTPUT);
  *oeport |= oemask; // High (disable output)
  delayMicroseconds(1);
  pinMode(_a, OUTPUT);
  *addraport &= ~addramask; // Low
  delayMicroseconds(1);
  pinMode(_b, OUTPUT);
  *addrbport &= ~addrbmask; // Low
  delayMicroseconds(1);
  pinMode(_c, OUTPUT);
  *addrcport &= ~addrcmask; // Low
  delayMicroseconds(1);
  if (nRows > 8) {
    pinMode(_d, OUTPUT);
    *addrdport &= ~addrdmask; // Low
  delayMicroseconds(1);
  }

  // The high six bits of the data port are set as outputs;
  // Might make this configurable in the future, but not yet.
  DATADIR = B11111100;
  DATAPORT = 0;
  // Set up Timer1 for interrupt:
  TCCR1A = _BV(WGM11);                          // Mode 14 (fast PWM), OC1A off
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // Mode 14, no prescale
  ICR1 = 100;
  TIMSK1 |= _BV(TOIE1); // Enable Timer1 interrupt
  sei();                // Enable global interrupts
}

void swapBuffers(boolean copy) 
{
  if (matrixbuff[0] != matrixbuff[1]) 
  {
    // To avoid 'tearing' display, actual swap takes place in the interrupt
    // handler, at the end of a complete screen refresh cycle.
    swapflag = true; // Set flag here, then...
    while (swapflag == true)
      delay(1); // wait for interrupt to clear it
    if (copy == true)
      memcpy(matrixbuff[backindex], matrixbuff[1 - backindex],
             WIDTH * nRows * 3);
  }
}

void drawPixel(int16_t x, int16_t y, uint16_t c) 
{
  uint8_t r, g, b, bit, limit, *ptr;

  if ((x < 0) || (x >= WIDTH) || (y < 0) || (y >= HEIGHT))
    return;
/*  int rotation = 0;
  
  switch (rotation) {
  case 1:
    _swap_int16_t(x, y);
    x = WIDTH - 1 - x;
    break;
  case 2:
    x = WIDTH - 1 - x;
    y = HEIGHT - 1 - y;
    break;
  case 3:
    _swap_int16_t(x, y);
    y = HEIGHT - 1 - y;
    break;
  }*/

  // Adafruit_GFX uses 16-bit color in 5/6/5 format, while matrix needs
  // 4/4/4.  Pluck out relevant bits while separating into R,G,B:
  r = c >> 12;        // RRRRrggggggbbbbb
  g = (c >> 7) & 0xF; // rrrrrGGGGggbbbbb
  b = (c >> 1) & 0xF; // rrrrrggggggBBBBb

  // Loop counter stuff
  bit = 2;
  limit = 1 << nPlanes;

  if (y < nRows) {
    // Data for the upper half of the display is stored in the lower
    // bits of each byte.
    ptr = &matrixbuff[backindex][y * WIDTH * (nPlanes - 1) + x]; // Base addr
    // Plane 0 is a tricky case -- its data is spread about,
    // stored in least two bits not used by the other planes.
    ptr[WIDTH * 2] &= ~B00000011; // Plane 0 R,G mask out in one op
    if (r & 1)
      ptr[WIDTH * 2] |= B00000001; // Plane 0 R: 64 bytes ahead, bit 0
    if (g & 1)
      ptr[WIDTH * 2] |= B00000010; // Plane 0 G: 64 bytes ahead, bit 1
    if (b & 1)
      ptr[WIDTH] |= B00000001; // Plane 0 B: 32 bytes ahead, bit 0
    else
      ptr[WIDTH] &= ~B00000001; // Plane 0 B unset; mask out
    // The remaining three image planes are more normal-ish.
    // Data is stored in the high 6 bits so it can be quickly
    // copied to the DATAPORT register w/6 output lines.
    for (; bit < limit; bit <<= 1) {
      *ptr &= ~B00011100; // Mask out R,G,B in one op
      if (r & bit)
        *ptr |= B00000100; // Plane N R: bit 2
      if (g & bit)
        *ptr |= B00001000; // Plane N G: bit 3
      if (b & bit)
        *ptr |= B00010000; // Plane N B: bit 4
      ptr += WIDTH;        // Advance to next bit plane
    }
  } else {
    // Data for the lower half of the display is stored in the upper
    // bits, except for the plane 0 stuff, using 2 least bits.
    ptr = &matrixbuff[backindex][(y - nRows) * WIDTH * (nPlanes - 1) + x];
    *ptr &= ~B00000011; // Plane 0 G,B mask out in one op
    if (r & 1)
      ptr[WIDTH] |= B00000010; // Plane 0 R: 32 bytes ahead, bit 1
    else
      ptr[WIDTH] &= ~B00000010; // Plane 0 R unset; mask out
    if (g & 1)
      *ptr |= B00000001; // Plane 0 G: bit 0
    if (b & 1)
      *ptr |= B00000010; // Plane 0 B: bit 0
    for (; bit < limit; bit <<= 1) {
      *ptr &= ~B11100000; // Mask out R,G,B in one op
      if (r & bit)
        *ptr |= B00100000; // Plane N R: bit 5
      if (g & bit)
        *ptr |= B01000000; // Plane N G: bit 6
      if (b & bit)
        *ptr |= B10000000; // Plane N B: bit 7
      ptr += WIDTH;        // Advance to next bit plane
    }
  }
}

#define CALLOVERHEAD 60 // Actual value measured = 56
#define LOOPTIME 200    // Actual value measured = 188

//Прерывание на отрисовку по таймеру

ISR(TIMER1_OVF_vect, ISR_BLOCK) 
{ // ISR_BLOCK important -- see notes later
  updateDisplay();   // Call refresh func for active display
  TIFR1 |= TOV1;                  // Clear Timer1 interrupt flag
}

void updateDisplay(void) 
{
  uint8_t i, tick, tock, *ptr;
  uint16_t t, duration;

  *oeport |= oemask;   // Disable LED output during row/plane switchover
  *latport |= latmask; // Latch data loaded during *prior* interrupt

  // Calculate time to next interrupt BEFORE incrementing plane #.
  // This is because duration is the display time for the data loaded
  // on the PRIOR interrupt.  CALLOVERHEAD is subtracted from the
  // result because that time is implicit between the timer overflow
  // (interrupt triggered) and the initial LEDs-off line at the start
  // of this method.
  t = (nRows > 8) ? LOOPTIME : (LOOPTIME * 2);
  duration = ((t + CALLOVERHEAD * 2) << plane) - CALLOVERHEAD;

  // Borrowing a technique here from Ray's Logic:
  // www.rayslogic.com/propeller/Programming/AdafruitRGB/AdafruitRGB.htm
  // This code cycles through all four planes for each scanline before
  // advancing to the next line.  While it might seem beneficial to
  // advance lines every time and interleave the planes to reduce
  // vertical scanning artifacts, in practice with this panel it causes
  // a green 'ghosting' effect on black pixels, a much worse artifact.

  if (++plane >= nPlanes) {   // Advance plane counter.  Maxed out?
    plane = 0;                // Yes, reset to plane 0, and
    if (++row >= nRows) {     // advance row counter.  Maxed out?
      row = 0;                // Yes, reset row counter, then...
      if (swapflag == true) { // Swap front/back buffers if requested
        backindex = 1 - backindex;
        swapflag = false;
      }
      buffptr = matrixbuff[1 - backindex]; // Reset into front buffer
    }
  } 
  else if (plane == 1) 
  {
    // Plane 0 was loaded on prior interrupt invocation and is about to
    // latch now, so update the row address lines before we do that:
    if (row & 0x1)
      *addraport |= addramask;
    else
      *addraport &= ~addramask;
    // MYSTERY: certain matrices REQUIRE these delays ???
    delayMicroseconds(1);
  //delay(1);
    if (row & 0x2)
      *addrbport |= addrbmask;
    else
      *addrbport &= ~addrbmask;
    delayMicroseconds(1);
  //delay(1);
    if (row & 0x4)
      *addrcport |= addrcmask;
    else
      *addrcport &= ~addrcmask;
    delayMicroseconds(1);
  //delay(1);
    if (nRows > 8) 
  {
      if (row & 0x8)
        *addrdport |= addrdmask;
      else
        *addrdport &= ~addrdmask;
      delayMicroseconds(1);
    //delay(1);
    }
  }

  // buffptr, being 'volatile' type, doesn't take well to optimization.
  // A local register copy can speed some things up:
  ptr = (uint8_t *)buffptr;

//#if defined(__AVR__)
  ICR1 = duration; // Set interval for next interrupt
  TCNT1 = 0;       // Restart interrupt timer
  *oeport &= ~oemask;   // Re-enable output
  *latport &= ~latmask; // Latch down

  // Record current state of CLKPORT register, as well as a second
  // copy with the clock bit set.  This makes the innnermost data-
  // pushing loops faster, as they can just set the PORT state and
  // not have to load/modify/store bits every single time.  It's a
  // somewhat rude trick that ONLY works because the interrupt
  // handler is set ISR_BLOCK, halting any other interrupts that
  // might otherwise also be twiddling the port at the same time
  // (else this would clobber them). only needed for AVR's where you
  // cannot set one bit in a single instruction
  tock = CLKPORT;
  tick = tock | clkmask;

  if (plane > 0) { // 188 ticks from TCNT1=0 (above) to end of function

    // Planes 1-3 copy bytes directly from RAM to PORT without unpacking.
    // The least 2 bits (used for plane 0 data) are presumed masked out
    // by the port direction bits.

// A tiny bit of inline assembly is used; compiler doesn't pick
// up on opportunity for post-increment addressing mode.
// 5 instruction ticks per 'pew' = 160 ticks total
#define pew                                                                    \
  asm volatile("ld  __tmp_reg__, %a[ptr]+"                                     \
               "\n\t"                                                          \
               "out %[data]    , __tmp_reg__"                                  \
               "\n\t"                                                          \
               "out %[clk]     , %[tick]"                                      \
               "\n\t"                                                          \
               "out %[clk]     , %[tock]"                                      \
               "\n" ::[ptr] "e"(ptr),                                          \
               [ data ] "I"(_SFR_IO_ADDR(DATAPORT)),                           \
               [ clk ] "I"(_SFR_IO_ADDR(CLKPORT)), [ tick ] "r"(tick),         \
               [ tock ] "r"(tock));

    // Loop is unrolled for speed:
    pew pew pew pew pew pew pew pew pew pew pew pew pew pew pew pew pew pew pew
        pew pew pew pew pew pew pew pew pew pew pew pew pew

        if (WIDTH == 64) {
      pew pew pew pew pew pew pew pew pew pew pew pew pew pew pew pew pew pew
          pew pew pew pew pew pew pew pew pew pew pew pew pew pew
    }

    buffptr = ptr; //+= 32;

  } else { // 920 ticks from TCNT1=0 (above) to end of function
    // Planes 1-3 (handled above) formatted their data "in place,"
    // their layout matching that out the output PORT register (where
    // 6 bits correspond to output data lines), maximizing throughput
    // as no conversion or unpacking is needed.  Plane 0 then takes up
    // the slack, with all its data packed into the 2 least bits not
    // used by the other planes.  This works because the unpacking and
    // output for plane 0 is handled while plane 3 is being displayed...
    // because binary coded modulation is used (not PWM), that plane
    // has the longest display interval, so the extra work fits.
    for (i = 0; i < WIDTH; i++) {
      DATAPORT = (ptr[i] << 6) | ((ptr[i + WIDTH] << 4) & 0x30) |
                 ((ptr[i + WIDTH * 2] << 2) & 0x0C);
      CLKPORT = tick; // Clock lo
      CLKPORT = tock; // Clock hi
    }
  }
}

//Все примитивы рисуются на базе drawpixel - это медленно
//В идеале для скорости работы надо переделать на memcpy/memset в фреймбуферы buffptr matrixbuff

void fillScreen(uint16_t c) 
{
  int i,j;
  for(i=0;i<32;i++)
  {
    for(j=0;j<64;j++)
    {
      drawPixel(j, i,  c);
    }
  }
}

void loop() 
{
  int i,j;
  
  //кодировка цвета
  //RRRRrGGGGggBBBBb

  // закрашиваем матрицу в белый цвет
  //memset(matrixbuff[backindex], 0xFFFF, WIDTH * nRows * 3);

  fillScreen(0xF000);
  // выводим цвет из буфера на экран
  swapBuffers(false);
  delay(1000);

  fillScreen(0x780);
  // выводим цвет из буфера на экран
  swapBuffers(false);
  delay(1000);

  fillScreen(0x1e);
  // выводим цвет из буфера на экран
  swapBuffers(false);
  delay(1000);

  /*for(i=0;i<15;i++)
  {
    for(j=0;j<16;j++)
    {
      drawPixel(j,    i,  j<<12|i<<1);
      drawPixel(16+j, i,  j<<12|i<<1|0x180);
      drawPixel(32+j, i,  j<<12|i<<1|0x380);
      drawPixel(48+j, i,  j<<12|i<<1|0x780);
    }
  }
 
  // выводим цвет из буфера на экран
  swapBuffers(false);
  delay(1000);*/
}
