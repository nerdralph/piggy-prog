// Piggy-prog AVR High-voltage Serial Fuse Resetter
// by Nerd Ralph http://nerdralph.blogspot.ca
// Adapted from code and design by Paul Willoughby 03/20/2010
//   http://www.rickety.us/2010/03/arduino-avr-high-voltage-serial-programmer/
//
// works with tiny13, tinyX4, tinyX5
// designed for use with a Pro Mini on top of an 8-pin tiny AVR
// Blog article:
// http://nerdralph.blogspot.ca/2018/05/piggyfuse-hvsp-avr-fuse-programmer.html

#define  RSTMON  14    // reset sense 22K resistor
//#define  LED     13
#define  GND     12    // Target GND
#define  RST12   11    // To NPN+PNP switch for RST 12V
#define  SCI     10    // Target Clock Input
#define  SDO      9    // Target Data Output
#define  SII      8    // Target Instruction Input
#define  SDI      7    // Target Data Input
#define  VCC      6    // Target VCC

#include "WProgram.h"
void setup();
void loop();
void oscTune();
byte shiftOut (byte sdata, byte sinstr);
byte sendCmd(HvCmd command);
void loadLoAddr(byte addr);
void busyWait();
void loadData(byte addr, int data);
void writeSig(byte osccal);
void loadPage(byte buf[PageSize]);
void sigErase();
void chipErase();
void writeFuse (unsigned int fuse, byte val);
byte readFuse(FuseOp fusecmd);
void dumpBuf();
void printFuses ();
void readSigPage();
byte readOsccal();
unsigned int readSignature ();
const int OSCTUNE=1;

#define  WLFUSE  0x646C
#define  WHFUSE  0x747C
#define  WEFUSE  0x666E

const int PageSize = 32;
byte PageBuf[64];       // max ATtiny page size

// fuse operation codes
enum FuseOp {
  RLFUSE = 0x686C,
  RHFUSE = 0x7A7E,
  REFUSE = 0x6A6E,
  RLOCK = 0x787C  
};

// HV programming commands
enum HvCmd {
  NoOp = 0x00,
  ReadFuse = 0x04,
  ReadSig = 0x08,
  WriteFlash = 0x10,
  WriteFuse = 0x40,
  ChipErase = 0x80
};

// Define ATTiny series signatures
#define  ATTINY13   0x9007  // L: 0x7A, H: 0xFF             8 pin
#define  ATTINY24   0x910B  // L: 0xE2, H: 0xDF, E: 0xFF   14 pin
#define  ATTINY25   0x9108  // L: 0xE2, H: 0xDF, E: 0xFF    8 pin
#define  ATTINY44   0x9207  // L: 0xE2, H: 0xDF, E: 0xFF   14 pin
#define  ATTINY45   0x9206  // L: 0xE2, H: 0xDF, E: 0xFF    8 pin
#define  ATTINY84   0x930C  // L: 0xE2, H: 0xDF, E: 0xFF   14 pin
#define  ATTINY85   0x930B  // L: 0xE2, H: 0xDF, E: 0xFF    8 pin
#define  BLANK      0xFFFF  // blank signature

byte success;

void setup() {
  pinMode(GND, OUTPUT);
  pinMode(RST12, OUTPUT);
  pinMode(RSTMON, OUTPUT);
  pinMode(VCC, OUTPUT);
  pinMode(SDI, OUTPUT);
  pinMode(SII, OUTPUT);
  pinMode(SCI, OUTPUT);
  pinMode(SDO, OUTPUT);     // Configured as input when in programming mode
  Serial.begin(57600);
  Serial.println(F("\nHSVP fuse resetter"));

  digitalWrite(VCC, HIGH);  // Vcc On
  delayMicroseconds(20);    // wait 20us for VCC to rise
  pinMode(RST12, INPUT);
  digitalWrite(RST12, HIGH);   // 12V to RST
  pinMode(RSTMON, INPUT);
  pinMode(SDO, INPUT);      // Set SDO to input before target drives it high
  delayMicroseconds(300);   // wait for target ready
  
  // target should now be in programming mode
  readSigPage();
  dumpBuf();
  unsigned int sig = readSignature();
  Serial.print(F("Signature: "));
  Serial.println(sig, HEX);
  Serial.print(F("OSCCAL: "));
  Serial.println(PageBuf[1], HEX);
  printFuses();
  long startMicros = micros();
  byte lock = readFuse(RLOCK);
  Serial.print(micros() - startMicros);
  Serial.println(F(" us fuse read time."));
  if ( lock != 0xFF) {
    Serial.println(F("Lock bits set.  Performing chip erase."));
    chipErase();
  }
  if (sig == BLANK) {
    Serial.println(F("Blank signature. resetting to t13"));
    writeSig(0x68);
  }
  //writeSig(0x60);
  success = 1;
  byte hfuse;
  if (sig == ATTINY13) {
    Serial.println(F("t13."));
    //hfuse = 0xE4;   // SELFPRG, DW, BOD1.8V, RSTDSBL
    hfuse = 0xE7;   // SELFPRG, DW
    if (readFuse(RHFUSE) == hfuse && !OSCTUNE){
      //hfuse = 0xEE;   // SELFPRG, RSTDSBL
      hfuse = 0xFF;   // alternate fuse setting
      success = 2;
    }
    startMicros = micros();
    // lfuse = SPI EES WDT CK8 SU1 SU0 CK1 CK0
    // 0x76 = SPIEN, 4ms startup, internal RC
    writeFuse(WLFUSE, 0x76);
    writeFuse(WHFUSE, hfuse);
  } else if (sig == ATTINY24 || sig == ATTINY44 || sig == ATTINY84 ||
             sig == ATTINY25 || sig == ATTINY45 || sig == ATTINY85) {
    Serial.println(F("tX[45]."));
    //hfuse = 0x1E;     // RSTDSBL, DW, SPIEN, BOD1.8V
    hfuse = 0x9F;     // DW, SPIEN
    if (readFuse(RHFUSE) == hfuse && !OSCTUNE){
      //hfuse = 0x5F;   // RSTDSBL, SPIEN
      hfuse = 0xDF;   // alternate fuse setting SPIEN
      success = 2;
    }
    startMicros = micros();
    writeFuse(WLFUSE, 0xE2);
    writeFuse(WHFUSE, hfuse);
    writeFuse(WEFUSE, 0xFE);
  } else if (sig == BLANK) {
    Serial.println(F("missing signature."));
    success = 0;
  } else {
    Serial.println(F("unrecognized device."));
    success = 0;
  }
  int duration = micros() - startMicros;
  if (success == 0){
    Serial.print(F("No"));
  } else if (success == 1) {
    Serial.print(F("Debug"));
  } else {
    Serial.print(F("Product"));
  }
  Serial.println(F(" fuses programmed."));

  Serial.print(duration);
  Serial.println(F(" us."));
  
  sig = readSignature();
  Serial.print(F("Signature: "));
  Serial.println(sig, HEX);
  printFuses();
  Serial.println(F("Fuse resetter finished."));
  Serial.println(F("Reset programmer to run again."));
}

void loop() {
  pinMode(SDI, INPUT);
  pinMode(SII, INPUT);
  pinMode(SCI, INPUT);
  digitalWrite(RST12, LOW);      // turns off 12V
  pinMode(RST12, OUTPUT);
  digitalWrite(VCC, LOW);

  if (OSCTUNE) oscTune();

  digitalWrite(LED, HIGH);
  if ( !success){
    digitalWrite(VCC, LOW);     // Vcc Off
    delay(1000);
  } else {
    delay(100);                 // 1 short flash for dW fuses
    if (success == 2){          // 2 short flashes for no dW
      digitalWrite(LED, LOW);
      delay(100);
      digitalWrite(LED, HIGH);
      delay(100);
    }
  }
  digitalWrite(LED, LOW);
  delay(2000);
}

void oscTune(){
  pinMode(RSTMON, OUTPUT);    // low
  delay(1);       // wait for power down
  pinMode(RSTMON, INPUT);
  digitalWrite(VCC, HIGH);
  long startMicros = micros();
  while (digitalRead(RSTMON) == LOW);
  Serial.print(micros() - startMicros);
  Serial.println(F(" us RESET high delay."));
  pinMode(RSTMON, OUTPUT);    // pulse reset
  delayMicroseconds(50);
  pinMode(RSTMON, INPUT);
  Serial.println(F("oscTune fininshed."));
}

byte shiftOut (byte sdata, byte sinstr) {
  int inBits = 0;
  unsigned int dout = (unsigned int) sdata << 2;
  unsigned int iout = (unsigned int) sinstr << 2;
  for (int ii = 10; ii >= 0; ii--)  {
    //digitalWrite(SDI, !!(dout & (1 << ii)));
    //digitalWrite(SII, !!(iout & (1 << ii)));
    if (dout & 1<<10)
      digitalWrite(SDI, HIGH);
    else
      digitalWrite(SDI, LOW);
    if (iout & 1<<10)
      digitalWrite(SII, HIGH);
    else
      digitalWrite(SII, LOW);
    dout <<= 1;
    iout <<= 1;
    inBits <<= 1;
    inBits |= digitalRead(SDO);
    digitalWrite(SCI, HIGH);
    digitalWrite(SCI, LOW);
  }
  return inBits >> 2;
}

byte sendCmd(HvCmd command) {
  return shiftOut(command, 0x4C);
}

void loadLoAddr(byte addr) {
  shiftOut(addr, 0x0C);
}

void busyWait(){
  while (!digitalRead(SDO));        // wait until SDO goes high
}

void loadData(byte addr, int data) {
  loadLoAddr(addr);
  shiftOut(data, 0x2C);
  shiftOut(data>>8, 0x3C);
  shiftOut(0x0, 0x7D);
  shiftOut(0x0, 0x7C);  
}

void writeSig(byte osccal)
{
  sendCmd(WriteFlash);
  // osscal is stored in high byte of address 0
  loadData(0, osccal << 8 | 0x1E);
  // t13 signature = 0x9007
  loadData(1, 0xFF90);
  loadData(2, 0xFF07);
  // undocumented sig page program command
  shiftOut(0x93, 0x55);
  shiftOut(0x0, 0x45);
  shiftOut(0x0, 0x35);
  busyWait();
  sendCmd(NoOp);
}

void loadPage(byte buf[PageSize]) {
  sendCmd(WriteFlash);
  // for (byte addr = 0; addr
}

void sigErase()
{
  shiftOut(ChipErase, 0xCC);
  shiftOut(0x00, 0x64);
  shiftOut(0x00, 0x6C);
  busyWait();
}

void chipErase()
{
  sendCmd(ChipErase);
  shiftOut(0x00, 0x64);
  shiftOut(0x00, 0x6C);
  busyWait();
  //sendCmd(NoOp);
}

void writeFuse (unsigned int fuse, byte val) {
  sendCmd(WriteFuse);
  shiftOut( val, 0x2C);
  shiftOut(0x00, (byte) (fuse >> 8));
  shiftOut(0x00, (byte) fuse);
  busyWait();
}

byte readFuse(FuseOp fusecmd){
  sendCmd(ReadFuse);
  shiftOut(0x00, (byte) (fusecmd >> 8));
  //return shiftOut(0x00, (byte) fusecmd);
  return sendCmd(NoOp);
}

void dumpBuf(){
    for (byte i = 0; i < PageSize; i++) {
        Serial.print(PageBuf[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
}

void printFuses () {
  byte val;
  val = readFuse(RLFUSE);
  Serial.print(F("LFuse: "));
  Serial.print(val, HEX);
  val = readFuse(RHFUSE);
  Serial.print(F(", HFuse: "));
  Serial.print(val, HEX);
  val = readFuse(REFUSE);
  Serial.print(F(", EFuse: "));
  Serial.print(val, HEX);
  val = readFuse(RLOCK);
  Serial.print(F(", Lock: "));
  Serial.println(val, HEX);
}

void readSigPage(){
  for (int addr = 0; addr < PageSize/2; addr++) {
    sendCmd(ReadSig);
    loadLoAddr(addr);
    shiftOut(0x00, 0x68);
    PageBuf[addr*2] = shiftOut(0x00, 0x78);
    //PageBuf[addr*2] = shiftOut(0x00, 0x6C);
    //shiftOut(0x00, 0x78);
    PageBuf[addr*2+1] = shiftOut(0x00, 0x6C);
  }
}

byte readOsccal() {
  sendCmd(ReadSig);
  loadLoAddr(0);
  shiftOut(0x00, 0x78);
  return sendCmd(NoOp);
}

unsigned int readSignature () {
  unsigned int sig = 0;
  byte val;
  for (int ii = 1; ii < 3; ii++) {
          sendCmd(ReadSig);
          loadLoAddr(ii);
          shiftOut(0x00, 0x68);
    val = shiftOut(0x00, 0x6C);
    sig = (sig << 8) + val;
  }
  return sig;
}


