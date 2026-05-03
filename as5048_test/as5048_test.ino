#include <SPI.h>

/*
 * AS5048A Deep Diagnostic — all 4 SPI modes, slow clock, extended CS timing
 * Wiring: CS->D10, SCK->D13, MISO->D12, MOSI->D11, 5V->5V(P1-8), GND->GND(P1-1)
 *
 * Both modes returned 0x0000 -> sensor IS driving MISO but outputting zeros.
 * Cause: CS timing, power, or parity. This sketch tests all 4 modes + raw dump.
 */

const int      CS_PIN = 10;
const uint32_t CLK    = 50000;  // 50 kHz -- very slow to rule out timing

void csLow()  { digitalWrite(CS_PIN, LOW);  delayMicroseconds(20); }
void csHigh() { digitalWrite(CS_PIN, HIGH); delayMicroseconds(20); }

uint16_t addParity(uint16_t v) {
  uint8_t cnt = 0; uint16_t tmp = v;
  for (uint8_t i = 0; i < 16; i++) { if (tmp & 1) cnt++; tmp >>= 1; }
  return v | ((uint16_t)(cnt & 1) << 15);
}

uint16_t xfer16(uint16_t cmd) {
  csLow();
  uint16_t r = SPI.transfer16(cmd);
  csHigh();
  delay(2);
  return r;
}

void testMode(const char* label, uint8_t mode) {
  SPI.beginTransaction(SPISettings(CLK, MSBFIRST, mode));
  xfer16(0x0000); xfer16(0x0000); xfer16(0x0000);  // flush pipeline

  uint16_t cmd      = addParity(0x4000 | 0x3FFF);
  uint16_t pipeline = xfer16(cmd);   // returns previous (garbage)
  delay(5);
  uint16_t resp     = xfer16(0x0000); // NOP -> angle from previous cmd
  SPI.endTransaction();

  uint16_t raw14 = resp & 0x3FFF;
  float    deg   = raw14 * (360.0f / 16383.0f);
  uint8_t  err   = (resp >> 14) & 1;

  Serial.print(label);
  Serial.print(F("  pipe=0x")); Serial.print(pipeline, HEX);
  Serial.print(F("  resp=0x")); Serial.print(resp, HEX);
  Serial.print(F("  raw14="));  Serial.print(raw14);
  Serial.print(F("  "));        Serial.print(deg, 1); Serial.print(F(" deg"));
  Serial.print(F("  err="));    Serial.print(err);

  if (resp == 0x0000)
    Serial.println(F("  [all zeros]"));
  else if (raw14 == 0x3FFF)
    Serial.println(F("  [MISO floating]"));
  else if (err)
    Serial.println(F("  [err flag -- check magnet orientation/distance]"));
  else
    Serial.println(F("  *** DATA OK ***"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  pinMode(CS_PIN, OUTPUT);
  csHigh();
  SPI.begin();
  delay(200);

  Serial.println(F("=== AS5048A Deep Diagnostic ==="));
  Serial.println(F("Rotate magnet between readings -- raw14 should change."));
  Serial.println();
  Serial.println(F("WIRING: P1-8(5V)->5V  P1-1(GND)->GND  P1-5(CSn)->D10"));
  Serial.println(F("        P1-4(SCK)->D13  P1-2(MISO)->D12  P1-3(MOSI)->D11"));
  Serial.println(F("        P1-7(3.3V) -- leave UNCONNECTED (it is an OUTPUT)"));
  Serial.println();
}

void loop() {
  testMode("Mode0", SPI_MODE0);
  testMode("Mode1", SPI_MODE1);
  testMode("Mode2", SPI_MODE2);
  testMode("Mode3", SPI_MODE3);

  Serial.print(F("RawDump(0xFFFF x3): "));
  SPI.beginTransaction(SPISettings(CLK, MSBFIRST, SPI_MODE1));
  for (int i = 0; i < 3; i++) {
    uint16_t v = xfer16(0xFFFF);
    Serial.print(F(" 0x")); Serial.print(v, HEX);
  }
  SPI.endTransaction();
  Serial.println();
  Serial.println();
  delay(2000);
}
