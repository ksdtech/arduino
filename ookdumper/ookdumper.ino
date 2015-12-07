//
// Simple OOK playground -- receives and dumps received signal and timing
//

#include <RFM69OOK.h>
#include <SPI.h>
#include <RFM69OOKregisters.h>

#define TSIZE 400
#define MAX_0_DUR 100000 // 100 ms
#define MIN_1_DUR 50 // 100 us
#define TOL 50 // +- tolerance

#define BIT_CHK 3
#define BIT_SYNC 2
#define BIT_HI 1
#define BIT_LO 0
const char* display_bits = "01Sx";
char channel_bits[4] = { 0, 0, 0, 0 };

#define DEBUG_LEVEL 0

RFM69OOK radio;

unsigned long t0 = 0;
unsigned long t0_last = 0;
uint16_t tt[TSIZE];

byte s0 = 0;
byte pos = 0;
bool restart = true;

// See RFM69 data sheet, section 3.3.2. Bit Rate Setting
// FxOSC = 32000000 (32 MHz)
// BITRATE = FxOSC / Desired BR
// So, for desired 8.0kb/s BR, BITRATE = 32000000 / 8000 = 4000 = 0x0FA0
#define RF_BITRATEMSB_8000 0x0F
#define RF_BITRATELSB_8000 0xA0

/* More attempts to be AccuRite-compatible
 * From https://hackaday.io/project/4490-re-purposing-acurite-temperature-sensors/log/14763-making-the-rfm69-receive-acurite-data
 */
void setAccuriteRx() {
  // rfm69_write(RegOpMode, RegOpModeStandby);
  radio.receiveEnd();

  // rfm69_write(RegDataModul, RegDataModulContinuous | RegDataModulOOK); // Set continuous OOK mode
  // already set in radio.initialize(), CONFIG[1]

  // RegBitrateSet(8000); // 8.0kb/s
  // in radio.initialize(), CONFIG[2] sets bitrate to 32768 b/s
  radio.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_8000);
  radio.writeReg(REG_BITRATELSB, RF_BITRATELSB_8000);

  // RegFrfSet(433920000); // fundamental frequency = 433.92MHz (really 433.920044 MHz)
  radio.setFrequencyMHz(433.9);

  // rfm69_write(RegRxBw, RegRxBwDccFreq4 | RegRxBwOOK50k); // 4% DC cancellation; 50k bandwidth in OOK mode
  // in radio.initialize(), CONFIG[4] sets RF_RXBW_DCCFREQ_010 and 10.4k bandwitth
  radio.setBandwidth(OOK_BW_50_0);
  // or radio.setBandwidth(OOK_BW_20_8);
  // or radio.setBandwidth(OOK_BW_10_4);

  // rfm69_write(RegLna, RegLnaZ200 | RegLnaGainSelect12db); // 200 ohm, -12db
  radio.writeReg(REG_LNA, RF_LNA_ZIN_200 | RF_LNA_GAINSELECT_MAXMINUS12);

  // rfm69_write(RegOokPeak, RegOokThreshPeak | RegOokThreshPeakStep0d5 | RegOokThreshPeakDec1c );
  // See the RFM69 data sheet, section 3.4.12. OOK Demodulator
  // I guess the above settings mean 0.5 dB decrement, period once per chip
  // already set in radio.initialize(), CONFIG[5] sets RF_OOKPEAK_THRESHTYPE_PEAK | RF_OOKPEAK_PEAKTHRESHSTEP_000 | RF_OOKPEAK_PEAKTHRESHDEC_000

  // If we were to use RF_OOKPEAK_THRESHTYPE_FIXED, then we'd set one of these
  // radio.setFixedThreshold(30);
  // radio.setFixedThreshold(40);
  // radio.setFixedThreshold(50);
  // radio.setFixedThreshold(55);
  radio.setFixedThreshold(60);

  // radio.setSensitivityBoost(SENSITIVITY_BOOST_HIGH);

  // rfm69_write(RegOpMode, RegOpModeRX);
  radio.receiveBegin();

  // rfm69_write(RegAfcFei, RegAfcFeiAfcClear);
  radio.writeReg(REG_AFCFEI, RF_AFCFEI_AFC_CLEAR);
}
 
void setup() {
  Serial.begin(115200);

  radio.initialize();
  setAccuriteRx();

  Serial.println(F("start"));
}

void loop() {
  unsigned long t;
  unsigned long d;
  byte s;
  uint16_t d_hi, d_lo;
  
  if (restart) {
    while (true) {
      if (radio.poll()) {
        break;
      }
#if DEBUG_LEVEL > 2
      t = micros();
      if (t >= (t0_last + 1000000)) {
        Serial.print(t / 1000000.0, 6);
        Serial.println(F(": waiting for HI"));
        t0_last = t;
      }
#endif
    }
    t0 = micros();
    s0 = 1;
    pos = 0;
    restart = false;
#if DEBUG_LEVEL > 2
    Serial.print(t0 / 1000000.0, 6);
    Serial.println(F(": HI received ater restart"));
#endif
    return;
  }

  s = radio.poll() ? 1 : 0;
  t = micros();
  d = t - t0;

  if (s0 != s) {
    if (s == 0 && d < MIN_1_DUR) {
#if DEBUG_LEVEL > 3
      Serial.print(t / 1000000.0, 6);
      Serial.print(F(": Short HI: "));
      Serial.print(d);
      Serial.println(F(", restarting"));
#endif
      restart = true;
      return;
    }
    tt[pos++] = d;
    t0 = t;
    s0 = s;
  }
  
  // for our tests, pos is 151
  if (pos >= TSIZE || s0 == 0 && d > MAX_0_DUR) {
#if DEBUG_LEVEL > 1    
    Serial.print(F("Long LO:  "));
    Serial.println(d);
#endif
    s = 1;
#if DEBUG_LEVEL > 0
    for (int i = 0; i < pos; i++) {
      Serial.print(i);
      Serial.print(F(": "));
      Serial.print(s);
      Serial.print(' ');
      Serial.println(tt[i]);
      s = !s;
    }
#else
    long bitval = 0;
    Serial.println(t0 / 1000000.0, 6);
    Serial.print("Bits ");
    for (int i = 0; i < pos; i+= 2) {
      d_hi = tt[i];
      d_lo = 0;
      if ((i + 1) < pos) {
        d_lo = tt[i+1];      
      }
      if (i == 16 || (i > 16 && i % 16 == 8)) {
        Serial.print(" ");
      }
      int c = d_hi > 300 ? (d_lo < 300 ? BIT_HI : BIT_SYNC) : BIT_LO;
      if (i >= 72 && i < 136) {
        if (i % 16 == 8) {
          c = BIT_CHK;
        } else {
          bitval *= 2;
          if (c == BIT_HI) {
            bitval += 1;
          }
        }
      }
      Serial.print(display_bits[c]);
    }
    Serial.println();
    Serial.print("Value ");
    Serial.println(bitval);
#endif

#if DEBUG_LEVEL > 2
    Serial.print(t0 / 1000000.0, 6);
    Serial.println(F(": restarting"));
#endif
    t0_last = 0;
    restart = true;
  }
}

