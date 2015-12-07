//
// AcuRite 00899 Rain Gauge Receiver
// Receives and prints out time of message and gauge value (number 
// of "clicks" of tipping bucket. Each click measures approx. 0.01 inches of rain.
//
// One message is broadcast on the 433 Mhz band using OOK every 60 seconds.
// Some trial and error involved in setting RFM69 bitrate, bandwidth
// and OOK threshold.  Depends on sensitivity, distance, etc.
//

#include <RFM69OOK.h>
#include <SPI.h>
#include <RFM69OOKregisters.h>

#define MAX_0_DUR 100000 // 100 ms
#define MIN_1_DUR 50 // 100 us
#define HI_LO_THRESHOLD 300 // 300 us
#define MESSAGE_LEN 76 // number of bits in message
#define TSIZE 400 // we actually only need MESSAGE_LEN*2 + 1

#define BIT_CHK 3
#define BIT_SYNC 2
#define BIT_HI 1
#define BIT_LO 0

// Instance of RFM69OOK library
RFM69OOK radio;

// Store durations in buffer
unsigned long t0 = 0;
unsigned long t0_last = 0;
uint16_t tt[TSIZE];

byte s0 = 0;
byte pos = 0;
bool restart = true;

// Setting the RFM69 bitrate
// See this article: http://www.sevenwatt.com/main/rfm69-ook-rssi-and-optimal-bitrate/
// We need to detect 200 usec, 400 usec and 600 usec signals
// By trial and error a bitrate of 8000 bps seems to work.
// t-bit = 125 usec, t-rssi = 2 * t-bit = 250 usec

// Register values for 8000 bps. See RFM69 data sheet, section 3.3.2. Bit Rate Setting
// FxOSC = 32000000 (32 MHz)
// BITRATE = FxOSC / Desired BR
// So, for desired 8000 bps BR, BITRATE = 32000000 / 8000 = 4000 = 0x0FA0
#define RF_BITRATEMSB_8000 0x0F
#define RF_BITRATELSB_8000 0xA0

// RFM69 settings to be AcuRite-compatible.  
// Comments with rfm69_write, RegBitrateSet, RegFrfSet are recommendations from this AcuRite project:
//   https://hackaday.io/project/4490-re-purposing-acurite-temperature-sensors/log/14763-making-the-rfm69-receive-acurite-data
// radio.initialize() call in RFM69OOK library already does a lot of the setup work for us.
void setAcuRiteRx() {
  // Go to standby mode.
  // - rfm69_write(RegOpMode, RegOpModeStandby);
  // This is set in RFM69OOK library here:
  radio.receiveEnd();

  // Set continuous OOK mode.
  // - rfm69_write(RegDataModul, RegDataModulContinuous | RegDataModulOOK); 
  // This is set in radio.initialize(), CONFIG[1]

  // Set bitrate.
  // - RegBitrateSet(8000); // 8000 bps
  // In radio.initialize(), CONFIG[2] sets bitrate to 32768 bps. We reset it here:
  radio.writeReg(REG_BITRATEMSB, RF_BITRATEMSB_8000);
  radio.writeReg(REG_BITRATELSB, RF_BITRATELSB_8000);

  // Set receiver frequency
  // - RegFrfSet(433920000); // fundamental frequency = 433.92 MHz (really 433.920044 MHz)
  radio.setFrequencyMHz(433.9);

  // Set bandwidth
  // 4% DC cancellation; 50 KHz bandwidth in OOK mode
  // - rfm69_write(RegRxBw, RegRxBwDccFreq4 | RegRxBwOOK50k); 
  // In radio.initialize(), CONFIG[4] sets RF_RXBW_DCCFREQ_010 and 10.4 KHz bandwitth
  // We adjust the bandwidth here.  The AcuRite has 3 different "channels" (A, B and C).
  // Not sure what the actual frequencies used are, but 50 KHz bandwdith seems to work for channel A:
  radio.setBandwidth(OOK_BW_50_0);

  // For tighter channels, find actual frequency and try these settings (20.8 or 10.4 KHz):
  // radio.setBandwidth(OOK_BW_20_8);
  // radio.setBandwidth(OOK_BW_10_4);

  // Set gain select
  // rfm69_write(RegLna, RegLnaZ200 | RegLnaGainSelect12db); // 200 ohm, -12 db
  radio.writeReg(REG_LNA, RF_LNA_ZIN_200 | RF_LNA_GAINSELECT_MAXMINUS12);

  // Set threshold and demodulator control. See the RFM69 data sheet, section 3.4.12. OOK Demodulator
  // - rfm69_write(RegOokPeak, RegOokThreshPeak | RegOokThreshPeakStep0d5 | RegOokThreshPeakDec1c);
  // I guess the above settings mean 0.5 dB decrement, period once per chip.
  // This is already set in radio.initialize(), CONFIG[5] sets RF_OOKPEAK_THRESHTYPE_PEAK | RF_OOKPEAK_PEAKTHRESHSTEP_000 | RF_OOKPEAK_PEAKTHRESHDEC_000

  // If we were to use RF_OOKPEAK_THRESHTYPE_FIXED, then we'd set one of these:
  // radio.setFixedThreshold(30);
  // radio.setFixedThreshold(40);
  // radio.setFixedThreshold(50);
  // radio.setFixedThreshold(55);
  radio.setFixedThreshold(60);

  // Apparently we don't need sensitivity boost
  // radio.setSensitivityBoost(SENSITIVITY_BOOST_HIGH);

  // Parameters all set, go back into operation
  // - rfm69_write(RegOpMode, RegOpModeRX);
  radio.receiveBegin();

  // Reset automatic frequency correction
  // - rfm69_write(RegAfcFei, RegAfcFeiAfcClear);
  radio.writeReg(REG_AFCFEI, RF_AFCFEI_AFC_CLEAR);
}
 
void setup() {
  Serial.begin(115200);

  // Perform basic RFM69OOK set up
  radio.initialize();

  // Customize set up for AcuRite transmitter
  setAcuRiteRx();

  Serial.println(F("start"));
}

void loop() {
  unsigned long t;
  unsigned long d;
  unsigned long bitval;
  bool valid;
  byte s;
  uint16_t d_hi, d_lo;
  
  if (restart) {
    while (true) {
      if (radio.poll()) {
        break;
      }
    }
    t0 = micros();
    s0 = 1;
    pos = 0;
    restart = false;
    return;
  }

  s = radio.poll() ? 1 : 0;
  t = micros();
  d = t - t0;

  if (s0 != s) {
    if (s == 0 && d < MIN_1_DUR) {
      restart = true;
      return;
    }

    // Store in buffer
    tt[pos++] = d;
    t0 = t;
    s0 = s;
  }
  
  if (pos >= TSIZE || s0 == 0 && d > MAX_0_DUR) {
    s = 1;
    bitval = 0;
    
    // Require at least 76 bits (151 transitions)
    valid = pos > 2*(MESSAGE_LEN-1);
    if (valid) {
      for (int i = 0; i < MESSAGE_LEN; i++) {
        d_hi = tt[2*i];
        d_lo = 0;
        if ((2*i + 1) < pos) {
          d_lo = tt[2*i + 1];
        }
        // Deterimine bit value: LO, HI, or SYNC
        int c = d_hi > HI_LO_THRESHOLD ? (d_lo < HI_LO_THRESHOLD ? BIT_HI : BIT_SYNC) : BIT_LO;
        
        // Message is 76 bits long
        //   bits 0-7 are checksum
        //   bits 8-11 are LONG sync bits (600 usecs hi, 600 usecs lo)
        //   bits 12-35 are channel bits
        //   bits 36-67 are data bits (4 7-bit words)
        //   bits 68-75 are checksum (repeated)
        
        // Check sync bits
        if (valid && i >= 8 && i < 12) {
          if (c != BIT_SYNC) {
            valid = false;
          }
        }
        
        // Build data
        if (valid && i >= 36 && i < 68) {
          // Highest bit of each 8-bit word in data bits is a 
          // check bit - ignore it. Otherwise, shift value
          // add add bit.
          if ((i - 36) % 8 != 0) {
            bitval *= 2;
            if (c == BIT_HI) {
              bitval += 1;
            }
          }
        }
      }
    }
      
    Serial.print("Time: ");
    Serial.println(t0 / 1000000.0, 6);
    if (valid) {
      Serial.print("Value: ");
      Serial.println(bitval);
    } else {
      Serial.println("Invalid message");
    }

    t0_last = 0;
    restart = true;
  }
}

