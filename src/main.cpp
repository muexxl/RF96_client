// rf95_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf95_server
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with
// the RFM95W, Adafruit Feather M0 with RFM95

#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <mxsupport.h>

#define FREQUENCY 868.0
#define SERIAL_SPEED 115200
#define SPREADINGFACTOR 6 // 6..12

#define REG_Detection_Optimize 0x31
#define REG_Detect_Threshold 0x37


// Singleton instance of the radio driver
RH_RF95 rf95;
//RH_RF95 rf95(5, 2); // Rocket Scream Mini Ultra Pro with the RFM95W
//RH_RF95 rf95(8, 3); // Adafruit Feather M0 with RFM95

// Need this on Arduino Zero with SerialUSB port (eg RocketScream Mini Ultra Pro)
//#define Serial SerialUSB

int bytes_rcvd{0};
int msg_rcvd{0};
unsigned long time1{millis()};
uint8_t last_counter{0};
uint16_t lost_messages{0};
bool reportingDue{false};
void print_report();

void handle_message(void *msg, int len);
void set_SF6();
void set_implicit_Header();
void set_detection_optimize();
void set_detection_threshold();
void print_register(uint8_t reg);
void setup()
{
  // Rocket Scream Mini Ultra Pro with the RFM95W only:
  // Ensure serial flash is not interfering with radio communication on SPI bus
  //  pinMode(4, OUTPUT);
  //  digitalWrite(4, HIGH);

  Serial.begin(SERIAL_SPEED);
  while (!Serial)
    ; // Wait for serial port to be available
  while (!rf95.init())
  {
    Serial.println("init failed");
    delay(1000);
  }
  Serial.println("init successful");

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // You can change the modulation parameters with eg
  // rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 2 to 20 dBm:
  //  rf95.setTxPower(20, false);
  // If you are using Modtronix inAir4 or inAir9, or any other module which uses the
  // transmitter RFO pins and not the PA_BOOST pins
  // then you can configure the power transmitter power for 0 to 15 dBm and with useRFO true.
  // Failure to do that will result in extremely low transmit powers.
  //  rf95.setTxPower(14, true);

  setupTimer1(2000);
  rf95.setModemConfig(rf95.Bw500Cr45Sf128);
  rf95.setFrequency(FREQUENCY);

   rf95.setModemConfig(rf95.Bw500Cr45Sf128);
  //rf95.setSpreadingFactor(SPREADINGFACTOR);
  // implicit header mode
  //rf95.spiWrite(0x1d, ((rf95.spiRead(0x1d) & 0xfe )| 0x01));

  rf95.setFrequency(FREQUENCY);
  
  Serial.print(F("BW500 configuration\n"));
  print_register(RH_RF95_REG_1D_MODEM_CONFIG1);
  print_register(RH_RF95_REG_1E_MODEM_CONFIG2);
  print_register(RH_RF95_REG_26_MODEM_CONFIG3);
  print_register(RH_RF95_REG_31_DETECT_OPTIMIZE);
  print_register(RH_RF95_REG_37_DETECTION_THRESHOLD);

  Serial.print(F("Changing to SF6\n"));
  set_SF6();
  print_register(RH_RF95_REG_1D_MODEM_CONFIG1);
  print_register(RH_RF95_REG_1E_MODEM_CONFIG2);
  print_register(RH_RF95_REG_26_MODEM_CONFIG3);
  print_register(RH_RF95_REG_31_DETECT_OPTIMIZE);
  print_register(RH_RF95_REG_37_DETECTION_THRESHOLD);

  Serial.print(F("Setting implicit Header\n"));
  set_implicit_Header();
  print_register(RH_RF95_REG_1D_MODEM_CONFIG1);
  print_register(RH_RF95_REG_1E_MODEM_CONFIG2);
  print_register(RH_RF95_REG_26_MODEM_CONFIG3);
  print_register(RH_RF95_REG_31_DETECT_OPTIMIZE);
  print_register(RH_RF95_REG_37_DETECTION_THRESHOLD);

  Serial.print(F("Setting detection optimize\n"));
  set_detection_optimize();
  print_register(RH_RF95_REG_1D_MODEM_CONFIG1);
  print_register(RH_RF95_REG_1E_MODEM_CONFIG2);
  print_register(RH_RF95_REG_26_MODEM_CONFIG3);
  print_register(RH_RF95_REG_31_DETECT_OPTIMIZE);
  print_register(RH_RF95_REG_37_DETECTION_THRESHOLD);

  Serial.print(F("Setting detection Threshold\n"));
  set_detection_threshold();
  print_register(RH_RF95_REG_1D_MODEM_CONFIG1);
  print_register(RH_RF95_REG_1E_MODEM_CONFIG2);
  print_register(RH_RF95_REG_26_MODEM_CONFIG3);
  print_register(RH_RF95_REG_31_DETECT_OPTIMIZE);
  print_register(RH_RF95_REG_37_DETECTION_THRESHOLD);


}

void loop()
{
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(3000))
  {
    // Should be a reply message for us now
    if (rf95.recv(buf, &len))
    {
      //Serial.println("got Message: ");
      handle_message(&buf, len);
      //printObject(&buf, len);
      //Serial.print("RSSI: ");
      //Serial.println(rf95.lastRssi(), DEC);
    }
    else
    {
      Serial.println("recv failed");
    }
  }
  else
  {
    Serial.println("No Message :/ ");
  }

  if (reportingDue)
  {
    print_report();
  }
  delay(0);
}

void handle_message(void *msg, int len)
{
  bytes_rcvd += len;
  ++msg_rcvd;
  uint8_t counter = *reinterpret_cast<uint8_t *>(msg);
  if (counter == last_counter + 1)
  {
    ;
  }
  else
  {
    ++lost_messages;
  }
  last_counter = counter;
}

void print_report()
{
  unsigned long dT = millis() - time1;
  float bps{0};
  time1 = millis();
  reportingDue = false;
  bps = (bytes_rcvd * 1000.0/dT) ;

  Serial.print("Bytes received: ");
  Serial.print(bytes_rcvd);
  Serial.print("\tdT: ");
  Serial.print(dT);
  Serial.print("\tBPS: ");
  Serial.print(bps);
  Serial.print("\treceived: ");
  Serial.print(msg_rcvd);
  Serial.print("\tlost: ");
  Serial.print(lost_messages);
  Serial.print("\n");


  msg_rcvd = 0;
  lost_messages = 0;
  bytes_rcvd = 0;
}

ISR(TIMER1_COMPA_vect)
{
  //Serial.println("This is TIMER1 ISR A\n");
  reportingDue = true;
}



void set_SF6(){
  rf95.setSpreadingFactor(6);
}

void set_implicit_Header(){
  rf95.spiWrite(RH_RF95_REG_1D_MODEM_CONFIG1, ((rf95.spiRead(RH_RF95_REG_1D_MODEM_CONFIG1) & 0xfe )| 0x01));
}

void set_detection_optimize(){
  rf95.spiWrite(REG_Detection_Optimize, ((rf95.spiRead(REG_Detection_Optimize) & 0xf8 )| 0x05)); // 0x05 = 0b101
}

void set_detection_threshold(){
  rf95.spiWrite(REG_Detect_Threshold, 0x0C); // 0x0C for SF6 , 0x0A for SF7..12
}

void print_register(uint8_t reg){
  Serial.print("Register 0x");
  Serial.print(reg, HEX);
  Serial.print("\t 0x");
  Serial.print(rf95.spiRead(reg), HEX);
  Serial.print("\n");
}
