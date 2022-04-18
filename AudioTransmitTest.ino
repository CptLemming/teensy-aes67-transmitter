#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <QNEthernet.h>
#include <TeensyID.h>
#include <queue>

using namespace qindesign::network;

#define SDCARD_CS_PIN    10
#define SDCARD_MOSI_PIN  7
#define SDCARD_SCK_PIN   14

AudioInputUSB            usb1;
AudioPlaySdWav           playWav1;
AudioSynthWaveform       waveform1;
AudioRecordQueue         recorder;
AudioPlayQueue           player;
AudioOutputI2S           i2s1;
AudioConnection          patchCord1(usb1, 0, recorder, 0);
//AudioConnection          patchCord2(player, 0, i2s1, 0);
//AudioConnection          patchCord3(player, 0, i2s1, 1);
AudioControlSGTL5000     sgtl5000_1;

IntervalTimer rtpOutputTimer;

IPAddress staticIP{192, 168, 30, 211};
IPAddress subnetMask{255, 255, 255, 0};
IPAddress gateway{192, 168, 30, 1};

IPAddress mcastip{239, 34, 13, 86}; // RPI
int mport = 5004;

EthernetUDP udp;

byte mac[6];

const unsigned short packetSize = 100;
int outputBufferReaderIndex = 0;
int outputBufferWriterIndex = 0;
int numOutputBuffers = 48;
int numSamples = 44;
uint16_t outputBuffer[48][50]; // 50 = 16 bits, 100 = 8 bits
int bufferIndex = 0;
unsigned short rtpPayloadType = 32864;
unsigned short sequenceNo = 0;
unsigned long ssrc = 1649937450;
unsigned long timestamp = 0;
unsigned long lastTimestamp = 0;

std::queue <int> _OutputQueue;

void setup(void) {
  teensyMAC(mac);

  Serial.begin(115200);

  AudioMemory(512);
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.5);

  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if (!(SD.begin(SDCARD_CS_PIN))) {
    // stop here, but print a message repetitively
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }

  stdPrint = &Serial;  // Make printf work (a QNEthernet feature)
  printf("Starting...\n");

  Ethernet.macAddress(mac);
  printf("MAC = %02x:%02x:%02x:%02x:%02x:%02x\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Ethernet.onLinkState([](bool state) {
    printf("[Ethernet] Link %s\n", state ? "ON" : "OFF");
  });
  printf("Starting Ethernet with static IP...\n");
  Ethernet.begin(staticIP, subnetMask, gateway);
  Serial.println(Ethernet.localIP());

  waveform1.pulseWidth(0.5);
  waveform1.begin(0.4, 220, WAVEFORM_PULSE);

  recorder.begin();

//  player.setBehaviour(AudioPlayQueue::NON_STALLING);
//  player.setMaxBuffers(32);

  udp.beginMulticast(mcastip, mport);

  rtpOutputTimer.begin(sendRTPData, 1000);  // run every 0.001 seconds

  printf("Started! %d\n", AUDIO_BLOCK_SAMPLES);
  lastTimestamp = millis();
}

// SAMPLES ==
//const packetOne = [0x80, 0x60, 0x65, 0x94, 0xfb, 0x20, 0x96, 0x35, 0xcc, 0x38, 0xa5, 0x55];
//const packetTwo = [0x80, 0x60, 0x65, 0x95, 0xfb, 0x20, 0x96, 0x61, 0xcc, 0x38, 0xa5, 0x55];

// RTP HEADER (big endian) = 12
// 16bit Version + RTP type = (1 << 15) + 9 = 32864
// 16bit sequence number = 0 - 65535
// 32bit timestamp
// 32bit ssrc = 1649937450
void sendRTPData() {
  if (_OutputQueue.size() <= 0) return;

  int bufferIndex = _OutputQueue.front();
  _OutputQueue.pop();

  timestamp = millis();
  outputBuffer[bufferIndex][0] = rtpPayloadType;
  outputBuffer[bufferIndex][1] = sequenceNo;
  outputBuffer[bufferIndex][2] = timestamp >> 16;
  outputBuffer[bufferIndex][3] = 0;
  outputBuffer[bufferIndex][4] = ssrc >> 16;
  outputBuffer[bufferIndex][5] = 0;

  uint8_t writeBuffer[100];

  // This uses big endian, which is wrong
  // memcpy(writeBuffer, outputBuffer[bufferIndex], packetSize);
  // So we swap the byte order manually
  int i = 0;
  for (i = 0; i < 50; i++) {
    writeBuffer[(i * 2) + 1] = (outputBuffer[bufferIndex][i] >> 0) & 0xFF;
    writeBuffer[(i * 2)] = (outputBuffer[bufferIndex][i] >> 8) & 0xFF;
  }

  Serial.print("Seq: ");
  Serial.print(sequenceNo);
  Serial.print("    Act: ");
  Serial.print((writeBuffer[2] << 8) | (writeBuffer[3] & 0xff));
  Serial.print("    Index: ");
  Serial.print(bufferIndex);
  Serial.print("    Time: ");
  Serial.println(timestamp - lastTimestamp);

  sequenceNo++;

  udp.beginPacket(mcastip, mport);
  udp.write(writeBuffer, packetSize);
  udp.endPacket();

//  outputBufferReaderIndex++;
//
//  if (outputBufferReaderIndex >= numOutputBuffers) outputBufferReaderIndex = 0;
}

int i = 0;
int inByte = 0;
void loop() {
  if (Serial.available() > 0) {
    inByte = Serial.read();

    Serial.println("Playing file: SDTEST3.WAV");
    playWav1.play("SDTEST3.WAV");
  }
  if (recorder.available() >= 2) {
    int16_t *recorderBuffer = recorder.readBuffer();
//    timestamp = millis();
//    Serial.print("Buffer full - ");
//    Serial.print(recorder.available());
//    Serial.print(" - ");
//    Serial.print(sizeof(recorderBuffer));
//    Serial.print(" - time: ");
//    Serial.println(timestamp);

    for(i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
      outputBuffer[outputBufferWriterIndex][bufferIndex + 6] = recorderBuffer[i];

      bufferIndex++;
      if (bufferIndex >= numSamples) {
        _OutputQueue.push(outputBufferWriterIndex);
        bufferIndex = 0;
        outputBufferWriterIndex++;
      }
      if (outputBufferWriterIndex >= numOutputBuffers) {
        outputBufferWriterIndex = 0;
      }
    }

    recorder.freeBuffer();
  }

//  player.playBuffer();
}
