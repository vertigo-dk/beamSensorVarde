#include <Ethernet.h>
#include "TeensyMAC.h"

EthernetUDP udp;
uint8_t udp_buffer[100];

IPAddress ip;
uint8_t mac[6];

int udpPort = 50000;
int outPort = 50001;

int beam_break_stat = 1;
#define beam_break_pin 23

#define PIN_RESET 9

void setup() {

  #ifdef PIN_RESET
  pinMode(PIN_RESET, OUTPUT);
  digitalWrite(PIN_RESET, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_RESET, HIGH);
  delay(150);
  #endif

  pinMode(beam_break_pin, INPUT_PULLUP);
  mac_addr macGet;
  for (int i = 3; i < 6; i++) {
    mac[i] = macGet.m[i];
  }

    // Calculate IP address
  ip[0] = 2;
  ip[1] = mac[3];// + ((oemCode >> 16) & 0xFF);
  ip[2] = mac[4];
  ip[3] = mac[5];

  IPAddress gateway(ip[0], 0, 0, 1);
  IPAddress subnet(255, 0, 0, 0);

  Ethernet.begin(mac, ip,  gateway, gateway, subnet);
  udp.begin(udpPort);

}

void loop() {
  if(digitalRead(beam_break_pin) != beam_break_stat) {
    
    beam_break_stat = digitalRead(beam_break_pin);

    //char addr[15];
    //oscAddr.toCharArray(addr, 15);
    char oscStr[23] = {0x2f, 0x42, 0x65, 0x61, 0x6d, 0x42, 0x72, 0x65, 0x61, 0x6b, 0x2f, 0x30, 0x30, 0x30, 0x30, 0x00, 0x2c, 0x69, 0x00, 0x00, 0x00, 0x00, 0x00};

    char dig3;
    char dig2;
    char dig1;
    char dig0;
    int addrINT = 2;    //| syd -> 1 | nord ->2 |
    dig3 = addrINT/1000;
    dig2 = (addrINT-(dig3*1000))/100;
    dig1 = (addrINT-(dig3*1000)-(dig2*100))/10;
    dig0 = addrINT-(dig3*1000)-(dig2*100)-(dig1*10);

    oscStr[11] = dig3 + 0x30;
    oscStr[12] = dig2 + 0x30;
    oscStr[13] = dig1 + 0x30;
    oscStr[14] = dig0 + 0x30;
    //memcpy(&oscStr+11, addrCHAR, 3);
    udp.beginPacket(IPAddress(2, 0, 0, 1), outPort);
    udp.write(oscStr, 23);
    udp.write(beam_break_stat);
    udp.endPacket();
    /*OSCMessage msg(addr);
    msg.add(beam_break_stat);
    udp.beginPacket(IPAddress(2, 0, 0, 1), OSCoutPort);
    msg.send(udp);
    udp.endPacket();
    msg.empty();*/
  }

}
