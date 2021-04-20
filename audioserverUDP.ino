/*
  Servidor UDP que envía audio de un MIC I2S
  Se envía y recibe el flujo I2S sin ninguna conversión
*/

#include <WiFi.h>
#include "driver/i2s.h"
#include "I2s_SettingRX.h"
#include <WiFiUdp.h>
#include <RTPPacket.h>

const char* ssid = "conuco4";
const char* password = "18921892";

const int udpPort = 3334;

WiFiUDP udpsocket;

TaskHandle_t timer1_TaskHandler = NULL;
TaskHandle_t audio_TaskHandler = NULL;

unsigned long totalrec=0;
unsigned long totalsent=0;
unsigned long speedrec=0;
unsigned long speedsent=0;
unsigned long tini=0;
byte scaled=0;
IPAddress remoteIP=(0,0,0,0);
int remotePort=0;
boolean clientexists=false;
int logtime=1;

static void timer1_task(void *arg)
{
  while (1)
    {
    Serial.print("Client: "); Serial.print(remoteIP);
    Serial.print(":"); Serial.print(remotePort);
    Serial.print(" Rec/Sent: "); Serial.print(speedrec/logtime);
    Serial.print("/"); Serial.println(speedsent/logtime);
    speedrec=0; speedsent=0; 
    vTaskDelay(logtime*1000 / portTICK_PERIOD_MS); 
    }
}

RTPPacket rtp;

static void audio_task(void *arg)
{
  uint8_t udpbuff[BUFFLEN];
  uint8_t rtpheader[12];
  size_t bytes_read=0;
  udpsocket.begin(udpPort);
  while(1)
    {
    int packetSize = udpsocket.parsePacket();
    if (packetSize>0) {
      remoteIP = udpsocket.remoteIP();
      remotePort=udpsocket.remotePort();
      clientexists=true;
      int leidos=udpsocket.read(udpbuff, BUFFLEN);
      if (leidos > 0)
          i2s_write_bytes(I2S_PORT_TX, udpbuff+12, leidos-12, portMAX_DELAY);
      speedrec=speedrec+leidos;
      totalrec=totalrec+leidos;
      }
    //////////////////////////////////// 
    i2s_read(I2S_PORT_RX, udpbuff+12, BUFFLEN-12, &bytes_read, portMAX_DELAY);
    if (clientexists)
      {
      if (bytes_read>0)
        {
        udpsocket.beginPacket(remoteIP,remotePort);
        udpsocket.write(udpbuff, BUFFLEN);
        udpsocket.endPacket();
        speedsent=speedsent+bytes_read;
        totalsent=totalsent+bytes_read;
        }
      }
    }
}

void connectWiFi()
{
  WiFi.begin(ssid, password);
  Serial.println("- WiFi Connecting");
  int count=0;  
  while (WiFi.status() != WL_CONNECTED) 
    { delay(500); Serial.print("."); count++; if (count>20) ESP.restart(); }
  Serial.print("+ WiFi Connected: ");  Serial.println(WiFi.localIP());
}

void setMCLK()   // activar MCLK para PCM1808
{
  REG_WRITE(PIN_CTRL, 0xFF0); 
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32  Servidor audio");
  connectWiFi();
  setMCLK();
  i2s_RX_init();
  i2s_TX_init();
  xTaskCreate(timer1_task, "timer1", 4096, NULL, 1, &timer1_TaskHandler);
  xTaskCreate(audio_task, "audio", 10000, NULL, 1, &audio_TaskHandler);
}

void loop() 
{
}

