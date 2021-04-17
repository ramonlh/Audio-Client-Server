/*
  Servidor UDP que envía audio de un MIC I2S
  Se envía y recibe el flujo I2S sin ninguna conversión
*/

#include <WiFi.h>
#include "driver/i2s.h"
#include "I2s_SettingRX.h"
#include "AsyncUDP.h"

const char* ssid = "conuco4";
const char* password = "18921892";

const int recPort = 3333;
const int sendPort = 3334;

AsyncUDP udprec;
AsyncUDP udpsend;

uint8_t sendbuff[BUFFLEN];

TaskHandle_t timer1_TaskHandler = NULL;
TaskHandle_t rec_audio_TaskHandler = NULL;
TaskHandle_t send_audio_TaskHandler = NULL;

unsigned long totalrec=0;
unsigned long totalsent=0;
unsigned long speedrec=0;
unsigned long speedsent=0;
unsigned long tini=0;
byte scaled=0;

static void timer1_task(void *arg)
{
  while (1)
    {
    Serial.print("Rec /Sent: "); Serial.print(speedrec);
    Serial.print(" / "); Serial.println(speedsent);
    speedrec=0; speedsent=0; 
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
    }
}

static void rec_audio_task(void *arg)
{
  if(udprec.listen(recPort)) {
      Serial.print("UDP Listening on: "); Serial.print(WiFi.localIP());
      Serial.print(":"); Serial.println(recPort);
      udprec.onPacket([](AsyncUDPPacket packet) {
          speedrec = speedrec + packet.length();
          totalrec = totalrec + packet.length();
          i2s_write_bytes(I2S_PORT_TX, packet.data(), packet.length(), portMAX_DELAY);
      });
  }
  while(1)
    {
    delay(1);
    }
}

static void send_audio_task(void *arg)
{
  size_t bytes_read=0;
  if(udpsend.connect(IPAddress(192,168,1,101), sendPort)) {
    Serial.print("UDP connected to: "); Serial.print(IPAddress(192,168,1,101));
    Serial.print(":"); Serial.println(sendPort);
  }   
  while (1)
    {
    i2s_read(I2S_PORT_RX, sendbuff, BUFFLEN, &bytes_read, portMAX_DELAY);
    if (bytes_read>0)
      {
      udpsend.write (sendbuff, bytes_read);
      speedsent=speedsent+bytes_read;
      totalsent=totalsent+bytes_read;
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
  xTaskCreate(rec_audio_task, "rec_audio", 10000, NULL, 1, &rec_audio_TaskHandler);
  xTaskCreate(send_audio_task, "send_audio", 10000, NULL, 1, &send_audio_TaskHandler);
}

void loop() 
{
}

