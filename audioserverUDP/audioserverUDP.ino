/*
  Servidor UDP que envía audio de un MIC I2S
  Se envía y recibe el flujo I2S sin ninguna conversión
*/

//// FFT  ////////////////////////////
//#define FFT_N       2048  // Must be a power of 2
#define FFT_N       1024  // igual al buffer del I2S
#define SAMPLEFREQ  218   //
#define TOTAL_TIME  9.391904 //The time in which data was captured. This is equal to FFT_N/sampling_freq
float fft_input[FFT_N];
float fft_output[FFT_N];
#include "ESP32_fft.h" // include the library
#include "fft_signal.h"
ESP_fft FFT(FFT_N, SAMPLEFREQ, FFT_REAL, FFT_FORWARD, fft_input, fft_output);


#include <WiFi.h>
#include "driver/i2s.h"
#include "I2s_SettingRX.h"
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <RTPPacket.h>

#define WIFI_SSID "elconuco"
#define WIFI_PASSWORD "18921892"
#define CALLSIGN "EA4GZI"
#define UDP_PORT 3334  
#define BROKER_URI "broker.mqtt-dashboard.com"
#define HOST_MYIP "icanhazip.com"
#define TOPIC_ROOT "ubitx/audioserver/"

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
char callsign[10] = CALLSIGN;
const int udpPort = UDP_PORT;
char mqttserver[40] = BROKER_URI;           // MQTT broker
char hostmyip[20] = HOST_MYIP;  // URL del servidor de IP pública
char mqtttopicaudiobase[50] = TOPIC_ROOT;
char mqtttopicaudioip[50] = TOPIC_ROOT;
char mqtttopicaudioport[50] = TOPIC_ROOT;
char mqtttopicaudiorate[50] = TOPIC_ROOT;
char mqtttopicaudiobits[50] = TOPIC_ROOT;
char myippub[16]="";                // 16 bytes, dirección IP pública

WiFiUDP udpsocket;
WiFiClient mqttClient;
PubSubClient PSclient(mqttClient);
RTPPacket rtp;

TaskHandle_t timer1_TaskHandler = NULL;
TaskHandle_t timer60_TaskHandler = NULL;
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

void mqttcallback(char* topic, uint8_t* payload, unsigned int length) 
{
  String msg="";
  for (byte j=0; j<length;j++) msg+=(char)payload[j];
  if (strstr(topic,"/RST")!=NULL) {  ESP.restart(); }  // recibido RST, resetear
}

void mqttsubscribe(char *topic)
{
  if (!WiFi.isConnected()) return;
  PSclient.subscribe(topic);   // topic
}

void mqttpublish(char *topic, char *payload)
{
  if (!WiFi.isConnected()) return;
  PSclient.publish(topic, payload,true);   // topic, payload
}

void mqttreconnect() 
  { 
  while (!PSclient.connected()) 
    {
    String clientID="ubitx-";
    clientID += String(random(0xffff), HEX);
    if (PSclient.connect(clientID.c_str())) 
      { 
      mqttsubscribe(mqtttopicaudioip);
      mqttsubscribe(mqtttopicaudioport); 
      mqttsubscribe(mqtttopicaudiorate); 
      mqttsubscribe(mqtttopicaudiobits); 
      }
    else
      {
      }
    }
  }

void handlePubSub()
{
  if (WiFi.isConnected()) 
    {
      {
      if (!PSclient.connected()) { mqttreconnect(); }
      }
    PSclient.loop();
    }
}

int getMyIP()
{
  if (!WiFi.isConnected()) return 0;
  String msg="/";
  HTTPClient http;
  http.begin(hostmyip, 80, msg);
  http.setConnectTimeout(1000);
  int httpCode=http.GET();
  if (httpCode > 0) {
    if (httpCode == HTTP_CODE_OK) { msg=http.getString(); msg.toCharArray(myippub, msg.length());  } }
  http.end();
  return httpCode;
}

static void timer1_task(void *arg)
{
  while (1)
    {
    if ((speedrec>0) || (speedsent>0)) 
      {
      Serial.print("Client: "); Serial.print(remoteIP);
      Serial.print(":"); Serial.print(remotePort);
      Serial.print(" Rec/Sent: "); Serial.print(speedrec/logtime);
      Serial.print("/"); Serial.println(speedsent/logtime);
      }
    speedrec=0; speedsent=0; 
    vTaskDelay(logtime*1000 / portTICK_PERIOD_MS); 
    }
}

static void timer60_task(void *arg)
{
  while (1)
    {
    getMyIP();
    char buff[10];
    mqttpublish(mqtttopicaudioip, myippub);
    mqttpublish(mqtttopicaudioport, itoa(udpPort,buff,10));
    mqttpublish(mqtttopicaudiorate, itoa(I2S_SAMPLE_RATE,buff,10));
    mqttpublish(mqtttopicaudiobits, itoa(I2S_SAMPLE_BITS,buff,10));
    vTaskDelay(logtime*60000 / portTICK_PERIOD_MS); 
    }
}

void handleFFT()
{
  long int t1 = micros();
  // Execute transformation
  FFT.removeDC();
  FFT.hammingWindow();
  FFT.execute();
  FFT.complexToMagnitude();
  long int t2 = micros();

  //FFT.print();
  Serial.print("Time taken: ");Serial.print((t2-t1)*1.0/1000);Serial.println(" milliseconds!");
  Serial.println();
  //Multiply the magnitude of the DC component with (1/FFT_N) to obtain the DC component
  Serial.printf("DC component : %f g\n", (fft_output[0])/FFT_N);  // DC is at [0]
  //Multiply the magnitude at all other frequencies with (2/FFT_N) to obtain the amplitude at that frequency
  Serial.printf("Fundamental Freq : %f Hz\t Mag: %f g\n", FFT.majorPeakFreq(), (FFT.majorPeak()/10000)*2/FFT_N);
  for (int i=0; i< 10; i++) {
    Serial.printf("%f:%f\n", FFT.frequency(i),fft_output[i]);
  }
  //Serial.print("\nTime taken: ");Serial.print((t2-t1)*1.0/1000);Serial.println(" milliseconds!");
}

static void audio_task(void *arg)
{
  uint8_t udpbuff[BUFFLEN];
  uint8_t rtpheader[12];
  size_t bytes_read=0;
  udpsocket.begin(udpPort);
  while(1)
    {
    //  lee UDP y escribe en I2S
    int packetSize = udpsocket.parsePacket();
    if (packetSize>0) {
      remoteIP = udpsocket.remoteIP();
      remotePort=udpsocket.remotePort();
      clientexists=true;
      int leidos=udpsocket.read(udpbuff, BUFFLEN);
      if (leidos > 0)
        i2s_write_bytes(I2S_PORT_TX, udpbuff, leidos, portMAX_DELAY);
      speedrec=speedrec+leidos;
      totalrec=totalrec+leidos;
      }
    // lee I2S y envía por UDP
    i2s_read(I2S_PORT_RX, udpbuff, BUFFLEN, &bytes_read, portMAX_DELAY);
    
    // FFT
    //for (int k = 0 ; k < FFT_N ; k++) fft_input[k] = (float)udpbuff[k];
    //handleFFT();
    // FFT end
    
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

char ssidAP[20]="UBITXAudio";     // 20 bytes, SSID en modo AP
char passAP[20]="12341234";       // 20 bytes, Password en modo AP
uint8_t canalAP=3;                  // 1 byte, canal ESP en modo AP
IPAddress EEip={192,168,1,148};   // 4 bytes, dirección IP
IPAddress EEgw{192,168,1,1};     // 4 bytes, puerta de enlace
IPAddress EEmask{255,255,255,0}; // 4 bytes, máscara de subred
IPAddress EEdns{8,8,8,8};        // 4 bytes, servidor DNS primario
IPAddress EEdns2{8,8,4,4};       // 4 bytes, servidor DNS secundario

void connectWiFi()
{
  WiFi.mode(WIFI_AP_STA);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  WiFi.channel(canalAP);
  WiFi.softAP(ssidAP, passAP, canalAP, false, 2);
  WiFi.config(EEip, EEgw, EEmask, EEdns, EEdns2); 
  WiFi.begin(ssid, password);
  Serial.print("- WiFi Connecting to "); Serial.print(ssid);
  Serial.print("/"); Serial.println(password);
  int count=0;  
  while (WiFi.status() != WL_CONNECTED) 
    { delay(500); Serial.print("."); count++; if (count>20) ESP.restart(); }
  Serial.print("+ WiFi Connected: ");  Serial.println(WiFi.localIP());
}

void initPubSub()
{
  PSclient.setServer(mqttserver, 1883); 
  PSclient.setCallback(mqttcallback); 
  strcat(mqtttopicaudioip,callsign);  strcat(mqtttopicaudioip,"/ip");
  strcat(mqtttopicaudioport,callsign);  strcat(mqtttopicaudioport,"/port");
  strcat(mqtttopicaudiorate,callsign);  strcat(mqtttopicaudiorate,"/rate");
  strcat(mqtttopicaudiobits,callsign);  strcat(mqtttopicaudiobits,"/bits");
  
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
  initPubSub();
  xTaskCreate(timer1_task, "timer1", 2048, NULL, 1, &timer1_TaskHandler);
  xTaskCreate(timer60_task, "timer60", 2048, NULL, 1, &timer60_TaskHandler);
  xTaskCreate(audio_task, "audio", 10000, NULL, 1, &audio_TaskHandler);
}

void loop() 
{
  handlePubSub();
  //handleFFT();
}

