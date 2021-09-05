/*	Cliente de audio
	Recibe y envía sonido vía UDP 
	Envía y recibe el flujo I2S sin ningún tipo de conversión
	Funciona con la tarjeta ESP32-LyraT
	Se conecta a un servidor UDP, entonces empieza a recibir y enviar audio
*/

#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "lwip/sockets.h"

#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "tcp_client_stream.h"
#include "i2s_stream.h"

#include "esp_peripherals.h"
#include "periph_wifi.h"
#include "periph_sdcard.h"
#include "board.h"
#include "es8388.h"
#include "esp_netif.h"
#include "input_key_service.h"
#include "mqtt_client.h"

#define I2S_PORT I2S_NUM_0
#define BITS_SAMPLE (24)
#define BUFFLEN (1024)
#define BROKER_URI "mqtt://broker.mqtt-dashboard.com"
#define HOST_MYIP "icanhazip.com"

static const char *TAG = "audioclientUDP";
static char callsign[32] = "callsign";
static char wifi_ssid[32] = "yourssid";
static char wifi_pass[32] = "yourpass";
static uint8_t modoaudio = 1;		// 1: mono, 2: stereo
static uint8_t linein = 0;			// 0: aux,  1: mic
static uint16_t samplerate = 8000;
static uint8_t samplebits = 24;
audio_board_handle_t board_handle;
audio_element_handle_t i2s_stream_writer, i2s_stream_reader;
esp_periph_set_handle_t set;
audio_event_iface_handle_t evt;
int player_volume=50;

unsigned long totalrec=0;
unsigned long totalsent=0;
unsigned long speedrec=0;
unsigned long speedsent=0;
int logtime=1;

int udpport=0;
char udphost[20]="0000";

char mqtttopicaudioip[50]="ubitx/audioserver/";
char mqtttopicaudioport[50]="ubitx/audioserver/";
char mqtttopicaudiorate[50]="ubitx/audioserver/";
char mqtttopicaudiobits[50]="ubitx/audioserver/";

static void timer1(void *pvParameters)
{
	while(1) {
	if ((speedrec>0) || (speedsent>0)) 
		{
			ESP_LOGI(TAG, "Rec/Sent: %lu/%lu", speedrec/logtime, speedsent/logtime);
			speedrec=0;	speedsent=0;
		}
		vTaskDelay(logtime*1000 / portTICK_PERIOD_MS);
	}
}

static void audio_task(void *pvParameters)
{
    while (1) {
		uint8_t udpbuffer[BUFFLEN];
		int ip_protocol = IPPROTO_IP;
		int addr_family = AF_INET;

		// sender ////////////////////////////////
        struct sockaddr_in dest_addrS;
        dest_addrS.sin_family = AF_INET;
        dest_addrS.sin_port = htons(udpport);
        dest_addrS.sin_addr.s_addr = inet_addr(udphost);
		
		// receiver //////////////////////////////////
        struct sockaddr_in dest_addrR;
        dest_addrR.sin_family = AF_INET;
        dest_addrR.sin_port = htons(udpport);
        dest_addrR.sin_addr.s_addr = htonl(INADDR_ANY);

        int udpsocket = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (udpsocket < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(udpsocket, (struct sockaddr *)&dest_addrR, sizeof(dest_addrR));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", udpport);
		
		size_t leni2s=0;
		int lenudp=0;
        while (1) {
			// sender
			/////////////////// read I2S->Send UDP
			i2s_read(I2S_PORT, udpbuffer, BUFFLEN, &leni2s, portMAX_DELAY);	// read from I2S port
            lenudp = sendto(udpsocket, udpbuffer, leni2s, 0, (struct sockaddr *)&dest_addrS, sizeof(dest_addrS));
			speedsent=speedsent+lenudp; totalsent=totalsent+lenudp; 
            if (lenudp < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
			
			// receiver
            lenudp = recv(udpsocket, udpbuffer, sizeof(udpbuffer),0);
			speedrec=speedrec+lenudp; totalrec=totalrec+lenudp;
            if (lenudp < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            else {
				i2s_write(I2S_PORT, udpbuffer, lenudp, &leni2s, portMAX_DELAY);	// send to I2S port
            }
        }

        if (udpsocket != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(udpsocket, 0);
            close(udpsocket);
        }
    }
    vTaskDelete(NULL);
}

static int readConfig()
{
    static FILE *file;
    if (file == NULL) {
        file = fopen("/sdcard/config.txt", "r");
        if (!file) {
            ESP_LOGE(TAG, "Error opening file");
            return -1;
        }
    }
    char line[32];
    fgets(line, sizeof(line), file);
    char* pos = strchr(line, '\n'); if (pos) { *pos = '\0'; }
		  pos = strchr(line, '\r'); if (pos) { *pos = '\0'; }
	strcpy(callsign,line);
    fgets(line, sizeof(line), file);
		  pos = strchr(line, '\n'); if (pos) { *pos = '\0'; }
		  pos = strchr(line, '\r'); if (pos) { *pos = '\0'; }
	strcpy(wifi_ssid,line);
    fgets(line, sizeof(line), file);
		  pos = strchr(line, '\n'); if (pos) { *pos = '\0'; }
		  pos = strchr(line, '\r'); if (pos) { *pos = '\0'; }
	strcpy(wifi_pass,line);
    fgets(line, sizeof(line), file);
		  pos = strchr(line, '\n'); if (pos) { *pos = '\0'; }
		  pos = strchr(line, '\r'); if (pos) { *pos = '\0'; }
	if (strcmp(line,"MONO")==0) 
		modoaudio=1; 
	else 
		modoaudio=2;
    fgets(line, sizeof(line), file);
		  pos = strchr(line, '\n'); if (pos) { *pos = '\0'; }
		  pos = strchr(line, '\r'); if (pos) { *pos = '\0'; }
	samplerate = atoi(line);
    fgets(line, sizeof(line), file);
		  pos = strchr(line, '\n'); if (pos) { *pos = '\0'; }
		  pos = strchr(line, '\r'); if (pos) { *pos = '\0'; }
	if (strcmp(line,"AUX")==0) 
		linein=1; 
	else 
		linein=2;
    fclose(file);
    ESP_LOGI(TAG, "CALLSIGN/SSID/PASS/MODE/RATE: '%s/%s/%s/%d/%d'", callsign,wifi_ssid, wifi_pass,modoaudio,samplerate);
    return 0;
}

static esp_err_t input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx)
{
    board_handle = (audio_board_handle_t) ctx;
    audio_hal_get_volume(board_handle->audio_hal, &player_volume);
	if (evt->type == INPUT_KEY_SERVICE_ACTION_CLICK_RELEASE) {
        switch ((int)evt->data) {
            case INPUT_KEY_USER_ID_REC:				// 1
                ESP_LOGI(TAG, "[ KEY ] [Play] input key event (%d)",(int)evt->data );
                break;
            case INPUT_KEY_USER_ID_SET:				// 2
                ESP_LOGI(TAG, "[ KEY ] [Set] input key event (%d)",(int)evt->data );
				readConfig();
			    break;		
            case INPUT_KEY_USER_ID_PLAY:			// 3
				if ((udpport>0) && (strcmp(udphost,"0000")!=0)) {
					xTaskCreate(audio_task, "audio_task", 10000, (void*)AF_INET, 5, NULL);
				}
                ESP_LOGI(TAG, "[ KEY ] [Play] input key event (%d)",(int)evt->data );
                break;
            case INPUT_KEY_USER_ID_MODE:			// 4
                ESP_LOGI(TAG, "[ KEY ] [Play] input key event (%d)",(int)evt->data );
                break;
            case INPUT_KEY_USER_ID_VOLDOWN:			// 5
                ESP_LOGI(TAG, "[ KEY ] [Vol-] input key event (%d)",(int)evt->data );
                player_volume -= 10;
                if (player_volume < 0) { player_volume = 0; }
                audio_hal_set_volume(board_handle->audio_hal, player_volume);
                ESP_LOGI(TAG, "[ KEY ] Volume set to %d %%", player_volume);
                break;
            case INPUT_KEY_USER_ID_VOLUP:			// 6
                ESP_LOGI(TAG, "[ KEY ] [Vol+] input key event (%d)",(int)evt->data );
                player_volume += 10;
                if (player_volume > 100) { player_volume = 100; }
                audio_hal_set_volume(board_handle->audio_hal, player_volume);
                ESP_LOGI(TAG, "[ KEY ] Volume set to %d %%", player_volume);
                break;
            }
        }
    return ESP_OK;
}

static char *create_string(const char *ptr, int len)
{
    char *ret;
    if (len <= 0) {
        return NULL;
    }
    ret = calloc(1, len + 1);
    memcpy(ret, ptr, len);
    return ret;
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
	
    esp_mqtt_client_handle_t client = event->client;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            esp_mqtt_client_subscribe(client, mqtttopicaudioip, 0);
            ESP_LOGI(TAG, "sent subscribe successful, topic=%s", mqtttopicaudioip);
            esp_mqtt_client_subscribe(client, mqtttopicaudioport, 0);
            ESP_LOGI(TAG, "sent subscribe successful, topic=%s", mqtttopicaudioport);
            esp_mqtt_client_subscribe(client, mqtttopicaudiorate, 0);
            ESP_LOGI(TAG, "sent subscribe successful, topic=%s", mqtttopicaudiorate);
            esp_mqtt_client_subscribe(client, mqtttopicaudiobits, 0);
            ESP_LOGI(TAG, "sent subscribe successful, topic=%s", mqtttopicaudiobits);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            break;
        case MQTT_EVENT_PUBLISHED:
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
			char *auxtopic = create_string(event->topic, event->topic_len);
			char *auxdata = create_string(event->data, event->data_len);
			if (strcmp(auxtopic, mqtttopicaudioip)==0) {	// rec. IP 
				strcpy(udphost, auxdata); 
				ESP_LOGI(TAG, "udphost: %s",udphost);
			}
			else if (strcmp(auxtopic,mqtttopicaudioport)==0) {	// rec PORT
				udpport=atoi(auxdata);
				ESP_LOGI(TAG, "udpport: %d",udpport);
			}
			else if (strcmp(auxtopic,mqtttopicaudiorate)==0) {	// samplerate
				samplerate=atoi(auxdata);
				ESP_LOGI(TAG, "samplerate: %d",samplerate);
			}
			else if (strcmp(auxtopic,mqtttopicaudiobits)==0) {	// sample bits
				samplebits=atoi(auxdata);
				ESP_LOGI(TAG, "samplebits: %d",samplebits);
			}
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

void setupSD()
{
    ESP_LOGI(TAG, "[ 1 ] Mount sdcard");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);
    audio_board_sdcard_init(set, SD_MODE_1_LINE);
}

void setupLOG()
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
}

void setupWiFi()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
      }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
}

void initWiFi()
{
    ESP_LOGI(TAG, "[ WiFi ] Start and wait for Wi-Fi network");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);
    periph_wifi_cfg_t wifi_cfg = {
        .ssid = wifi_ssid,
        .password = wifi_pass,
    };
    ESP_LOGI(TAG, "[ 3.1 ] Connecting to %s/%s", wifi_ssid, wifi_pass);
    esp_periph_handle_t wifi_handle = periph_wifi_init(&wifi_cfg);
    esp_periph_start(set, wifi_handle);
    periph_wifi_wait_for_connected(wifi_handle, portMAX_DELAY);
    ESP_LOGI(TAG, "[ 3.2 ] Connected to %s",wifi_ssid ); 
}

void setupCODEC()
{
    ESP_LOGI(TAG, "[ CODEC ] Start codec chip");
    board_handle = audio_board_init();
	if (linein == 0)		// line aux
		{
		audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_LINE_IN, AUDIO_HAL_CTRL_START);
		esp_err_t ret = es8388_write_reg(ES8388_ADCCONTROL2, ADC_INPUT_LINPUT2_RINPUT2);
		if (ret != ESP_OK) { ESP_LOGI(TAG, "[ ES8388 ] ES8388 writereg error : %d", ret); }
		}
	else					// mic
		audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);
}

void setupI2S()
{
    ESP_LOGI(TAG, "[2.1] Create i2s stream to write data to codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
	i2s_cfg.i2s_port = I2S_PORT;
	i2s_cfg.out_rb_size = 8192;
	i2s_cfg.task_core = 1;
	i2s_cfg.i2s_config.mode =(I2S_MODE_TX | I2S_MODE_RX | I2S_MODE_MASTER);
	i2s_cfg.i2s_config.intr_alloc_flags =ESP_INTR_FLAG_LEVEL1;
	if (modoaudio==1)
		i2s_cfg.i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
	else
		i2s_cfg.i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
	i2s_cfg.i2s_config.communication_format=I2S_COMM_FORMAT_STAND_I2S;
	i2s_cfg.i2s_config.sample_rate=samplerate;
	i2s_cfg.i2s_config.bits_per_sample=BITS_SAMPLE;
	i2s_cfg.i2s_config.dma_buf_count=32;
	i2s_cfg.i2s_config.dma_buf_len=64;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);
	
    ESP_LOGI(TAG, "[2.2] Create i2s stream to read data from codec chip");
    i2s_cfg.type = AUDIO_STREAM_READER;
    i2s_stream_reader = i2s_stream_init(&i2s_cfg);
}

void setupLISTENER()
{
	ESP_LOGI(TAG, "[ LISTENER ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    evt = audio_event_iface_init(&evt_cfg);
}

void setupPERIPH()
{
	ESP_LOGI(TAG, "[PERIPH] Initialize peripherals management");
	esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
	set = esp_periph_set_init(&periph_cfg);
	audio_board_key_init(set);
}

void setupKEYS()
{
    ESP_LOGI(TAG, "[ KEY ] Create and start input key service");
    input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
    input_key_service_cfg_t input_cfg = INPUT_KEY_SERVICE_DEFAULT_CONFIG();
    input_cfg.handle = set;
    periph_service_handle_t input_ser = input_key_service_create(&input_cfg);
    input_key_service_add_key(input_ser, input_key_info, INPUT_KEY_NUM);
    periph_service_set_callback(input_ser, input_key_service_cb, (void *)board_handle);
}

void setupMQTT()
{
	strcat(mqtttopicaudioip,callsign); strcat(mqtttopicaudioip,"/ip");
	strcat(mqtttopicaudioport,callsign); strcat(mqtttopicaudioport,"/port");
	strcat(mqtttopicaudiorate,callsign); strcat(mqtttopicaudiorate,"/rate");
	strcat(mqtttopicaudiobits,callsign); strcat(mqtttopicaudiobits,"/bits");
    const esp_mqtt_client_config_t mqtt_cfg = {
        .uri = BROKER_URI,
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

void app_main(void)
{
	setupLOG();
	setupSD();
	readConfig();
	setupWiFi();
    setupPERIPH();
	setupI2S();
	setupCODEC();
	setupKEYS();
	initWiFi();
	setupLISTENER();
	setupMQTT();
    //xTaskCreate(audio_task, "audio_task", 10000, (void*)AF_INET, 5, NULL);
    xTaskCreate(timer1, "timer1", 4096, NULL, 5, NULL);

    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) { ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret); continue; }
    }

	// PARAR TODO
    ESP_LOGI(TAG, "[ 6 ] Stop audio_pipeline");
    esp_periph_set_stop_all(set);
    audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);
    audio_event_iface_destroy(evt);
    audio_element_deinit(i2s_stream_writer);
    esp_periph_set_destroy(set);
	
}
