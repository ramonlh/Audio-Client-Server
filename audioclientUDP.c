/*	Cliente de audio
	Recibe y envía sonido vía TCP 
	El servidor envía y recibe el flujo I2S sin ningún tipo de conversión
	Funciona con la tarjeta ESP32-LyraT
	Se conecta aun servidor TCP, entonces empieza a recibir y enviar audio
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

#if __has_include("esp_idf_version.h")
#include "esp_idf_version.h"
#else
#define ESP_IDF_VERSION_VAL(major, minor, patch) 1
#endif

#include "esp_netif.h"

#define WIFI_SSID "conuco4"
#define WIFI_PASSWORD "18921892"

#define SEND_HOST "192.168.1.125"
#define REC_PORT 3334		// en el servidor es a la inversa
#define SEND_PORT 3333		// en el servidor es a la inversa

#define I2S_PORT I2S_NUM_0
#define SAMPLE_RATE (8000)
#define BITS_SAMPLE (24)
#define BUFFLEN (1024)

static const char *TAG = "audioclient";
audio_element_handle_t i2s_stream_writer, i2s_stream_reader;

unsigned long totalrec=0;
unsigned long totalsent=0;
unsigned long speedrec=0;
unsigned long speedsent=0;
int player_volume;

static void timer1(void *pvParameters)
{
	while(1) {
		if ((speedrec>0) || (speedsent>0)) {
			ESP_LOGI(TAG, "speedrec / speedsent: %lu / %lu", speedrec, speedsent);
			speedrec=0;	speedsent=0;
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

static void udp_server_task(void *pvParameters)
{
    uint8_t rx_buffer[BUFFLEN];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {

        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(REC_PORT);
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", REC_PORT);

        while (1) {
            //ESP_LOGI(TAG, "Waiting for data");
            struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer),0,(struct sockaddr *)&source_addr, &socklen);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.sin6_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                }
				speedrec=speedrec+len;
				totalrec=totalrec+len;
				size_t byteswritten=0;
				i2s_write(I2S_PORT, rx_buffer, len, &byteswritten, portMAX_DELAY);
                //ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);

                //int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                //if (err < 0) {
                //    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                //    break;
                //}
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

static void udp_sender_task(void *pvParameters)
{
    int addr_family = 0;
    int ip_protocol = 0;
    uint8_t tx_buffer[BUFFLEN];

    while (1) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(SEND_HOST);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(SEND_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
		
        int sockudp = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sockudp < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", SEND_HOST, SEND_PORT);
			
        while (1) {
			/////////////////// read I2S->Send TCP
			size_t reci2s=0;
			i2s_read(I2S_PORT, tx_buffer, BUFFLEN, &reci2s, portMAX_DELAY);
            int sentlen = sendto(sockudp, tx_buffer, BUFFLEN, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (sentlen < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
			totalsent=totalsent+sentlen; speedsent=speedsent+sentlen;
            }
			
		if (sockudp != -1) {
			ESP_LOGE(TAG, "Shutting down socket and restarting...");
			shutdown(sockudp, 0);
			close(sockudp);
		}
       }
    vTaskDelete(NULL);
}


void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(esp_netif_init());
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
	
    ESP_LOGI(TAG, "[ 1 ] Start codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);
	esp_err_t ret = es8388_write_reg(ES8388_ADCCONTROL2, ADC_INPUT_LINPUT2_RINPUT2);
	if (ret != ESP_OK) { ESP_LOGI(TAG, "[ ES8388 ] ES8388 writereg error : %d", ret); }
    player_volume=100;
    audio_hal_get_volume(board_handle->audio_hal, &player_volume);

    ESP_LOGI(TAG, "[2.1] Create i2s stream to write data to codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
	i2s_cfg.i2s_port = I2S_PORT;
	i2s_cfg.out_rb_size = 8192;
	i2s_cfg.task_core = 1;
	i2s_cfg.i2s_config.mode =(I2S_MODE_TX | I2S_MODE_RX | I2S_MODE_MASTER);
	i2s_cfg.i2s_config.intr_alloc_flags =ESP_INTR_FLAG_LEVEL1;
	i2s_cfg.i2s_config.channel_format=I2S_CHANNEL_FMT_ONLY_LEFT;
	i2s_cfg.i2s_config.communication_format=I2S_COMM_FORMAT_STAND_I2S;
	i2s_cfg.i2s_config.sample_rate=SAMPLE_RATE;
	i2s_cfg.i2s_config.bits_per_sample=BITS_SAMPLE;
	i2s_cfg.i2s_config.dma_buf_count=32;
	i2s_cfg.i2s_config.dma_buf_len=64;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);
	
    ESP_LOGI(TAG, "[2.2] Create i2s stream to read data from codec chip");
    i2s_cfg.type = AUDIO_STREAM_READER;
    i2s_stream_reader = i2s_stream_init(&i2s_cfg);

    ESP_LOGI(TAG, "[ 3 ] Start and wait for Wi-Fi network");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);
    periph_wifi_cfg_t wifi_cfg = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASSWORD,
    };
    esp_periph_handle_t wifi_handle = periph_wifi_init(&wifi_cfg);
    esp_periph_start(set, wifi_handle);
    periph_wifi_wait_for_connected(wifi_handle, portMAX_DELAY);

    ESP_LOGI(TAG, "[ 4 ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    xTaskCreate(udp_sender_task, "udp_sender", 10000, NULL, 5, NULL);
    xTaskCreate(timer1, "timer1", 10000, NULL, 5, NULL);
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);

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
