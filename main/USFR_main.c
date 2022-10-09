/* I2S Digital Microphone Recording Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <memory.h>
#include <math.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/rmt.h"
#include "led_strip.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"

#define I2S_CLK  4
#define I2S_DTA  5
#define RMT_TX  12
#define UART_TX 32
#define UART_RX 33
#define SAMPLE_SIZE (16 * 1024)
#define BYTE_RATE (I2Sclk * (16 / 8)) * 2
#define I2Schan 0
#define I2Sclk 50*1000//160 * 1000
#define I2Smode I2S_CHANNEL_STEREO//I2S_CHANNEL_MONO
#define RMT_TIMEOUT 10
#define RED color[0]
#define YELLOW color[1]
#define GREEN color[2]
#define SDU color[3]
#define FILTER_ORDER 8

static const char *TAG = "UltraSonicFollowReceiver-ESP32";
static const int RX_BUF_SIZE = 1024;
static int16_t i2s_readraw_buff[SAMPLE_SIZE];
static led_strip_t *strip;
size_t bytes_read;
uint8_t color[4][3] ={
    { 255,   0,  0},//RED
    { 255, 255,  0},//YELLOW
    {   0, 255,  0},//GREEN
    { 156,  12, 19}//SDU-RED
    };

void init_i2s()
{
    // Set the I2S configuration as PDM and 16bits per sample
    // 设置I2S为PDM模式并设置每次采样16位
    i2s_config_t i2s_config = {
        // 工作模式 - 主机、接收、PDM
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM,
        // 采样率 - 160kHz
        .sample_rate = 20*1000,
        // 采样深度
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        // 通道格式 - 左右声道
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        //通信格式
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        //中断级别
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,
        //接收/传输数据的DMA缓冲区的总数
        .dma_buf_count = 8,
        // DMA缓冲区中的帧长度
        .dma_buf_len = 512,
        // 是否使用APLL
        .use_apll = false,
    };

    // Set the pinout configuration (set using menuconfig)
    i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_PIN_NO_CHANGE,
        .bck_io_num = I2S_PIN_NO_CHANGE,
        .ws_io_num = 4,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = 5,
    };

    // Call driver installation function before any I2S R/W operation.
    ESP_ERROR_CHECK(i2s_driver_install(I2Schan, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2Schan, &pin_config));
    ESP_ERROR_CHECK(i2s_set_clk(I2Schan, 20*1000, I2S_BITS_PER_SAMPLE_16BIT, I2Smode));

    ESP_LOGI(TAG, "CLK = %.1f kHz", i2s_get_clk(I2Schan)/1000);
    vTaskDelay(1000/portTICK_RATE_MS);
    ESP_ERROR_CHECK(i2s_set_sample_rates(I2Schan,I2Sclk));
    ESP_ERROR_CHECK(i2s_set_clk(I2Schan, I2Sclk, I2S_BITS_PER_SAMPLE_16BIT, I2Smode));
    ESP_LOGI(TAG, "CLK = %.1f kHz", i2s_get_clk(I2Schan)/1000);

    

}

void init_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 460800,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void init_rmt(void)
{
    // init rmt
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(RMT_TX, RMT_CHANNEL_0);
    // set counter clock to 40MHz
    config.clk_div = 2;
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(1, (led_strip_dev_t)config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, RMT_TIMEOUT));
    ESP_LOGI(TAG, "LED Inited");
}

int txData(const char *data, const int start, const int len)
{
    static const char *TX_TAG = "TX";
    esp_log_level_set(TX_TAG, ESP_LOG_INFO);
    const int txBytes = uart_write_bytes(UART_NUM_1, data + start, len);
    ESP_LOGI(TX_TAG, "Wrote %d bytes", txBytes);
    return txBytes;
}

void rxData(char *data)
{
    static const char *RX_TAG = "RX";
    esp_log_level_set(RX_TAG, ESP_LOG_INFO);

    while (1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 10 / portTICK_RATE_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TAG, "Read %d bytes: '%s'", rxBytes, data);
            break;
        }
    }
}

void set_color(led_strip_t *strip,uint8_t *color)
{
    ESP_ERROR_CHECK(strip->set_pixel(strip, 0, (uint32_t) color[0], (uint32_t) color[1], (uint32_t) color[2]));
    ESP_ERROR_CHECK(strip->refresh(strip, RMT_TIMEOUT));
}

void clear_color(led_strip_t *strip)
{
    ESP_ERROR_CHECK(strip->clear(strip, RMT_TIMEOUT));
}

float filter_sum(void)
{
    ESP_LOGI(TAG, "Filter Start!");
    float sum[2] = { 0.0, 0.0 };
    float *left = (float*) malloc(SAMPLE_SIZE * sizeof(float)/2);
    float *right = (float*) malloc(SAMPLE_SIZE * sizeof(float)/2);
    //滤波器差分方程参数
    float b[FILTER_ORDER+1] = {0.0556,0.1503,0.2578,0.3064,0.2578,0.1503,0.0556,0.0101,0.0556};
    float a[FILTER_ORDER] = {5.8722,16.4355,28.1077,31.9720,24.7116,12.7036,3.9904,0.5975};
    //左右声道初始赋值
    for(uint8_t i = 0; i < FILTER_ORDER*4 ; i++)
    {
         left[i] = 0.0;
        right[i] = 0.0;
    }
    for(uint16_t i = FILTER_ORDER*2; i < SAMPLE_SIZE/2 ; i++)
    {
         left[i] = b[FILTER_ORDER] * i2s_readraw_buff[i*2];
        right[i] = b[FILTER_ORDER] * i2s_readraw_buff[i*2+1];
        for(uint j = 0; j < FILTER_ORDER ; j++)
        {
             left[i] += b[j] * i2s_readraw_buff[(i-1-j)*2] - a[j] * left[i-1-j];
            right[i] += b[j] * i2s_readraw_buff[(i-1-j)*2+1] - a[j] * right[i-1-j];
        }
        sum[0] += abs(left[i]) / ((SAMPLE_SIZE/2)-(FILTER_ORDER*2));
        sum[1] += abs(right[i]) / ((SAMPLE_SIZE/2)-(FILTER_ORDER*2));
        //ESP_LOGI(TAG, "%.3f", copy[i]);
    }
    ESP_LOGI(TAG, "Filter End!");
    ESP_LOGI(TAG, "Avg Left  = %.3f",sum[0]);
    ESP_LOGI(TAG, "Avg Right = %.3f",sum[1]);
    free(left);
    free(right);
    return sum[0] - sum[1];
}

void app_main(void)
{
    ESP_LOGI(TAG, "PDM microphone recording Example start");

    // Init 
    init_rmt();
    set_color(strip,SDU);
    init_i2s();
    init_uart();
    set_color(strip,RED);
    

    i2s_read(I2Schan, (char *)i2s_readraw_buff, SAMPLE_SIZE, &bytes_read, 20);
    for(int i =0;i<100;i++)
        ESP_LOGI(TAG, "%hu", i2s_readraw_buff[i]);
    i2s_read(I2Schan, (char *)i2s_readraw_buff, SAMPLE_SIZE, &bytes_read, 20);
    ESP_LOGI(TAG, "Read %d bytes", (int)bytes_read);
    //char data[RX_BUF_SIZE];
    while (1)
    {
        //rxData(data);
        //txData(data);
        i2s_read(I2Schan, (char *)i2s_readraw_buff, SAMPLE_SIZE, &bytes_read, 20);
        i2s_read(I2Schan, (char *)i2s_readraw_buff, SAMPLE_SIZE, &bytes_read, 20);
        ESP_LOGI(TAG, "Read %d bytes", (int)bytes_read);
        //ESP_LOGI(TAG, "%hu", i2s_readraw_buff[1]);
        ESP_LOGI(TAG, "Delta = %.3f", filter_sum());
        vTaskDelay(1000/portTICK_RATE_MS);
    }
    // Stop I2S driver and destroy
    ESP_ERROR_CHECK(i2s_driver_uninstall(I2Schan));
}