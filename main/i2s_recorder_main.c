/* I2S Digital Microphone Recording Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
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
#include "sdmmc_cmd.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "string.h"

static const char *TAG = "pdm_rec_example";
static const int RX_BUF_SIZE = 1024;

#define TXD_PIN 32
#define RXD_PIN 33
#define SAMPLE_SIZE (16 * 1024)
#define BYTE_RATE (I2Sclk * (16 / 8)) * 2
#define I2Schan 0
#define I2Sclk 160 * 1000

static int16_t i2s_readraw_buff[SAMPLE_SIZE];
size_t bytes_read;

void init_microphone(void)
{
    // Set the I2S configuration as PDM and 16bits per sample
    // 设置I2S为PDM模式并设置每次采样16位
    i2s_config_t i2s_config = {
        // 工作模式 - 主机、接收、PDM
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM,
        // 采样率 - 160kHz
        .sample_rate = I2Sclk,
        // 采样深度
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        // 通道格式 - 左右声道
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        //通信格式
        .communication_format = I2S_COMM_FORMAT_STAND_PCM_SHORT,
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
    ESP_ERROR_CHECK(i2s_set_clk(I2Schan, I2Sclk, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO));
}

/*filter
void filter(void)
{
    float a[15]={1.0000, 0.4770, 1.8441, 0.7249, 1.4256, 0.4546, 0.5995, 0.1505, 0.1482, 0.0278, 0.0215, 0.0027, 0.0017, 0.0001, 0.0001 };
    for(uint i = 14; i < 10000 ; i++)
    {
        yf[i] = 0.0000001518 * signal[ i-7 ];
        for(uint j = 1; j < 15 ; j++)
            yf[i] = yf[i] - a[j] * yf[ i+1-j ];
    }
}
*/

void init_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char *data)
{
    static const char *TX_TAG = "TX";
    esp_log_level_set(TX_TAG, ESP_LOG_INFO);
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    //ESP_LOGI(TX_TAG, "Wrote %d bytes", txBytes);
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
            //ESP_LOGI(RX_TAG, "Read %d bytes: '%s'", rxBytes, data);
            break;
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "PDM microphone recording Example start");
    // Init the PDM digital microphone
    init_microphone();
    init_uart();
    char data[RX_BUF_SIZE];
    // Start Recording
    // Read the RAW samples from the microphone
    ESP_LOGI(TAG, "CLK = %.1f kHz", i2s_get_clk(I2Schan)/1000);
    i2s_read(I2Schan, (char *)i2s_readraw_buff, SAMPLE_SIZE, &bytes_read, 100);
    ESP_LOGI(TAG, "%d", i2s_readraw_buff[0]);
    while (1)
    {
        // i2s_read(I2Schan, (char *)i2s_readraw_buff, SAMPLE_SIZE, &bytes_read, 100);
        rxData(data);
        //ESP_LOGI(TAG, "UART:%s", data);
        sendData(data);
    }
    // Stop I2S driver and destroy
    ESP_ERROR_CHECK(i2s_driver_uninstall(I2Schan));
}
