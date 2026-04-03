#include <inttypes.h>
#include "freertos/FreeRTOS.h"

#include <math.h>
#include <string.h>
#include <algorithm>
#include <ranges>
#include <span>

#include "driver/gptimer.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "soc/spi_periph.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"

#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
#include <stdio.h>



#include "driver/rmt_rx.h"

#define IDLE_THRESHOLD_US 300


#define RMT_SYMBOLS (48*8)
static rmt_symbol_word_t rmt_buf[RMT_SYMBOLS] __attribute__((aligned(4)));


static const char *TAG = "MHI-AC-CTRL-core";

#define vTaskDelayMs(ms)            vTaskDelay((ms)/portTICK_PERIOD_MS)

#define RCV_HOST                    SPI2_HOST
#define ESP_INTR_FLAG_DEFAULT       0                                           // default to allocating a non-shared interrupt of level 1, 2 or 3.
#define STACK_SIZE 2560

#define MHI_FRAME_LEN_SHORT         20
#define MHI_FRAME_LEN_LONG          33

gptimer_handle_t cs_timer = NULL;
static StaticTask_t xTaskBuffer;
static StackType_t xStack[ STACK_SIZE ];
static StaticTask_t xTaskBuffer2;
static StackType_t xStack2[ STACK_SIZE ];
static TaskHandle_t mhi_poll_task_handle = NULL;
static TaskHandle_t rmt_poll_task_handle = NULL;

enum FrameIndices {
  SB0 = 0,
  SB1 = 1,
  SB2 = 2,
  DB0 = 3, ///< mode DB0[4:2]
  DB1 = 4, ///< fan speed [1-3]
  DB2 = 5, ///< set room temp DB2[6:0]. T[°C]=DB2[6:0]/2 The resolution is 0.50°C
  DB3 = 6, ///< room temp DB3[7:0]. T[°C]=(DB3[7:0]-61)/4 The resolution is 0.25°C
  DB4 = 7, ///< error code
  DB6 = 9, ///< fan speed 4 DB6[6]
  DB9 = 12,
  DB10 = 13,
  DB11 = 14,
  DB12 = 15,
  DB13 = 16, ///< compressor status. DB13[0] AC is on, DB13[1] AC is in heat mode, DB13[2]  compressor running/idle
  DB14 = 17, ///< used on MISO toggle clock bit every 20 frames
  CBH = 18,
  CBL = 19,
  DB15,
  DB16,
  DB17,
  DB18,
  DB19,
  DB20,
  DB21,
  DB22,
  DB23,
  DB24,
  DB25,
  DB26,
  CBL2,
  FRAME_LEN = MHI_FRAME_LEN_LONG
};

struct Config {
  uint8_t mosi_pin;
  uint8_t miso_pin;
  uint8_t sclk_pin;
};

// Needs length that is a multiple of 4
using spi_dma_buf_t = std::array<uint8_t, MHI_FRAME_LEN_LONG + 4 - (MHI_FRAME_LEN_LONG % 4)>;

static DMA_ATTR spi_dma_buf_t sendbuf;
static DMA_ATTR spi_dma_buf_t recvbuf;

static uint32_t frame_errors = 0;
static uint32_t frame_oks = 0;

static bool validate_signature(uint8_t sb0, uint8_t sb1, uint8_t sb2) {
  return (sb0 & 0xfe) == 0x6c && sb1 == 0x80 && sb2 == 0x04;
}

static int validate_frame_short(std::span<uint8_t, MHI_FRAME_LEN_SHORT> mosi_frame, uint16_t rx_checksum) {
  if (! validate_signature(mosi_frame[SB0], mosi_frame[SB1], mosi_frame[SB2])) {
    ESP_LOGW(TAG, "wrong MOSI signature. 0x%02x 0x%02x 0x%02x",
                mosi_frame[0], mosi_frame[1], mosi_frame[2]);

    return -1;
  } else if ( (mosi_frame[CBH] != (rx_checksum>>8 & 0xff)) | (mosi_frame[CBL] != (rx_checksum & 0xff)) ) {
    ESP_LOGW(TAG, "wrong short MOSI checksum. calculated 0x%04x. MOSI[18]:0x%02x MOSI[19]:0x%02x",
                rx_checksum, mosi_frame[CBH], mosi_frame[CBL]);

    return -2;
  }
  return 0;
}

static int validate_frame_long(std::span<uint8_t, MHI_FRAME_LEN_LONG> mosi_frame, uint8_t rx_checksum) {
  if(mosi_frame[CBL2] != rx_checksum) {
    ESP_LOGW(TAG, "wrong long MOSI checksum. calculated 0x%02x. MOSI[32]:0x%02x",
                rx_checksum, mosi_frame[CBL2]);
    return -3;
  }
  return 0;
}

static int validate_frame(std::span<uint8_t, MHI_FRAME_LEN_LONG> mosi_frame, uint8_t frame_len) {
  int err = 0;;
  uint16_t rx_checksum = 0;
  // Frame len has been validated before to only be either MHI_FRAME_LEN_LONG or MHI_FRAME_LEN_SHORT
  for (uint8_t i = 0; i < frame_len; i++) {
    switch(i) {
      case CBH:
        // validate checksum short
        err = validate_frame_short(std::span{mosi_frame}.first<MHI_FRAME_LEN_SHORT>(), rx_checksum);
        if(err) {
          return err;
        }
        rx_checksum += mosi_frame[CBH];
        rx_checksum += mosi_frame[CBL];
        // skip over CBL
        i++;
        break;
      case CBL2:
        err = validate_frame_long(mosi_frame, rx_checksum);
        if(err) {
          return err;
        }
        break;
      default:
        rx_checksum += mosi_frame[i];
    }
  }
  return err;
}

static void mhi_poll_task(void *arg)
{
    esp_err_t err = 0;

    spi_slave_transaction_t spi_slave_trans;

    //Set up a transaction of MHI_FRAME_LEN bytes to send/receive
    spi_slave_trans.length = recvbuf.size() * 8;
    spi_slave_trans.tx_buffer = &sendbuf;
    spi_slave_trans.rx_buffer = &recvbuf;

    while(1) {
        if (err) {
            ESP_LOGW(TAG, "error %i . trans len: %i", err, spi_slave_trans.trans_len);
            frame_errors++;
        } else {
            ESP_LOGW(TAG, "Frame OK (%lu / %lu)", frame_oks, frame_oks + frame_errors);
        }
        err = 0;

        // We're ready, wait on the SPI transaction to happen
        err = spi_slave_transmit(RCV_HOST, &spi_slave_trans, pdMS_TO_TICKS(1000));
        if(err) {
          if(err == ESP_ERR_TIMEOUT) {
            frame_errors++;
            ESP_LOGE(TAG, "SPI transaction timeout.");
          } else {
            frame_errors++;
            ESP_LOGE(TAG, "get_trans_result error: %i", err);
          }
          continue;
        }

        // Transaction must be of a supported length
        if(spi_slave_trans.trans_len != MHI_FRAME_LEN_LONG * 8
            && spi_slave_trans.trans_len != MHI_FRAME_LEN_SHORT * 8) {
          err = true;
          continue;
        }
        const size_t trans_len_bytes = spi_slave_trans.trans_len / 8;

        // Validate SPI transaction
        err = validate_frame(std::span{recvbuf}.first<MHI_FRAME_LEN_LONG>(), trans_len_bytes);
        if(err != 0) {
          frame_errors++;
          continue;
        }
        frame_oks++;
    }
}

void init(const Config& config) {
    esp_err_t err;

    std::fill(sendbuf.begin(), sendbuf.end(), 0xff);

    // configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = config.mosi_pin,
        .miso_io_num = config.miso_pin,
        .sclk_io_num = config.sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .data_io_default_level = 1,
        .max_transfer_sz = 0,
        .flags = SPICOMMON_BUSFLAG_GPIO_PINS | SPICOMMON_BUSFLAG_SLAVE,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
        .intr_flags = 0,
    };

    // configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .spics_io_num = -1,
        .flags = SPI_SLAVE_BIT_LSBFIRST,
        .queue_size = 1,
        .mode = 3,                    //CPOL=1, CPHA=1
        .post_setup_cb = 0,
        .post_trans_cb = 0,
    };

    // initialize SPI slave interface
    err = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);    // can't disable DMA. no comms if you do...
    ESP_ERROR_CHECK(err);


    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<config.sclk_pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;                                     // required to prevent abort() caused by floating pin when daughtboard not connected
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&io_conf);


    io_conf.pin_bit_mask = (1ULL<<21);
    gpio_config(&io_conf);


    mhi_poll_task_handle = xTaskCreateStatic(mhi_poll_task, "mhi_task", STACK_SIZE, NULL, 10, xStack, &xTaskBuffer);
}

static void print_waveform(const rmt_symbol_word_t *syms, size_t count)
{
    printf("Captured %d symbols (%d clock pulses):\n", (int)count, (int)count);
    if( count == 264)
    {
      bool state = syms[0].level0;
      for (size_t i = 0; i < count; i++) {
        if(syms[i].level0 != state || syms[i].level1 == state) {
          printf("sym error %i", i);
          goto print;
        }
      }
      return;
    }
print:
    printf("%-6s %-12s %-12s\n", "Sym#", "Level0/dur0", "Level1/dur1");
    printf("----------------------------------\n");
    for (size_t i = 0; i < count; i++) {
        printf("[%3d]  L%d: %5d µs    L%d: %5d µs\n",
               (int)i,
               syms[i].level0, syms[i].duration0,
               syms[i].level1, syms[i].duration1);
    }
}

static bool IRAM_ATTR on_recv_done(rmt_channel_handle_t channel,
                                   const rmt_rx_done_event_data_t *edata,
                                   void *user_ctx)
{
    // edata->received_symbols points into rmt_buf
    // edata->num_symbols is how many were actually written
    //
    // Can't call printf from ISR — post to a task instead.
    // For simplicity here we use a queue to pass the count.
    BaseType_t high_task_woken = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    size_t num = edata->num_symbols;
    xQueueSendFromISR(queue, &num, &high_task_woken);
    return high_task_woken == pdTRUE;
}

static void rmt_init(void *arg) {
    Config* c= static_cast<Config*>(arg);

    QueueHandle_t queue = xQueueCreate(4, sizeof(size_t));

    // 1. Create the RX channel
    rmt_channel_handle_t rx_chan;
    rmt_rx_channel_config_t rx_cfg = {};  // zero-initialize everything first
    rx_cfg.gpio_num          = static_cast<gpio_num_t>(c->sclk_pin);
    rx_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
    rx_cfg.resolution_hz     = 10000000;
    rx_cfg.mem_block_symbols = RMT_SYMBOLS;
    rx_cfg.flags.with_dma    = true;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &rx_chan));

    // 2. Register callback, pass queue as user context
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = on_recv_done,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &cbs, queue));

    // 3. Enable and arm
    ESP_ERROR_CHECK(rmt_enable(rx_chan));

    rmt_receive_config_t recv_cfg = {};
    recv_cfg.signal_range_min_ns = 0;
    recv_cfg.signal_range_max_ns = IDLE_THRESHOLD_US * 1000 * 10;
    // flags.en_partial_rx defaults to false, which is what we want
    ESP_ERROR_CHECK(rmt_receive(rx_chan, rmt_buf, sizeof(rmt_buf), &recv_cfg));

    ESP_LOGI(TAG, "Listening...");

    while (1) {
        size_t num_symbols;
        if (xQueueReceive(queue, &num_symbols, portMAX_DELAY)) {
            print_waveform(rmt_buf, num_symbols);

            // Re-arm for the next burst
            ESP_ERROR_CHECK(rmt_receive(rx_chan, rmt_buf, sizeof(rmt_buf), &recv_cfg));

            // Trigger Chip Select low->high->low
            esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, spi_periph_signal[RCV_HOST].spics_in, false);
            esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, spi_periph_signal[RCV_HOST].spics_in, false);
        }
    }

}


extern "C" {
void app_main(void)
{
    printf("Hello world!\n");

    Config c = {
      .mosi_pin = 48,
      //.miso_pin = 34,
      .miso_pin = 40,
      .sclk_pin = 21
      //.sclk_pin = 43
    };

    vTaskDelayMs(1000);

    init(c);
    rmt_poll_task_handle = xTaskCreateStatic(rmt_init, "rmt_task", STACK_SIZE, &c, 10, xStack2, &xTaskBuffer2);
}
}
