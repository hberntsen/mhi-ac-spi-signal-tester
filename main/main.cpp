#include <inttypes.h>

#include <math.h>
#include <string.h>
#include <algorithm>
#include <ranges>
#include <span>

#include "driver/gptimer.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "soc/spi_periph.h"

#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
#include <stdio.h>

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
static TaskHandle_t mhi_poll_task_handle = NULL;

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

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
  uint64_t current_timer_value;
  // We need to know whether the timer has already been started. If we start it when it is already started, we get the
  // following in the logs:
  // > gptimer: gptimer_start(348): timer is not enabled in the logs
  //
  // There is no direct API to check the status, so use the raw count instead
  ESP_ERROR_CHECK(gptimer_get_raw_count(cs_timer, &current_timer_value));
  if(current_timer_value == 0) {
    gptimer_start(cs_timer);
  } else {
    ESP_ERROR_CHECK(gptimer_set_raw_count(cs_timer, 0));
  }
}

static bool IRAM_ATTR gptimer_isr_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Stop timer, gpio isr will turn it back on
    gptimer_stop(timer);
    // Set to 0 to tell the GPIO isr that the timer is not running
    gptimer_set_raw_count(cs_timer, 0);

    // Trigger Chip Select low->high->low
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, spi_periph_signal[RCV_HOST].spics_in, false);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, spi_periph_signal[RCV_HOST].spics_in, false);

    vTaskNotifyGiveFromISR(mhi_poll_task_handle, &xHigherPriorityTaskWoken);

    return xHigherPriorityTaskWoken;
}

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

    gptimer_event_callbacks_t timer_callbacks = {
      .on_alarm = gptimer_isr_callback
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(cs_timer, &timer_callbacks, NULL));
    ESP_ERROR_CHECK(gptimer_enable(cs_timer));
    // Set the count to 0, so the gpio ISR will start the timer
    ESP_ERROR_CHECK(gptimer_set_raw_count(cs_timer, 0));

    //Set up a transaction of MHI_FRAME_LEN bytes to send/receive
    spi_slave_trans.length = recvbuf.size() * 8;
    spi_slave_trans.tx_buffer = &sendbuf;
    spi_slave_trans.rx_buffer = &recvbuf;

    while(1) {
        if (err) {
            ESP_LOGW(TAG, "error %i . trans len: %i", err, spi_slave_trans.trans_len);
            frame_errors++;
        } else {
            ESP_LOGW(TAG, "Frame OK");
        }
        err = 0;


        // The GPIO CLK triggered timer alarm will clear right after a frame. Wait for it so we don't transmit the
        // transaction mid-frame. There is a very small chance that it will still happen in between the check here and
        // the actual transaction starting.
        uint64_t current_timer_value;
        do {
          uint32_t frames_received = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(10000));
          if(frames_received == 0) {
            ESP_LOGE(TAG, "No SPI clock detected");
            frame_errors += 1;
            continue;
          } else if(frames_received > 1) {
            // Add missed frames to frame_errors
            frame_errors += frames_received - 1;
            ESP_LOGE(TAG, "Missed %u frames", frames_received - 1);
          }
          ESP_ERROR_CHECK(gptimer_get_raw_count(cs_timer, &current_timer_value));
        } while(current_timer_value != 0);
        // We're ready, wait on the SPI transaction to happen
        err = spi_slave_transmit(RCV_HOST, &spi_slave_trans, pdMS_TO_TICKS(10000));
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
        .max_transfer_sz = 0,
        .flags = SPICOMMON_BUSFLAG_GPIO_PINS | SPICOMMON_BUSFLAG_SLAVE,
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

    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, spi_periph_signal[RCV_HOST].spics_in, false);

    // Set up timer
    gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000, // Plenty of resolution to encode a rough 20ms ;)
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &cs_timer));
    gptimer_alarm_config_t timer_alarm_config = {
      // Configure the alarm value (in milliseconds) and the interrupt on alarm. there is a gap between each frame of
      // 40ms with short frames and about 33ms with long frames.
      // We set the alarm to 1ms, so it is triggering after 1ms of no clock. We receive about 2 bytes per millisecond.
      // This tolerance should be enough and will give us maximum time to process this frame and prepare the next.
      //
      // Once the alarm triggers, we toggle the CS line to mark the end/start of a SPI transaction to the hardware, as
      // it won't work without. The GPIO interrupt below resets this timer every time the clock signal is low. When the
      // SPI transaction is complete, the master leaves the clock pin high, setting of the alarm and us toggling the
      // pin.
      .alarm_count = 1 * (timer_config.resolution_hz / 1000),
      .reload_count = 0,
      .flags = {
        // We manually reload based on the clock signal we get from the AC
        .auto_reload_on_alarm = false,
      }
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(cs_timer, &timer_alarm_config));
    // The ISR uses the counter value to determine whether it is already running. The ISR will probably trigger before
    // we are ready in the mhi_poll_task, so set a non-zero value so it assumes it has already been started. The
    // mhi_poll_task will prepare the timer with a 0 count.
    ESP_ERROR_CHECK(gptimer_set_raw_count(cs_timer, timer_config.resolution_hz));

    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<config.sclk_pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;                                     // required to prevent abort() caused by floating pin when daughtboard not connected
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(static_cast<gpio_num_t>(config.sclk_pin), gpio_isr_handler, NULL);
    gpio_intr_enable(static_cast<gpio_num_t>(config.sclk_pin));

    mhi_poll_task_handle = xTaskCreateStatic(mhi_poll_task, "mhi_task", STACK_SIZE, NULL, 10, xStack, &xTaskBuffer);
}


extern "C" {
void app_main(void)
{
    printf("Hello world!\n");

    Config c = {
      .mosi_pin = 48,
      .miso_pin = 34,
      //.sclk_pin = 21
      .sclk_pin = 43
    };

    init(c);
}
}
