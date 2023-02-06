/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/xtensa_api.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bt_app_core.h"
#include "driver/i2s.h"
#include "freertos/ringbuf.h"

#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_SPDIF
#include "spdif.h"
#endif

static void bt_app_task_handler(void *arg);
static bool bt_app_send_msg(bt_app_msg_t *msg);
static void bt_app_work_dispatched(bt_app_msg_t *msg);

static xQueueHandle s_bt_app_task_queue = NULL;
static xTaskHandle s_bt_app_task_handle = NULL;
static xTaskHandle s_bt_i2s_task_handle = NULL;
static RingbufHandle_t s_ringbuf_i2s = NULL;;

#define RINGBUF_SIZE (16 * 1024)

bool bt_app_work_dispatch(bt_app_cb_t p_cback, uint16_t event, void *p_params, int param_len, bt_app_copy_cb_t p_copy_cback)
{
    ESP_LOGD(BT_APP_CORE_TAG, "%s event 0x%x, param len %d", __func__, event, param_len);

    bt_app_msg_t msg;
    memset(&msg, 0, sizeof(bt_app_msg_t));

    msg.sig = BT_APP_SIG_WORK_DISPATCH;
    msg.event = event;
    msg.cb = p_cback;

    if (param_len == 0) {
        return bt_app_send_msg(&msg);
    } else if (p_params && param_len > 0) {
        if ((msg.param = malloc(param_len)) != NULL) {
            memcpy(msg.param, p_params, param_len);
            /* check if caller has provided a copy callback to do the deep copy */
            if (p_copy_cback) {
                p_copy_cback(&msg, msg.param, p_params);
            }
            return bt_app_send_msg(&msg);
        }
    }

    return false;
}

static bool bt_app_send_msg(bt_app_msg_t *msg)
{
    if (msg == NULL) {
        return false;
    }

    if (xQueueSend(s_bt_app_task_queue, msg, 10 / portTICK_RATE_MS) != pdTRUE) {
        ESP_LOGE(BT_APP_CORE_TAG, "%s xQueue send failed", __func__);
        return false;
    }
    return true;
}

static void bt_app_work_dispatched(bt_app_msg_t *msg)
{
    if (msg->cb) {
        msg->cb(msg->event, msg->param);
    }
}

static void bt_app_task_handler(void *arg)
{
    bt_app_msg_t msg;
    for (;;) {
        if (pdTRUE == xQueueReceive(s_bt_app_task_queue, &msg, (portTickType)portMAX_DELAY)) {
            ESP_LOGD(BT_APP_CORE_TAG, "%s, sig 0x%x, 0x%x", __func__, msg.sig, msg.event);
            switch (msg.sig) {
            case BT_APP_SIG_WORK_DISPATCH:
                bt_app_work_dispatched(&msg);
                break;
            default:
                ESP_LOGW(BT_APP_CORE_TAG, "%s, unhandled sig: %d", __func__, msg.sig);
                break;
            } // switch (msg.sig)

            if (msg.param) {
                free(msg.param);
            }
        }
    }
}

void bt_app_task_start_up(void)
{
    s_bt_app_task_queue = xQueueCreate(10, sizeof(bt_app_msg_t));
    xTaskCreate(bt_app_task_handler, "BtAppT", 3072, NULL, configMAX_PRIORITIES - 3, &s_bt_app_task_handle);
    return;
}

void bt_app_task_shut_down(void)
{
    if (s_bt_app_task_handle) {
        vTaskDelete(s_bt_app_task_handle);
        s_bt_app_task_handle = NULL;
    }
    if (s_bt_app_task_queue) {
        vQueueDelete(s_bt_app_task_queue);
        s_bt_app_task_queue = NULL;
    }
}

static void bt_i2s_task_handler(void *arg)
{
    uint8_t *data = NULL;
    size_t item_size = 0;
#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_SPDIF
    extern uint8_t s_volume;
    s_volume = 100;		// initialize default volume
#else
    size_t bytes_written = 0;
#endif

    for (;;) {
        data = (uint8_t *)xRingbufferReceive(s_ringbuf_i2s, &item_size, (portTickType)portMAX_DELAY);
        if (item_size != 0){
#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
	    uint16_t *dac = (uint16_t *)data;

	    for (int i = 0; i < item_size / sizeof(uint16_t); i++) {
		dac[i] += 32768; // convert to unsigned 16bit audio data
	    }
#endif
#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_SPDIF
	    int16_t *audio = (int16_t *)data;

	    if (s_volume < 100) {
	    	for (int i = 0; i < item_size / sizeof(uint16_t); i++) {
		    audio[i] = (audio[i] * s_volume) / 100;
		}
	    }

	    spdif_write(data, item_size);
#else
            i2s_write(0, data, item_size, &bytes_written, portMAX_DELAY);
#endif
            vRingbufferReturnItem(s_ringbuf_i2s,(void *)data);
        }
    }
}

void bt_i2s_task_start_up(void)
{
    s_ringbuf_i2s = xRingbufferCreate(RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if(s_ringbuf_i2s == NULL){
        return;
    }

#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_SPDIF
    xTaskCreate(bt_i2s_task_handler, "BtI2ST", 2048, NULL, configMAX_PRIORITIES - 3, &s_bt_i2s_task_handle);
#else
    xTaskCreate(bt_i2s_task_handler, "BtI2ST", 1024, NULL, configMAX_PRIORITIES - 3, &s_bt_i2s_task_handle);
#endif
    return;
}

void bt_i2s_task_shut_down(void)
{
    if (s_bt_i2s_task_handle) {
        vTaskDelete(s_bt_i2s_task_handle);
        s_bt_i2s_task_handle = NULL;
    }

    if (s_ringbuf_i2s) {
        vRingbufferDelete(s_ringbuf_i2s);
        s_ringbuf_i2s = NULL;
    }
}

#define AUDIO_SAMPLE_SIZE (16 * 2 / 8) // 16bit, 2ch, 8bit/byte

// Calculate a floating average of buffer fill percentage
// Since the buffer usually fills slowly and then gets empty in one go
// the 100% average filled will be at RINGBUF_SIZE / 2
// Values > 100 could be returned. This is when buffer overflows are likely
UBaseType_t buffer_floating_avg() {
    const static UBaseType_t avg_range = 200;
    static UBaseType_t avg_items = RINGBUF_SIZE / 8;  // assume 25% at start

    UBaseType_t items;
    vRingbufferGetInfo(s_ringbuf_i2s, NULL, NULL, NULL, NULL, &items);
    avg_items = ((avg_range - 1) * avg_items + items) / avg_range;

    return avg_items * 100 / (RINGBUF_SIZE / 2);
}

// Return true if serial output of buffer fill status should be printed
// Either if number of dropped samples changes (force==true) or every count_max calls
bool rate_limited_output( bool force ) {
    const static UBaseType_t count_max = 100;
    static UBaseType_t count = 0;
    if( force || ++count > count_max) {
        count = 0;
        return true;
    }
    return false;
}

// Return number of samples to drop in the new data
// Make sure the number toggles rarely by using hysteresis
int rate_control()
{
    const static UBaseType_t percent_limits[] = { 15, 25, 35, 45, 55 };  // limits to change curr_diff
    const static int max_diff = 1;   // max samples to add to the new data 
    const static int min_diff = -3;  // max samples to chop off the new data
    static int last_diff = 0;        // start with unchanged data

    int curr_diff = last_diff;       // start with unchanged level

    UBaseType_t filled_percent = buffer_floating_avg();
    if( curr_diff != max_diff && filled_percent < percent_limits[-curr_diff]) {
        curr_diff++;
    }
    else if( curr_diff != min_diff && filled_percent > percent_limits[2-curr_diff]) {
        curr_diff--;
    }

    if( rate_limited_output(curr_diff != last_diff) ) {
        ESP_LOGI(BT_APP_CORE_TAG, "%s %u%%, %+d samples", __func__, filled_percent, curr_diff);
    }

    last_diff = curr_diff;

    return curr_diff;
}

size_t write_ringbuf(const uint8_t *data, size_t size)
{
    size_t ret = size;
    
    int diff_samples = rate_control();  // number of samples to add or remove from data
    
    if( diff_samples > 0 ) {
        // send first sample twice
        xRingbufferSend(s_ringbuf_i2s, (void *)data, AUDIO_SAMPLE_SIZE, (portTickType)portMAX_DELAY);
    } 
    else if( diff_samples <= -3 ) {
        // send new data in three parts, each with last sample removed
        size /= AUDIO_SAMPLE_SIZE;
        size /= 3;
        size -= 1;
        size *= AUDIO_SAMPLE_SIZE;
        xRingbufferSend(s_ringbuf_i2s, (void *)data, size, (portTickType)portMAX_DELAY);
        data += size + AUDIO_SAMPLE_SIZE;
        xRingbufferSend(s_ringbuf_i2s, (void *)data, size, (portTickType)portMAX_DELAY);
        data += size + AUDIO_SAMPLE_SIZE;
    }
    else if( diff_samples == -2 ) {
        // send new data in two parts, each with last sample removed
        size /= AUDIO_SAMPLE_SIZE;
        size /= 2;
        size -= 1;
        size *= AUDIO_SAMPLE_SIZE;
        xRingbufferSend(s_ringbuf_i2s, (void *)data, size, (portTickType)portMAX_DELAY);
        data += size + AUDIO_SAMPLE_SIZE;
    }
    else if( diff_samples == -1 ) {
        // just last sample removed
        size -= AUDIO_SAMPLE_SIZE;
    } 
    
    BaseType_t done = xRingbufferSend(s_ringbuf_i2s, (void *)data, size, (portTickType)portMAX_DELAY);
    if(done){
        return ret;  // number of processed bytes, including dropped
    } else {
        return 0;
    }
}
