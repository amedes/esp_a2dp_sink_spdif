/*
    This example code is in the Public Domain (or CC0 licensed, at your option.)

    Unless required by applicable law or agreed to in writing, this
    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
*/

#include "freertos/FreeRTOS.h"
#include "driver/i2s.h"

#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_SPDIF

#define I2S_NUM			(0)

#define I2S_BITS_PER_SAMPLE	(32)
#define I2S_CHANNELS		2
#define BMC_BITS_PER_SAMPLE	64
#define BMC_BITS_FACTOR		(BMC_BITS_PER_SAMPLE / I2S_BITS_PER_SAMPLE)
#define SPDIF_BLOCK_SAMPLES	192
#define SPDIF_BUF_DIV		2	// double buffering
#define DMA_BUF_COUNT		2
#define DMA_BUF_LEN		(SPDIF_BLOCK_SAMPLES * BMC_BITS_PER_SAMPLE / I2S_BITS_PER_SAMPLE / SPDIF_BUF_DIV)
#define I2S_BUG_MAGIC		(26 * 1000 * 1000)	// magic number for avoiding I2S bug
#define SPDIF_BLOCK_SIZE	(SPDIF_BLOCK_SAMPLES * (BMC_BITS_PER_SAMPLE/8) * I2S_CHANNELS)
#define SPDIF_BUF_SIZE		(SPDIF_BLOCK_SIZE / SPDIF_BUF_DIV)
#define SPDIF_BUF_ARRAY_SIZE	(SPDIF_BUF_SIZE / sizeof(uint32_t))

static uint32_t spdif_buf[SPDIF_BUF_ARRAY_SIZE];
static uint32_t *spdif_ptr;

/*
 * 8bit PCM to 16bit BMC conversion table, LSb first, 0 end
 */
static const uint16_t bmc_tab[256] = {
    0xcccc, 0x4ccc, 0x2ccc, 0xaccc, 0x34cc, 0xb4cc, 0xd4cc, 0x54cc,
    0x32cc, 0xb2cc, 0xd2cc, 0x52cc, 0xcacc, 0x4acc, 0x2acc, 0xaacc,
    0x334c, 0xb34c, 0xd34c, 0x534c, 0xcb4c, 0x4b4c, 0x2b4c, 0xab4c,
    0xcd4c, 0x4d4c, 0x2d4c, 0xad4c, 0x354c, 0xb54c, 0xd54c, 0x554c,
    0x332c, 0xb32c, 0xd32c, 0x532c, 0xcb2c, 0x4b2c, 0x2b2c, 0xab2c,
    0xcd2c, 0x4d2c, 0x2d2c, 0xad2c, 0x352c, 0xb52c, 0xd52c, 0x552c,
    0xccac, 0x4cac, 0x2cac, 0xacac, 0x34ac, 0xb4ac, 0xd4ac, 0x54ac,
    0x32ac, 0xb2ac, 0xd2ac, 0x52ac, 0xcaac, 0x4aac, 0x2aac, 0xaaac,
    0x3334, 0xb334, 0xd334, 0x5334, 0xcb34, 0x4b34, 0x2b34, 0xab34,
    0xcd34, 0x4d34, 0x2d34, 0xad34, 0x3534, 0xb534, 0xd534, 0x5534,
    0xccb4, 0x4cb4, 0x2cb4, 0xacb4, 0x34b4, 0xb4b4, 0xd4b4, 0x54b4,
    0x32b4, 0xb2b4, 0xd2b4, 0x52b4, 0xcab4, 0x4ab4, 0x2ab4, 0xaab4,
    0xccd4, 0x4cd4, 0x2cd4, 0xacd4, 0x34d4, 0xb4d4, 0xd4d4, 0x54d4,
    0x32d4, 0xb2d4, 0xd2d4, 0x52d4, 0xcad4, 0x4ad4, 0x2ad4, 0xaad4,
    0x3354, 0xb354, 0xd354, 0x5354, 0xcb54, 0x4b54, 0x2b54, 0xab54,
    0xcd54, 0x4d54, 0x2d54, 0xad54, 0x3554, 0xb554, 0xd554, 0x5554,
    0x3332, 0xb332, 0xd332, 0x5332, 0xcb32, 0x4b32, 0x2b32, 0xab32,
    0xcd32, 0x4d32, 0x2d32, 0xad32, 0x3532, 0xb532, 0xd532, 0x5532,
    0xccb2, 0x4cb2, 0x2cb2, 0xacb2, 0x34b2, 0xb4b2, 0xd4b2, 0x54b2,
    0x32b2, 0xb2b2, 0xd2b2, 0x52b2, 0xcab2, 0x4ab2, 0x2ab2, 0xaab2,
    0xccd2, 0x4cd2, 0x2cd2, 0xacd2, 0x34d2, 0xb4d2, 0xd4d2, 0x54d2,
    0x32d2, 0xb2d2, 0xd2d2, 0x52d2, 0xcad2, 0x4ad2, 0x2ad2, 0xaad2,
    0x3352, 0xb352, 0xd352, 0x5352, 0xcb52, 0x4b52, 0x2b52, 0xab52,
    0xcd52, 0x4d52, 0x2d52, 0xad52, 0x3552, 0xb552, 0xd552, 0x5552,
    0xccca, 0x4cca, 0x2cca, 0xacca, 0x34ca, 0xb4ca, 0xd4ca, 0x54ca,
    0x32ca, 0xb2ca, 0xd2ca, 0x52ca, 0xcaca, 0x4aca, 0x2aca, 0xaaca,
    0x334a, 0xb34a, 0xd34a, 0x534a, 0xcb4a, 0x4b4a, 0x2b4a, 0xab4a,
    0xcd4a, 0x4d4a, 0x2d4a, 0xad4a, 0x354a, 0xb54a, 0xd54a, 0x554a,
    0x332a, 0xb32a, 0xd32a, 0x532a, 0xcb2a, 0x4b2a, 0x2b2a, 0xab2a,
    0xcd2a, 0x4d2a, 0x2d2a, 0xad2a, 0x352a, 0xb52a, 0xd52a, 0x552a,
    0xccaa, 0x4caa, 0x2caa, 0xacaa, 0x34aa, 0xb4aa, 0xd4aa, 0x54aa,
    0x32aa, 0xb2aa, 0xd2aa, 0x52aa, 0xcaaa, 0x4aaa, 0x2aaa, 0xaaaa,
};

// BMC preamble
#define BMC_B		0xcce8cccc	// block start
#define BMC_M		0xcce2cccc	// left ch
#define BMC_W		0xcce4cccc	// right ch
#define BMC_MW_DIF	(BMC_M ^ BMC_W)
#define SYNC_OFFSET	2		// byte offset of SYNC
#define SYNC_FLIP	((BMC_B ^ BMC_M) >> (SYNC_OFFSET * 8))
#define SET_MSb		(1 << 31)

// initialize S/PDIF buffer
static void spdif_buf_init(void)
{
    int i;
    uint32_t bmc_mw = BMC_W;

    for (i = 0; i < SPDIF_BUF_ARRAY_SIZE; i += 2) {
	spdif_buf[i] = bmc_mw ^= BMC_MW_DIF;
    }
}

// initialize I2S for S/PDIF transmission
void spdif_init(int rate)
{
    int sample_rate = rate * BMC_BITS_FACTOR;
    int bclk = sample_rate * I2S_BITS_PER_SAMPLE * I2S_CHANNELS;
    int mclk = (I2S_BUG_MAGIC / bclk) * bclk; // use mclk for avoiding I2S bug
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
    	.sample_rate = sample_rate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = DMA_BUF_COUNT,
        .dma_buf_len = DMA_BUF_LEN,
        .use_apll = true,
	.tx_desc_auto_clear = true,
    	.fixed_mclk = mclk,	// avoiding I2S bug
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = -1,
        .ws_io_num = -1,
        .data_out_num = CONFIG_SPDIF_DATA_PIN,
        .data_in_num = -1,
    };

    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM, &pin_config));

    // initialize S/PDIF buffer
    spdif_buf_init();
    spdif_ptr = &spdif_buf[1];
}

// write audio data to I2S buffer
void spdif_write(const void *src, size_t size)
{
    const uint16_t *p = src;

    while (p < (uint16_t *)src + size / 2) {

	// convert PCM 16bit data to BMC 32bit pulse pattern
	*spdif_ptr = SET_MSb | ((~bmc_tab[(uint8_t)*p] << 16) ^ (int16_t)bmc_tab[*p >> 8]);
	p++;
	spdif_ptr += 2; 	// advance to next audio data
 
	if (spdif_ptr >= &spdif_buf[SPDIF_BUF_ARRAY_SIZE]) {
    	    size_t i2s_write_len;

	    // set block start preamble
	    ((uint8_t *)spdif_buf)[SYNC_OFFSET] ^= SYNC_FLIP;

	    i2s_write(I2S_NUM, spdif_buf, sizeof(spdif_buf), &i2s_write_len, portMAX_DELAY);

	    spdif_ptr = &spdif_buf[1];
	}
    }
}

// change S/PDIF sample rate
void spdif_set_sample_rates(int rate)
{
    // uninstall and reinstall I2S driver for avoiding I2S bug
    i2s_driver_uninstall(I2S_NUM);
    spdif_init(rate);
}

#endif // SPDIF
