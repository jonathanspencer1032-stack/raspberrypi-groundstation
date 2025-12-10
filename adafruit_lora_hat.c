#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <gpiod.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

struct gpio_line {
    struct gpiod_line_request *req;
    unsigned int offset;
};


static void gpio_line_release(struct gpio_line *line) {
    if (line && line->req) {
        gpiod_line_request_release(line->req);
        line->req = NULL;
    }
}

static int gpio_line_setup(struct gpio_line *line,
                           struct gpiod_chip *chip,
                           unsigned int offset,
                           bool output,
                           int initval,
                           bool pull_up) {
    struct gpiod_line_settings *settings = gpiod_line_settings_new();
    struct gpiod_line_config *config = NULL;
    struct gpiod_request_config *req_cfg = NULL;
    if (!settings) return -1;

    if (gpiod_line_settings_set_direction(settings,
            output ? GPIOD_LINE_DIRECTION_OUTPUT : GPIOD_LINE_DIRECTION_INPUT) < 0) {
        goto fail;
    }

    if (output) {
        enum gpiod_line_value val = initval ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;
        if (gpiod_line_settings_set_output_value(settings, val) < 0) goto fail;
    } else {
        if (pull_up) gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_UP);
    }

    config = gpiod_line_config_new();
    if (!config) goto fail;
    if (gpiod_line_config_add_line_settings(config, &offset, 1, settings) < 0) goto fail;

    req_cfg = gpiod_request_config_new();
    if (!req_cfg) goto fail;
    gpiod_request_config_set_consumer(req_cfg, "adafruit_lora_hat");

    struct gpiod_line_request *req = gpiod_chip_request_lines(chip, req_cfg, config);
    if (!req) goto fail;

    line->req = req;
    line->offset = offset;

    gpiod_request_config_free(req_cfg);
    gpiod_line_config_free(config);
    gpiod_line_settings_free(settings);
    return 0;

fail:
    if (req_cfg) gpiod_request_config_free(req_cfg);
    if (config) gpiod_line_config_free(config);
    if (settings) gpiod_line_settings_free(settings);
    return -1;
}

static int gpio_line_set_value(struct gpio_line *line, int value) {
    if (!line || !line->req) return -1;
    enum gpiod_line_value val = value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;
    return gpiod_line_request_set_value(line->req, line->offset, val);
}

static int gpio_line_get_value(struct gpio_line *line) {
    if (!line || !line->req) return -1;
    enum gpiod_line_value val = gpiod_line_request_get_value(line->req, line->offset);
    if (val == GPIOD_LINE_VALUE_ERROR) return -1;
    return val == GPIOD_LINE_VALUE_ACTIVE ? 1 : 0;
}

// ---------------------------------------------------------------------------
// Hardware layout for Adafruit LoRa Radio Bonnet (RFM95W + SSD1306 128x32 OLED)
// ---------------------------------------------------------------------------
static const char *SPI_DEV = "/dev/spidev0.1";  // CE1 on Pi header
static const char *I2C_DEV = "/dev/i2c-1";
static const char *GPIO_CHIP = "/dev/gpiochip0";
static const int OLED_ADDR = 0x3C;

#define PIN_RFM95_RST  25
#define PIN_RFM95_DIO0 22
#define PIN_OLED_RST    4
#define PIN_BTN_A       5
#define PIN_BTN_B       6
#define PIN_BTN_C      12

#define RADIO_FREQ_MHZ 915.0
#define TX_POWER_DBM   20

#define PERIOD_SLOW 5.0
#define PERIOD_FAST 1.0

// ---------------------------------------------------------------------------
// Very small 5x7 font (public domain, ASCII 32-127)
// ---------------------------------------------------------------------------
static const uint8_t font5x7[96][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5f,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},{0x14,0x7f,0x14,0x7f,0x14},
    {0x24,0x2a,0x7f,0x2a,0x12},{0x23,0x13,0x08,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1c,0x22,0x41,0x00},{0x00,0x41,0x22,0x1c,0x00},{0x14,0x08,0x3e,0x08,0x14},{0x08,0x08,0x3e,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
    {0x3e,0x51,0x49,0x45,0x3e},{0x00,0x42,0x7f,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4b,0x31},
    {0x18,0x14,0x12,0x7f,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3c,0x4a,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1e},{0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},{0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3e},{0x7e,0x11,0x11,0x11,0x7e},{0x7f,0x49,0x49,0x49,0x36},{0x3e,0x41,0x41,0x41,0x22},
    {0x7f,0x41,0x41,0x22,0x1c},{0x7f,0x49,0x49,0x49,0x41},{0x7f,0x09,0x09,0x09,0x01},{0x3e,0x41,0x49,0x49,0x7a},
    {0x7f,0x08,0x08,0x08,0x7f},{0x00,0x41,0x7f,0x41,0x00},{0x20,0x40,0x41,0x3f,0x01},{0x7f,0x08,0x14,0x22,0x41},
    {0x7f,0x40,0x40,0x40,0x40},{0x7f,0x02,0x0c,0x02,0x7f},{0x7f,0x04,0x08,0x10,0x7f},{0x3e,0x41,0x41,0x41,0x3e},
    {0x7f,0x09,0x09,0x09,0x06},{0x3e,0x41,0x51,0x21,0x5e},{0x7f,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7f,0x01,0x01},{0x3f,0x40,0x40,0x40,0x3f},{0x1f,0x20,0x40,0x20,0x1f},{0x3f,0x40,0x38,0x40,0x3f},
    {0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7f,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7f,0x00},{0x04,0x02,0x01,0x02,0x04},{0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},{0x7f,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7f},{0x38,0x54,0x54,0x54,0x18},{0x08,0x7e,0x09,0x01,0x02},{0x0c,0x52,0x52,0x52,0x3e},
    {0x7f,0x08,0x04,0x04,0x78},{0x00,0x44,0x7d,0x40,0x00},{0x20,0x40,0x44,0x3d,0x00},{0x7f,0x10,0x28,0x44,0x00},
    {0x00,0x41,0x7f,0x40,0x00},{0x7c,0x04,0x18,0x04,0x78},{0x7c,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},
    {0x7c,0x14,0x14,0x14,0x08},{0x08,0x14,0x14,0x18,0x7c},{0x7c,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3f,0x44,0x40,0x20},{0x3c,0x40,0x40,0x20,0x7c},{0x1c,0x20,0x40,0x20,0x1c},{0x3c,0x40,0x30,0x40,0x3c},
    {0x44,0x28,0x10,0x28,0x44},{0x0c,0x50,0x50,0x50,0x3c},{0x44,0x64,0x54,0x4c,0x44},{0x00,0x08,0x36,0x41,0x00},
    {0x00,0x00,0x7f,0x00,0x00},{0x00,0x41,0x36,0x08,0x00},{0x10,0x08,0x08,0x10,0x08}
};
#define FONT(c) (font5x7[(c) - 32])

// ---------------------------------------------------------------------------
// OLED helpers
// ---------------------------------------------------------------------------
#define OLED_W 128
#define OLED_H 32
static uint8_t oled_buf[OLED_W * (OLED_H / 8)];
static int i2c_fd = -1;

static void oled_cmd(uint8_t c) {
    uint8_t frame[2] = {0x00, c};
    write(i2c_fd, frame, sizeof(frame));
}

static void oled_data(const uint8_t *d, size_t n) {
    uint8_t packet[17];
    packet[0] = 0x40;
    size_t idx = 0;
    while (idx < n) {
        size_t chunk = (n - idx > 16) ? 16 : (n - idx);
        memcpy(&packet[1], d + idx, chunk);
        write(i2c_fd, packet, chunk + 1);
        idx += chunk;
    }
}

static void oled_hw_reset(struct gpio_line *rst) {
    gpio_line_set_value(rst, 0);
    usleep(10000);
    gpio_line_set_value(rst, 1);
    usleep(10000);
}

static void oled_init(struct gpio_line *rst) {
    oled_hw_reset(rst);
    oled_cmd(0xAE);
    oled_cmd(0xD5); oled_cmd(0x80);
    oled_cmd(0xA8); oled_cmd(0x1F);
    oled_cmd(0xD3); oled_cmd(0x00);
    oled_cmd(0x40);
    oled_cmd(0x8D); oled_cmd(0x14);
    oled_cmd(0x20); oled_cmd(0x00);
    oled_cmd(0xA1);
    oled_cmd(0xC8);
    oled_cmd(0xDA); oled_cmd(0x02);
    oled_cmd(0x81); oled_cmd(0x8F);
    oled_cmd(0xD9); oled_cmd(0xF1);
    oled_cmd(0xDB); oled_cmd(0x40);
    oled_cmd(0xA4);
    oled_cmd(0xA6);
    oled_cmd(0xAF);
}

static void oled_clear(void) {
    memset(oled_buf, 0, sizeof(oled_buf));
}

static void oled_draw_char(int x, int y, char c) {
    if (c < 32 || c > 127) c = '?';
    const uint8_t *glyph = FONT(c);
    for (int col = 0; col < 5; ++col) {
        uint8_t data = glyph[col];
        for (int row = 0; row < 7; ++row) {
            if (!(data & (1 << row))) continue;
            int px = x + col;
            int py = y + row;
            if (px < 0 || px >= OLED_W || py < 0 || py >= OLED_H) continue;
            int idx = px + (py / 8) * OLED_W;
            oled_buf[idx] |= (1 << (py & 7));
        }
    }
}

static void oled_draw_text(int x, int y, const char *text) {
    int cursor = x;
    while (*text) {
        oled_draw_char(cursor, y, *text++);
        cursor += 6;
    }
}

static void oled_show(void) {
    oled_cmd(0x21); oled_cmd(0);   oled_cmd(127);
    oled_cmd(0x22); oled_cmd(0);   oled_cmd(3);
    oled_data(oled_buf, sizeof(oled_buf));
}

// ---------------------------------------------------------------------------
// SPI helpers
// ---------------------------------------------------------------------------
static int spi_fd = -1;
static uint8_t spi_mode = SPI_MODE_0;
static uint32_t spi_speed = 8000000;
static uint8_t spi_bits = 8;

static int spi_init(const char *dev) {
    int fd = open(dev, O_RDWR);
    if (fd < 0) {
        perror("open spidev");
        return -1;
    }
    if (ioctl(fd, SPI_IOC_WR_MODE, &spi_mode) < 0) perror("SPI mode");
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits) < 0) perror("SPI bits");
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0) perror("SPI speed");
    return fd;
}

static uint8_t sx127x_read_reg(uint8_t addr) {
    uint8_t tx[2] = { (uint8_t)(addr & 0x7F), 0x00 };
    uint8_t rx[2] = {0};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 2,
        .speed_hz = spi_speed,
        .bits_per_word = spi_bits
    };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    return rx[1];
}

static void sx127x_write_reg(uint8_t addr, uint8_t value) {
    uint8_t tx[2] = { (uint8_t)(addr | 0x80), value };
    uint8_t rx[2] = {0};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 2,
        .speed_hz = spi_speed,
        .bits_per_word = spi_bits
    };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
}

static void sx127x_burst_write(uint8_t addr, const uint8_t *data, size_t len) {
    uint8_t *tx = malloc(len + 1);
    uint8_t *rx = calloc(1, len + 1);
    tx[0] = addr | 0x80;
    memcpy(tx + 1, data, len);
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = len + 1,
        .speed_hz = spi_speed,
        .bits_per_word = spi_bits
    };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    free(tx);
    free(rx);
}

static void sx127x_burst_read(uint8_t addr, uint8_t *data, size_t len) {
    uint8_t *tx = calloc(1, len + 1);
    uint8_t *rx = calloc(1, len + 1);
    tx[0] = addr & 0x7F;
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = len + 1,
        .speed_hz = spi_speed,
        .bits_per_word = spi_bits
    };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    memcpy(data, rx + 1, len);
    free(tx);
    free(rx);
}

// ---------------------------------------------------------------------------
// SX127x register map
// ---------------------------------------------------------------------------
#define REG_FIFO            0x00
#define REG_OPMODE          0x01
#define REG_FRFMSB          0x06
#define REG_FRFMID          0x07
#define REG_FRFLSB          0x08
#define REG_PA_CONFIG       0x09
#define REG_LNA             0x0C
#define REG_FIFO_ADDR_PTR   0x0D
#define REG_FIFO_TX_BASE    0x0E
#define REG_FIFO_RX_BASE    0x0F
#define REG_FIFO_RX_CURRENT 0x10
#define REG_IRQ_FLAGS       0x12
#define REG_RX_NB_BYTES     0x13
#define REG_PKT_SNR         0x19
#define REG_PKT_RSSI        0x1A
#define REG_MODEM_CONFIG1   0x1D
#define REG_MODEM_CONFIG2   0x1E
#define REG_PREAMBLE_MSB    0x20
#define REG_PREAMBLE_LSB    0x21
#define REG_PAYLOAD_LEN     0x22
#define REG_MODEM_CONFIG3   0x26
#define REG_DIO_MAPPING1    0x40
#define REG_VERSION         0x42
#define REG_PA_DAC          0x4D

#define OPMODE_LONG_RANGE_MODE (1 << 7)
#define OPMODE_SLEEP  0x00
#define OPMODE_STDBY  0x01
#define OPMODE_TX     0x03
#define OPMODE_RXCONT 0x05

static void radio_sleep(void)   { sx127x_write_reg(REG_OPMODE, OPMODE_LONG_RANGE_MODE | OPMODE_SLEEP); }
static void radio_standby(void) { sx127x_write_reg(REG_OPMODE, OPMODE_LONG_RANGE_MODE | OPMODE_STDBY); }
static void radio_tx(void)      { sx127x_write_reg(REG_OPMODE, OPMODE_LONG_RANGE_MODE | OPMODE_TX); }
static void radio_rx_cont(void) { sx127x_write_reg(REG_OPMODE, OPMODE_LONG_RANGE_MODE | OPMODE_RXCONT); }

static void radio_set_freq(double mhz) {
    uint32_t frf = (uint32_t)((mhz * 1000000.0) / 61.03515625);
    sx127x_write_reg(REG_FRFMSB, (frf >> 16) & 0xFF);
    sx127x_write_reg(REG_FRFMID, (frf >> 8) & 0xFF);
    sx127x_write_reg(REG_FRFLSB, frf & 0xFF);
}

static void radio_set_tx_power(int dbm) {
    int out = dbm - 2;
    if (out < 0) out = 0;
    if (out > 15) out = 15;
    sx127x_write_reg(REG_PA_CONFIG, 0x80 | (uint8_t)out);
    sx127x_write_reg(REG_PA_DAC, dbm > 17 ? 0x87 : 0x84);
}

static void radio_set_modem(int bw_khz, int sf, int cr_denom) {
    uint8_t bw_code = (bw_khz == 125) ? 0x70 : (bw_khz == 250) ? 0x80 : 0x90;
    uint8_t cr_code = (uint8_t)(((cr_denom - 4) & 0x07) << 1);
    sx127x_write_reg(REG_MODEM_CONFIG1, bw_code | cr_code);
    uint8_t sf_code = (uint8_t)((sf << 4) & 0xF0);
    sx127x_write_reg(REG_MODEM_CONFIG2, sf_code | (1 << 2) | 0x03);
    sx127x_write_reg(REG_MODEM_CONFIG3, (0 << 3) | (1 << 2));
}

static void radio_tx_payload(const uint8_t *payload, uint8_t len) {
    sx127x_write_reg(REG_IRQ_FLAGS, 0xFF);
    sx127x_write_reg(REG_FIFO_TX_BASE, 0x00);
    sx127x_write_reg(REG_FIFO_ADDR_PTR, 0x00);
    sx127x_burst_write(REG_FIFO, payload, len);
    sx127x_write_reg(REG_PAYLOAD_LEN, len);
    radio_tx();
    for (int i = 0; i < 2000; ++i) {
        uint8_t flags = sx127x_read_reg(REG_IRQ_FLAGS);
        if (flags & 0x08) {
            sx127x_write_reg(REG_IRQ_FLAGS, 0x08);
            break;
        }
        usleep(1000);
    }
    radio_standby();
}

static int radio_receive_packet(uint8_t *buf, size_t maxlen, int *rssi_dbm, float *snr_db) {
    uint8_t irq = sx127x_read_reg(REG_IRQ_FLAGS);
    if (!(irq & 0x40)) return 0; // no RxDone
    if (irq & 0x20) {
        sx127x_write_reg(REG_IRQ_FLAGS, 0xFF);
        return -1;
    }

    uint8_t current = sx127x_read_reg(REG_FIFO_RX_CURRENT);
    uint8_t bytes = sx127x_read_reg(REG_RX_NB_BYTES);
    sx127x_write_reg(REG_FIFO_ADDR_PTR, current);

    if (bytes > maxlen) bytes = (uint8_t)maxlen;
    sx127x_burst_read(REG_FIFO, buf, bytes);

    int8_t snr_raw = (int8_t)sx127x_read_reg(REG_PKT_SNR);
    if (snr_db) *snr_db = snr_raw / 4.0f;
    int16_t pkt_rssi = sx127x_read_reg(REG_PKT_RSSI);
    if (rssi_dbm) *rssi_dbm = pkt_rssi - 157;

    sx127x_write_reg(REG_IRQ_FLAGS, 0xFF);
    return bytes;
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
int main(void) {
    struct gpiod_chip *chip = gpiod_chip_open(GPIO_CHIP);
    if (!chip) {
        perror("open gpiochip");
        return 1;
    }

    struct gpio_line rst_radio = {0}, rst_oled = {0};
    struct gpio_line btn_a = {0}, btn_b = {0}, btn_c = {0};
    if (gpio_line_setup(&rst_radio, chip, PIN_RFM95_RST, true, 1, false) < 0 ||
        gpio_line_setup(&rst_oled,  chip, PIN_OLED_RST,  true, 1, false) < 0 ||
        gpio_line_setup(&btn_a, chip, PIN_BTN_A, false, 0, true) < 0 ||
        gpio_line_setup(&btn_b, chip, PIN_BTN_B, false, 0, true) < 0 ||
        gpio_line_setup(&btn_c, chip, PIN_BTN_C, false, 0, true) < 0) {
        fprintf(stderr, "Failed to request GPIO lines\n");
        return 1;
    }

    i2c_fd = open(I2C_DEV, O_RDWR);
    if (i2c_fd < 0) {
        perror("open i2c");
        return 1;
    }
    if (ioctl(i2c_fd, I2C_SLAVE, OLED_ADDR) < 0) {
        perror("i2c addr");
        return 1;
    }
    oled_init(&rst_oled);
    oled_clear();
    oled_draw_text(0, 0, "LoRa init...");
    oled_show();

    spi_fd = spi_init(SPI_DEV);
    if (spi_fd < 0) return 1;

    gpio_line_set_value(&rst_radio, 0); usleep(10000);
    gpio_line_set_value(&rst_radio, 1); usleep(10000);

    radio_sleep();
    usleep(10000);
    radio_standby();

    uint8_t version = sx127x_read_reg(REG_VERSION);
    radio_set_freq(RADIO_FREQ_MHZ);
    radio_set_tx_power(TX_POWER_DBM);
    radio_set_modem(125, 7, 5);
    sx127x_write_reg(REG_LNA, 0x23);
    sx127x_write_reg(REG_PREAMBLE_MSB, 0x00);
    sx127x_write_reg(REG_PREAMBLE_LSB, 0x08);
    sx127x_write_reg(REG_FIFO_TX_BASE, 0x00);
    sx127x_write_reg(REG_FIFO_RX_BASE, 0x80);

    char header[32];
    snprintf(header, sizeof(header), "RFM95 v0x%02X", version);
    oled_clear();
    oled_draw_text(0, 0, header);
    oled_draw_text(0, 8, "RX mode ready");
    oled_draw_text(0, 16, "A:TX  B:stats");
    oled_show();

    radio_rx_cont();

    int tx_counter = 0;
    uint8_t rx_buf[256];
    char last_text[64] = "";
    int last_rssi = 0;
    float last_snr = 0.0f;
    bool show_stats = false;

    while (1) {
        bool a_pressed = gpio_line_get_value(&btn_a) == 0;
        bool b_pressed = gpio_line_get_value(&btn_b) == 0;
        bool c_pressed = gpio_line_get_value(&btn_c) == 0;

        if (a_pressed) {
            char payload[48];
            int len = snprintf(payload, sizeof(payload), "Pi TX #%d", tx_counter++);
            if (len < 0) len = 0;
            if (len > 47) len = 47;
            radio_tx_payload((const uint8_t *)payload, (uint8_t)len);
            radio_rx_cont();

            oled_clear();
            oled_draw_text(0, 0, "TX sent");
            oled_draw_text(0, 8, payload);
            oled_draw_text(0, 16, "Listening...");
            oled_show();
            usleep(200000);
        }

        if (b_pressed) {
            show_stats = !show_stats;
            if (show_stats) {
                char l1[24], l2[24];
                snprintf(l1, sizeof(l1), "RSSI %d dBm", last_rssi);
                snprintf(l2, sizeof(l2), "SNR %.1f dB", last_snr);
                oled_clear();
                oled_draw_text(0, 0, "Show stats");
                oled_draw_text(0, 8, l1);
                oled_draw_text(0, 16, l2);
            } else {
                oled_clear();
                oled_draw_text(0, 0, "Show payload");
                oled_draw_text(0, 8, last_text);
            }
            oled_show();
            usleep(250000);
        }

        if (c_pressed) {
            last_text[0] = '\0';
            oled_clear();
            oled_draw_text(0, 0, "Display reset");
            oled_draw_text(0, 8, "Waiting packets");
            oled_show();
            usleep(250000);
        }

        int rssi_dbm = 0;
        float snr_db = 0.0f;
        int rx_len = radio_receive_packet(rx_buf, sizeof(rx_buf) - 1, &rssi_dbm, &snr_db);
        if (rx_len > 0) {
            size_t copy = (size_t)rx_len;
            if (copy > sizeof(last_text) - 1) copy = sizeof(last_text) - 1;
            for (size_t i = 0; i < copy; ++i) {
                last_text[i] = isprint(rx_buf[i]) ? (char)rx_buf[i] : '.';
            }
            last_text[copy] = '\0';
            last_rssi = rssi_dbm;
            last_snr = snr_db;

            char disp1[22]={0}, disp2[22]={0}, stats_line[24];
            strncpy(disp1, last_text, sizeof(disp1)-1);
            if (strlen(last_text) > sizeof(disp1)-1) {
                strncpy(disp2, last_text + sizeof(disp1)-1, sizeof(disp2)-1);
            }
            snprintf(stats_line, sizeof(stats_line), "R:%d S:%.1f", last_rssi, last_snr);

            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            printf("[%ld.%03ld] RX %dB RSSI %d dBm SNR %.1f dB | %s\n",
                   ts.tv_sec, ts.tv_nsec / 1000000L, rx_len, rssi_dbm, snr_db, last_text);
            fflush(stdout);

            if (show_stats) {
                char l1[24], l2[24];
                snprintf(l1, sizeof(l1), "RSSI %d dBm", last_rssi);
                snprintf(l2, sizeof(l2), "SNR %.1f dB", last_snr);
                oled_clear();
                oled_draw_text(0, 0, "RX stats");
                oled_draw_text(0, 8, l1);
                oled_draw_text(0, 16, l2);
            } else {
                oled_clear();
                oled_draw_text(0, 0, "RX packet");
                oled_draw_text(0, 8, disp1);
                if (disp2[0]) {
                    oled_draw_text(0, 16, disp2);
                } else {
                    oled_draw_text(0, 16, stats_line);
                }
            }
            oled_show();
        }

        usleep(10000);
    }

    close(spi_fd);
    close(i2c_fd);
    gpio_line_release(&rst_radio);
    gpio_line_release(&rst_oled);
    gpio_line_release(&btn_a);
    gpio_line_release(&btn_b);
    gpio_line_release(&btn_c);
    gpiod_chip_close(chip);
    return 0;
}
