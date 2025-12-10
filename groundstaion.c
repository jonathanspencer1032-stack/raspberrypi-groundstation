// Raspberry Pi 4 + Adafruit RFM95W + OLED Bonnet (PID 4074)
// - SPI: RFM95 on /dev/spidev0.1 (CE1)
// - RESET (RFM95): GPIO25
// - DIO0 (unused here; polling): GPIO22 (bonnet ties it)
// - OLED: SSD1306 128x32 on I2C-1 (0x3C), RST on GPIO4
// - Buttons: A=GPIO5, B=GPIO6, C=GPIO12 (pull-ups on bonnet)
// Build: gcc -O2 -o pi_lora_oled_tx pi_lora_oled_tx.c -lgpiod
// Run:   sudo ./pi_lora_oled_tx
// NOTE: Set your region freq (US 915.0, EU 868.0, etc.) and obey RF regs.

#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/i2c-dev.h>
#include <gpiod.h>

// -------------------- Config --------------------
static const char *SPI_DEV   = "/dev/spidev0.1"; // CE1
static const char *I2C_DEV   = "/dev/i2c-1";
static const int   I2C_ADDR  = 0x3C;             // SSD1306
static const char *GPIO_CHIP = "/dev/gpiochip0";

// Bonnet pins
#define PIN_RFM95_RST  25
#define PIN_RFM95_DIO0 22   // not mandatory here
#define PIN_OLED_RST    4
#define PIN_BTN_A       5
#define PIN_BTN_B       6
#define PIN_BTN_C      12

// Radio region freq (MHz)
#define RADIO_FREQ_MHZ 915.0

// TX power (dBm); SX127x +20 dBm requires PA_BOOST path; obey your regs
#define TX_POWER_DBM 20

// TX cadence
#define PERIOD_SLOW 5.0
#define PERIOD_FAST 1.0

// -------------------- Minimal 5x7 font (ASCII 32..127) --------------------
// 5 bytes/char, MSB=top row; rendered into 128x32 buffer. (Common public-domain 5x7)
static const uint8_t font5x7[][5] = {
#include <stddef.h>
};
#undef offsetof
// To keep the snippet compact in this answer box, the font table is injected below:
static const uint8_t font_data[96][5] = {
  {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},{0x14,0x7F,0x14,0x7F,0x14},
  {0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
  {0x00,0x1C,0x22,0x41,0x00},{0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
  {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
  {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
  {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
  {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},
  {0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},{0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
  {0x32,0x49,0x79,0x41,0x3E},{0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
  {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},
  {0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},
  {0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
  {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},
  {0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},
  {0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
  {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},{0x40,0x40,0x40,0x40,0x40},
  {0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},{0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},
  {0x38,0x44,0x44,0x48,0x7F},{0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},
  {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},{0x7F,0x10,0x28,0x44,0x00},
  {0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},{0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},
  {0x7C,0x14,0x14,0x14,0x08},{0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
  {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},{0x3C,0x40,0x30,0x40,0x3C},
  {0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},{0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},
  {0x00,0x00,0x7F,0x00,0x00},{0x00,0x41,0x36,0x08,0x00},{0x10,0x08,0x08,0x10,0x08}
};
#define FONT(c) (font_data[(c) - 32])

// -------------------- OLED (SSD1306 128x32) --------------------
#define OLED_W 128
#define OLED_H 32
static uint8_t oled_buf[OLED_W * (OLED_H/8)]; // 128 * 4 pages

static int i2c_fd = -1;

static void oled_cmd(uint8_t c){
    uint8_t buf[2] = {0x00, c};
    write(i2c_fd, buf, 2);
}

static void oled_data(const uint8_t *d, size_t n){
    // 0x40 = data stream prefix
    size_t i=0;
    uint8_t pkt[17];
    pkt[0] = 0x40;
    while(i<n){
        size_t chunk = (n - i > 16) ? 16 : (n - i);
        memcpy(pkt+1, d+i, chunk);
        write(i2c_fd, pkt, chunk+1);
        i += chunk;
    }
}

static void oled_init(int rst_line, struct gpiod_line *rst){
    // Hardware reset pulse
    gpiod_line_set_value(rst, 0); usleep(10000);
    gpiod_line_set_value(rst, 1); usleep(10000);

    oled_cmd(0xAE); // display off
    oled_cmd(0xD5); oled_cmd(0x80);           // clock
    oled_cmd(0xA8); oled_cmd(0x1F);           // multiplex for 32
    oled_cmd(0xD3); oled_cmd(0x00);           // offset
    oled_cmd(0x40);                           // start line
    oled_cmd(0x8D); oled_cmd(0x14);           // charge pump
    oled_cmd(0x20); oled_cmd(0x00);           // horizontal addressing
    oled_cmd(0xA1);                           // segment remap
    oled_cmd(0xC8);                           // COM scan dec
    oled_cmd(0xDA); oled_cmd(0x02);           // compins for 128x32
    oled_cmd(0x81); oled_cmd(0x8F);           // contrast
    oled_cmd(0xD9); oled_cmd(0xF1);           // precharge
    oled_cmd(0xDB); oled_cmd(0x40);           // vcomh
    oled_cmd(0xA4);                           // resume
    oled_cmd(0xA6);                           // normal display
    oled_cmd(0x2E);                           // deactivate scroll
    oled_cmd(0xAF);                           // display on
}

static void oled_clear(){
    memset(oled_buf, 0x00, sizeof(oled_buf));
}

static void oled_draw_char(int x, int y, char c){
    if(c < 32 || c > 127) c = '?';
    const uint8_t *g = FONT(c);
    for(int i=0;i<5;i++){
        uint8_t col = g[i];
        for(int b=0;b<7;b++){
            int px = x + i;
            int py = y + b;
            if(px<0||px>=OLED_W||py<0||py>=OLED_H) continue;
            if(col & (1<<b)){
                int idx = px + (py/8)*OLED_W;
                oled_buf[idx] |=  (1<<(py&7));
            }
        }
    }
}

static void oled_draw_text(int x, int y, const char *s){
    int cx = x;
    while(*s){
        oled_draw_char(cx, y, *s++);
        cx += 6; // 5px + 1px space
    }
}

static void oled_show(){
    // Set column/page ranges
    oled_cmd(0x21); oled_cmd(0); oled_cmd(127);
    oled_cmd(0x22); oled_cmd(0); oled_cmd(3);
    oled_data(oled_buf, sizeof(oled_buf));
}

// -------------------- SPI (spidev) helpers --------------------
static int spi_fd = -1;
static uint8_t spi_mode = SPI_MODE_0;
static uint32_t spi_speed = 8000000; // 8 MHz is fine for SX127x
static uint8_t spi_bits = 8;

static int spi_init(const char *dev){
    int fd = open(dev, O_RDWR);
    if(fd < 0){ perror("open spidev"); return -1; }
    if(ioctl(fd, SPI_IOC_WR_MODE, &spi_mode) < 0) { perror("SPI mode"); }
    if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits) < 0) { perror("SPI bits"); }
    if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0) { perror("SPI speed"); }
    return fd;
}

static uint8_t sx127x_read_reg(uint8_t addr){
    uint8_t tx[2] = { addr & 0x7F, 0x00 };
    uint8_t rx[2] = {0};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 2,
        .speed_hz = spi_speed,
        .bits_per_word = spi_bits,
        .delay_usecs = 0
    };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    return rx[1];
}

static void sx127x_write_reg(uint8_t addr, uint8_t val){
    uint8_t tx[2] = { (uint8_t)(addr | 0x80), val };
    uint8_t rx[2] = {0};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 2,
        .speed_hz = spi_speed,
        .bits_per_word = spi_bits,
        .delay_usecs = 0
    };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
}

static void sx127x_burst_write(uint8_t addr, const uint8_t *data, size_t n){
    uint8_t *tx = malloc(n+1);
    uint8_t *rx = calloc(1, n+1);
    tx[0] = addr | 0x80;
    memcpy(tx+1, data, n);
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = n+1,
        .speed_hz = spi_speed,
        .bits_per_word = spi_bits
    };
    ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    free(tx); free(rx);
}

// -------------------- SX127x registers (subset) --------------------
#define REG_OPMODE         0x01
#define REG_FRFMSB         0x06
#define REG_FRFMID         0x07
#define REG_FRFLSB         0x08
#define REG_PA_CONFIG      0x09
#define REG_LNA            0x0C
#define REG_FIFO_ADDR_PTR  0x0D
#define REG_FIFO_TX_BASE   0x0E
#define REG_FIFO_RX_BASE   0x0F
#define REG_FIFO           0x00
#define REG_IRQ_FLAGS      0x12
#define REG_RX_NB_BYTES    0x13
#define REG_PKT_SNR        0x19
#define REG_PKT_RSSI       0x1A
#define REG_MODEM_CONFIG1  0x1D
#define REG_MODEM_CONFIG2  0x1E
#define REG_PREAMBLE_MSB   0x20
#define REG_PREAMBLE_LSB   0x21
#define REG_PAYLOAD_LEN    0x22
#define REG_MODEM_CONFIG3  0x26
#define REG_DIO_MAPPING1   0x40
#define REG_VERSION        0x42
#define REG_PA_DAC         0x4D

// OPMODE bits
#define OPMODE_LONG_RANGE_MODE  (1<<7)
#define OPMODE_SLEEP    0x00
#define OPMODE_STDBY    0x01
#define OPMODE_TX       0x03
#define OPMODE_RXCONT   0x05

static void radio_set_opmode(uint8_t mode){
    uint8_t m = sx127x_read_reg(REG_OPMODE);
    m = (m & 0xF8) | (mode & 0x07);
    sx127x_write_reg(REG_OPMODE, m);
}

static void radio_sleep(){  sx127x_write_reg(REG_OPMODE, OPMODE_LONG_RANGE_MODE | OPMODE_SLEEP); }
static void radio_standby(){sx127x_write_reg(REG_OPMODE, OPMODE_LONG_RANGE_MODE | OPMODE_STDBY); }
static void radio_tx(){     sx127x_write_reg(REG_OPMODE, OPMODE_LONG_RANGE_MODE | OPMODE_TX); }

// FRF = Freq * (2^19 / 32e6)
static void radio_set_freq(double mhz){
    uint32_t frf = (uint32_t)((mhz * 1000000.0) / 61.03515625); // 32e6 / 2^19 ≈ 61.03515625
    sx127x_write_reg(REG_FRFMSB, (frf >> 16) & 0xFF);
    sx127x_write_reg(REG_FRFMID, (frf >> 8) & 0xFF);
    sx127x_write_reg(REG_FRFLSB, (frf) & 0xFF);
}

static void radio_set_tx_power(int dbm){
    // PA_BOOST path: set PA_DAC and PA_CONFIG accordingly
    // PA_CONFIG: 0x80 (PA_BOOST) | (OutputPower & 0x0F)
    int out = dbm - 2; // rough mapping for PA_BOOST w/ PADAC=0x87 (20dBm)
    if(out < 0) out = 0;
    if(out > 15) out = 15;
    sx127x_write_reg(REG_PA_CONFIG, 0x80 | (uint8_t)out);
    if(dbm > 17) sx127x_write_reg(REG_PA_DAC, 0x87); else sx127x_write_reg(REG_PA_DAC, 0x84);
}

static void radio_set_modem_bw_sf_cr(int bw_khz, int sf, int cr_denom){
    // REG_MODEM_CONFIG1: BW[7:4], CR[3:1], ImplicitHeader[0]
    // Common BW codes: 125kHz=0x70, 250=0x80, 500=0x90
    uint8_t bw_code = (bw_khz==125)?0x70: (bw_khz==250)?0x80:0x90;
    uint8_t cr_code = ((cr_denom-4)&0x07)<<1; // CR 4/5..4/8
    sx127x_write_reg(REG_MODEM_CONFIG1, bw_code | cr_code | 0x00);
    // MODEM_CONFIG2: SF[7:4], RxPayloadCrcOn[2], SymbTimeout[1:0]
    uint8_t sf_code = (uint8_t)((sf<<4)&0xF0);
    sx127x_write_reg(REG_MODEM_CONFIG2, sf_code | (1<<2) | 0x03);
    // MODEM_CONFIG3: LowDataRateOptimize[3], AgcAutoOn[2]
    sx127x_write_reg(REG_MODEM_CONFIG3, (0<<3) | (1<<2));
}

static void radio_write_fifo_and_tx(const uint8_t *data, uint8_t len){
    sx127x_write_reg(REG_IRQ_FLAGS, 0xFF); // clear
    sx127x_write_reg(REG_FIFO_TX_BASE, 0x00);
    sx127x_write_reg(REG_FIFO_ADDR_PTR, 0x00);
    sx127x_burst_write(REG_FIFO, data, len);
    sx127x_write_reg(REG_PAYLOAD_LEN, len);
    radio_tx();
    // Wait for TxDone
    for(int i=0;i<2000;i++){ // ~2s worst case
        uint8_t f = sx127x_read_reg(REG_IRQ_FLAGS);
        if(f & 0x08){ // TxDone
            sx127x_write_reg(REG_IRQ_FLAGS, 0x08);
            break;
        }
        usleep(1000);
    }
    radio_standby();
}

// -------------------- Buttons (libgpiod) --------------------
static int line_get(struct gpiod_chip *chip, int pin, bool output, int initval, struct gpiod_line **out){
    struct gpiod_line *l = gpiod_chip_get_line(chip, pin);
    if(!l) return -1;
    int rv = output ? gpiod_line_request_output(l, "app", initval)
                    : gpiod_line_request_input (l, "app");
    if(rv) return -1;
    *out = l; return 0;
}

// -------------------- Main --------------------
int main(void){
    // Open GPIO
    struct gpiod_chip *chip = gpiod_chip_open(GPIO_CHIP);
    if(!chip){ perror("open gpiochip"); return 1; }
    struct gpiod_line *l_rst=NULL,*l_oledrst=NULL,*l_btna=NULL,*l_btnb=NULL,*l_btnc=NULL;
    if(line_get(chip, PIN_RFM95_RST, true, 1, &l_rst))     { fprintf(stderr,"gpio rst\n"); return 1; }
    if(line_get(chip, PIN_OLED_RST,  true, 1, &l_oledrst)) { fprintf(stderr,"gpio oled rst\n"); return 1; }
    if(line_get(chip, PIN_BTN_A,     false,0,&l_btna))     { fprintf(stderr,"gpio btn a\n"); return 1; }
    if(line_get(chip, PIN_BTN_B,     false,0,&l_btnb))     { fprintf(stderr,"gpio btn b\n"); return 1; }
    if(line_get(chip, PIN_BTN_C,     false,0,&l_btnc))     { fprintf(stderr,"gpio btn c\n"); return 1; }

    // I2C open + set addr
    i2c_fd = open(I2C_DEV, O_RDWR);
    if(i2c_fd<0){ perror("open i2c"); return 1; }
    if(ioctl(i2c_fd, I2C_SLAVE, I2C_ADDR)<0){ perror("i2c addr"); return 1; }
    oled_init(PIN_OLED_RST, l_oledrst);
    oled_clear();
    oled_draw_text(0,0,"Init RFM95...");
    oled_show();

    // SPI open
    spi_fd = spi_init(SPI_DEV);
    if(spi_fd<0) return 1;

    // RFM95 hardware reset pulse
    gpiod_line_set_value(l_rst, 0); usleep(10000);
    gpiod_line_set_value(l_rst, 1); usleep(10000);

    // LoRa mode + standby
    sx127x_write_reg(REG_OPMODE, OPMODE_LONG_RANGE_MODE | OPMODE_SLEEP);
    usleep(10000);
    radio_standby();

    // Check version (should be 0x12 for SX1276/7/8/9)
    uint8_t ver = sx127x_read_reg(REG_VERSION);

    // Configure radio
    radio_set_freq(RADIO_FREQ_MHZ);
    radio_set_tx_power(TX_POWER_DBM);
    radio_set_modem_bw_sf_cr(125, 7, 5); // BW=125k, SF7, CR=4/5
    sx127x_write_reg(REG_LNA, 0x23);     // LNA gain boost
    sx127x_write_reg(REG_PREAMBLE_MSB, 0x00);
    sx127x_write_reg(REG_PREAMBLE_LSB, 0x08);
    sx127x_write_reg(REG_FIFO_TX_BASE, 0x00);
    sx127x_write_reg(REG_FIFO_RX_BASE, 0x80);

    // OLED splash
    char line[32];
    oled_clear();
    snprintf(line, sizeof(line), "RFM95 v0x%02X", ver);
    oled_draw_text(0,0,line);
    oled_draw_text(0,8,"A:TX  B:rate");
    oled_draw_text(0,16,"Freq 915.0MHz");
    oled_show();

    // Main loop
    int counter=0;
    bool fast=false;
    struct timespec ts_last; clock_gettime(CLOCK_MONOTONIC, &ts_last);

    for(;;){
        // buttons (active low pull-ups)
        int a = gpiod_line_get_value(l_btna)==0;
        int b = gpiod_line_get_value(l_btnb)==0;

        if(b){
            fast = !fast;
            oled_clear();
            oled_draw_text(0,0, fast ? "Rate: FAST" : "Rate: SLOW");
            oled_draw_text(0,8,"A:TX  B:rate");
            oled_show();
            usleep(250000); // debounce
        }

        // cadence
        double period = fast ? PERIOD_FAST : PERIOD_SLOW;
        struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
        double dt = (now.tv_sec - ts_last.tv_sec) + (now.tv_nsec - ts_last.tv_nsec)/1e9;

        bool do_tx = a || (dt >= period);
        if(a) usleep(150000); // debounce

        if(do_tx){
            char payload[48];
            int n = snprintf(payload, sizeof(payload), "Pi4 hello #%d", counter);
            if(n<0) n=0; if(n>47) n=47;

            radio_write_fifo_and_tx((const uint8_t*)payload, (uint8_t)n);
            ts_last = now;
            counter++;

            oled_clear();
            oled_draw_text(0,0,"TX OK");
            oled_draw_text(0,8,payload);
            char pwr[24]; snprintf(pwr,sizeof(pwr),"Pwr %ddBm", TX_POWER_DBM);
            oled_draw_text(0,16,pwr);
            oled_show();

            usleep(200000);
        }

        usleep(10000);
    }

    // (not reached)
    close(spi_fd);
    close(i2c_fd);
    gpiod_chip_close(chip);
    return 0;
}
