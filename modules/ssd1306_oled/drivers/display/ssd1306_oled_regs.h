#ifndef SSD1306_OLED_REGS_H_
#define SSD1306_OLED_REGS_H_

/* I2C control byte prefixes */
#define SSD1306_CTRL_CMD   0x00  /* all following bytes are commands */
#define SSD1306_CTRL_DATA  0x40  /* all following bytes are data */

/* Fundamental commands */
#define SSD1306_SET_CONTRAST        0x81  /* + 1 byte: 0x00-0xFF */
#define SSD1306_DISPLAY_RAM         0xA4  /* output follows RAM content */
#define SSD1306_DISPLAY_ALL_ON      0xA5  /* all pixels ON */
#define SSD1306_NORMAL_DISPLAY      0xA6
#define SSD1306_INVERT_DISPLAY      0xA7
#define SSD1306_DISPLAY_OFF         0xAE
#define SSD1306_DISPLAY_ON          0xAF

/* Scrolling commands */
#define SSD1306_DEACTIVATE_SCROLL   0x2E

/* Addressing mode commands */
#define SSD1306_SET_MEM_ADDR_MODE   0x20  /* + 1 byte: 0x00/0x01/0x02 */
#define SSD1306_ADDR_MODE_HORIZ     0x00
#define SSD1306_ADDR_MODE_VERT      0x01
#define SSD1306_ADDR_MODE_PAGE      0x02
#define SSD1306_SET_COL_ADDR        0x21  /* + 2 bytes: start, end */
#define SSD1306_SET_PAGE_ADDR       0x22  /* + 2 bytes: start, end */

/* Hardware configuration commands */
#define SSD1306_SET_START_LINE      0x40  /* | line (0-63) */
#define SSD1306_SEG_REMAP_NORMAL    0xA0
#define SSD1306_SEG_REMAP_FLIP      0xA1
#define SSD1306_SET_MUX_RATIO       0xA8  /* + 1 byte: 15-63 */
#define SSD1306_COM_SCAN_NORMAL     0xC0
#define SSD1306_COM_SCAN_FLIP       0xC8
#define SSD1306_SET_DISPLAY_OFFSET  0xD3  /* + 1 byte: 0-63 */
#define SSD1306_SET_COM_PINS        0xDA  /* + 1 byte */
#define SSD1306_COM_PINS_SEQ        0x02
#define SSD1306_COM_PINS_ALT        0x12

/* Timing & driving commands */
#define SSD1306_SET_CLOCK_DIV       0xD5  /* + 1 byte: [freq:4][div:4] */
#define SSD1306_SET_PRECHARGE       0xD9  /* + 1 byte: [phase2:4][phase1:4] */
#define SSD1306_SET_VCOM_DESEL      0xDB  /* + 1 byte */

/* Charge pump commands */
#define SSD1306_SET_CHARGE_PUMP     0x8D  /* + 1 byte */
#define SSD1306_CHARGE_PUMP_OFF     0x10
#define SSD1306_CHARGE_PUMP_ON      0x14

#endif /* SSD1306_OLED_REGS_H_ */
