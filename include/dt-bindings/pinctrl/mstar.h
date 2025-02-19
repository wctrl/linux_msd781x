#ifndef INCLUDE_DT_BINDINGS_PINCTRL_MSTAR_H_
#define INCLUDE_DT_BINDINGS_PINCTRL_MSTAR_H_

/* standard pin names that are used across mstar/sigmastars parts */
#define PINNAME_PM_UART_RX	"pm_uart_rx"
#define PINNAME_PM_UART_TX	"pm_uart_tx"
#define PINNAME_PM_SD_CDZ	"pm_sd_cdz"
#define PINNAME_PM_IRIN		"pm_irin"
#define PINNAME_PM_GPIO0	"pm_gpio0"
#define PINNAME_PM_GPIO2	"pm_gpio2"
#define PINNAME_PM_GPIO4	"pm_gpio4"
#define PINNAME_PM_GPIO5	"pm_gpio5"
#define PINNAME_PM_GPIO6	"pm_gpio6"
#define PINNAME_PM_GPIO8	"pm_gpio8"
#define PINNAME_PM_SPI_CZ	"pm_spi_cz"
#define PINNAME_PM_SPI_DI	"pm_spi_di"
#define PINNAME_PM_SPI_WPZ	"pm_spi_wpz"
#define PINNAME_PM_SPI_DO	"pm_spi_do"
#define PINNAME_PM_SPI_CK	"pm_spi_ck"
#define PINNAME_PM_SPI_HOLD	"pm_spi_hold"
#define PINNAME_PM_SPI_HLD	PINNAME_PM_SPI_HOLD
#define PINNAME_PM_LED0		"pm_led0"
#define PINNAME_PM_LED1		"pm_led1"
#define PINNAME_FUART_TX	"fuart_tx"
#define PINNAME_FUART_RX	"fuart_rx"
#define	PINNAME_FUART_CTS	"fuart_cts"
#define PINNAME_FUART_RTS	"fuart_rts"
#define PINNAME_SPI0_CZ		"spi0_cz"
#define PINNAME_SPI0_CZ1	"spi0_cz1"
#define PINNAME_SPI0_CK		"spi0_ck"
#define PINNAME_SPI0_DI		"spi0_di"
#define PINNAME_SPI0_DO		"spi0_do"
#define PINNAME_SD_CLK		"sd_clk"
#define PINNAME_SD_CMD		"sd_cmd"
#define PINNAME_SD_D0		"sd_d0"
#define PINNAME_SD_D1		"sd_d1"
#define PINNAME_SD_D2		"sd_d2"
#define PINNAME_SD_D3		"sd_d3"
#define PINNAME_USB_DM		"usb_dm"
#define PINNAME_USB_DP		"usb_dp"
#define PINNAME_USB_DM1		"usb_dm1"
#define PINNAME_USB_DP1		"usb_dp1"
#define PINNAME_USB1_DM		"usb1_dm"
#define PINNAME_USB1_DP		"usb1_dp"
#define PINNAME_USB_CID		"usb_cid"
#define PINNAME_I2C0_SCL	"i2c0_scl"
#define PINNAME_I2C0_SDA	"i2c0_sda"
#define PINNAME_I2C1_SCL	"i2c1_scl"
#define PINNAME_I2C1_SDA	"i2c1_sda"
#define PINNAME_ETH_RN		"eth_rn"
#define PINNAME_ETH_RP		"eth_rp"
#define PINNAME_ETH_TN		"eth_tn"
#define PINNAME_ETH_TP		"eth_tp"
#define PINNAME_LCD_DE		"lcd_de"
#define PINNAME_LCD_PCLK	"lcd_pclk"
#define PINNAME_LCD_VSYNC	"lcd_vsync"
#define PINNAME_LCD_HSYNC	"lcd_hsync"
#define PINNAME_LCD_0		"lcd_0"
#define PINNAME_LCD_1		"lcd_1"
#define PINNAME_LCD_2		"lcd_2"
#define PINNAME_LCD_3		"lcd_3"
#define PINNAME_LCD_4		"lcd_4"
#define PINNAME_LCD_5		"lcd_5"
#define PINNAME_LCD_6		"lcd_6"
#define PINNAME_LCD_7		"lcd_7"
#define PINNAME_LCD_8		"lcd_8"
#define PINNAME_LCD_9		"lcd_9"
#define PINNAME_LCD_10		"lcd_10"
#define PINNAME_LCD_11		"lcd_11"
#define PINNAME_LCD_12		"lcd_12"
#define PINNAME_LCD_13		"lcd_13"
#define PINNAME_LCD_14		"lcd_14"
#define PINNAME_LCD_15		"lcd_15"
#define PINNAME_UART1_TX	"uart1_tx"
#define PINNAME_UART1_RX	"uart1_rx"
#define PINNAME_UART2_TX	"uart2_tx"
#define PINNAME_UART2_RX	"uart2_rx"
#define PINNAME_GPIO0		"gpio0"
#define PINNAME_GPIO1		"gpio1"
#define PINNAME_GPIO2		"gpio2"
#define PINNAME_GPIO3		"gpio3"
#define PINNAME_GPIO4		"gpio4"
#define PINNAME_GPIO5		"gpio5"
#define PINNAME_GPIO6		"gpio6"
#define PINNAME_GPIO7		"gpio7"
#define PINNAME_GPIO10		"gpio10"
#define PINNAME_GPIO11		"gpio11"
#define PINNAME_GPIO12		"gpio12"
#define PINNAME_GPIO13		"gpio13"
#define PINNAME_GPIO14		"gpio14"
#define PINNAME_GPIO47		"gpio47"
#define PINNAME_GPIO48		"gpio48"
#define PINNAME_GPIO85		"gpio85"
#define PINNAME_GPIO86		"gpio86"
#define PINNAME_GPIO90		"gpio90"
#define PINNAME_TTL0		"ttl0"
#define PINNAME_TTL1		"ttl1"
#define PINNAME_TTL2		"ttl2"
#define PINNAME_TTL3		"ttl3"
#define PINNAME_TTL4		"ttl4"
#define PINNAME_TTL5		"ttl5"
#define PINNAME_TTL6		"ttl6"
#define PINNAME_TTL7		"ttl7"
#define PINNAME_TTL8		"ttl8"
#define PINNAME_TTL9		"ttl9"
#define PINNAME_TTL10		"ttl10"
#define PINNAME_TTL11		"ttl11"
#define PINNAME_TTL12		"ttl12"
#define PINNAME_TTL13		"ttl13"
#define PINNAME_TTL14		"ttl14"
#define PINNAME_TTL15		"ttl15"
#define PINNAME_TTL16		"ttl16"
#define PINNAME_TTL17		"ttl17"
#define PINNAME_TTL18		"ttl18"
#define PINNAME_TTL19		"ttl19"
#define PINNAME_TTL20		"ttl20"
#define PINNAME_TTL21		"ttl21"
#define PINNAME_TTL22		"ttl22"
#define PINNAME_TTL23		"ttl23"
#define PINNAME_TTL24		"ttl24"
#define PINNAME_TTL25		"ttl25"
#define PINNAME_TTL26		"ttl26"
#define PINNAME_TTL27		"ttl27"
#define PINNAME_HDMITX_SCL	"hdmitx_scl"
#define PINNAME_HDMITX_SDA	"hdmitx_sda"
#define PINNAME_HDMITXHPD	"hdmitxhpd"
#define PINNAME_HDMI2TXCN	"hdmi2txcn"
#define PINNAME_HDMI2TXCP	"hdmi2txcp"
#define PINNAME_HDMI2TX0N	"hdmi2tx0n"
#define PINNAME_HDMI2TX0P	"hdmi2tx0p"
#define PINNAME_HDMI2TX1N	"hdmi2tx1n"
#define PINNAME_HDMI2TX1P	"hdmi2tx1p"
#define PINNAME_HDMI2TX2N	"hdmi2tx2n"
#define PINNAME_HDMI2TX2P	"hdmi2tx2p"
/*
 * for later parts with more sensor interfaces
 * the pin naming seems to have changed
 */
#define PINNAME_SR0_D2		"sr0_d2"
#define PINNAME_SR0_D3		"sr0_d3"
#define PINNAME_SR0_D4		"sr0_d4"
#define PINNAME_SR0_D5		"sr0_d5"
#define PINNAME_SR0_D6		"sr0_d6"
#define PINNAME_SR0_D7		"sr0_d7"
#define PINNAME_SR0_D8		"sr0_d8"
#define PINNAME_SR0_D9		"sr0_d9"
#define PINNAME_SR0_D10		"sr0_d10"
#define PINNAME_SR0_D11		"sr0_d11"
#define PINNAME_SR0_GPIO0	"sr0_gpio0"
#define PINNAME_SR0_GPIO1	"sr0_gpio1"
#define PINNAME_SR0_GPIO2	"sr0_gpio2"
#define PINNAME_SR0_GPIO3	"sr0_gpio3"
#define PINNAME_SR0_GPIO4	"sr0_gpio4"
#define PINNAME_SR0_GPIO5	"sr0_gpio5"
#define PINNAME_SR0_GPIO6	"sr0_gpio6"
#define PINNAME_SR1_GPIO0	"sr1_gpio0"
#define PINNAME_SR1_GPIO1	"sr1_gpio1"
#define PINNAME_SR1_GPIO2	"sr1_gpio2"
#define PINNAME_SR1_GPIO3	"sr1_gpio3"
#define PINNAME_SR1_GPIO4	"sr1_gpio4"
#define PINNAME_SR1_D0P		"sr1_d0p"
#define PINNAME_SR1_D0N		"sr1_d0n"
#define PINNAME_SR1_CKP		"sr1_ckp"
#define PINNAME_SR1_CKN		"sr1_ckn"
#define PINNAME_SR1_D1P		"sr1_d1p"
#define PINNAME_SR1_D1N		"sr1_d1n"

/* pioneer3 parts have some new naming.. */
#define PINNAME_SR_GPIO0	"sr_gpio0"
#define PINNAME_SR_GPIO1	"sr_gpio1"
#define PINNAME_SR_GPIO2	"sr_gpio2"
#define PINNAME_SR_GPIO3	"sr_gpio3"
#define PINNAME_SR_GPIO4	"sr_gpio4"
#define PINNAME_SR_GPIO5	"sr_gpio5"
#define PINNAME_SR_GPIO6	"sr_gpio6"
#define PINNAME_SR_GPIO7	"sr_gpio7"
#define PINNAME_SR_GPIO8	"sr_gpio8"
#define PINNAME_SR_GPIO9	"sr_gpio9"
#define PINNAME_SR_GPIO10	"sr_gpio10"
#define PINNAME_SR_GPIO11	"sr_gpio11"

/* MSC313/MSC313E */
/* Chip pin numbers */
#define PIN_MSC313_PM_SD_CDZ	15
#define PIN_MSC313_PM_IRIN	16
#define PIN_MSC313_PM_UART_RX	18
#define PIN_MSC313_PM_UART_TX	19
#define PIN_MSC313_PM_GPIO4	21
#define PIN_MSC313_PM_SPI_CZ	22
#define PIN_MSC313_PM_SPI_DI	23
#define PIN_MSC313_PM_SPI_WPZ	24
#define PIN_MSC313_PM_SPI_DO	25
#define PIN_MSC313_PM_SPI_CK	26
#define PIN_MSC313_ETH_RN	31
#define PIN_MSC313_ETH_RP	32
#define PIN_MSC313_ETH_TN	33
#define PIN_MSC313_ETH_TP	34
#define PIN_MSC313_FUART_RX	36
#define PIN_MSC313_FUART_TX	37
#define PIN_MSC313_FUART_CTS	38
#define PIN_MSC313_FUART_RTS	39
#define PIN_MSC313_I2C1_SCL	41
#define PIN_MSC313_I2C1_SDA	42
#define PIN_MSC313_SR_IO2	44
#define PIN_MSC313_SR_IO3	45
#define PIN_MSC313_SR_IO4	46
#define PIN_MSC313_SR_IO5	47
#define PIN_MSC313_SR_IO6	48
#define PIN_MSC313_SR_IO7	49
#define PIN_MSC313_SR_IO8	50
#define PIN_MSC313_SR_IO9	51
#define PIN_MSC313_SR_IO10	52
#define PIN_MSC313_SR_IO11	53
#define PIN_MSC313_SR_IO12	54
#define PIN_MSC313_SR_IO13	55
#define PIN_MSC313_SR_IO14	56
#define PIN_MSC313_SR_IO15	57
#define PIN_MSC313_SR_IO16	58
#define PIN_MSC313_SR_IO17	59
#define PIN_MSC313_SPI0_CZ	63
#define PIN_MSC313_SPI0_CK	64
#define PIN_MSC313_SPI0_DI	65
#define PIN_MSC313_SPI0_DO	66
#define PIN_MSC313_SD_CLK	68
#define PIN_MSC313_SD_CMD	69
#define PIN_MSC313_SD_D0	70
#define PIN_MSC313_SD_D1	71
#define PIN_MSC313_SD_D2	72
#define PIN_MSC313_SD_D3	73
#define PIN_MSC313_USB_DM	75
#define PIN_MSC313_USB_DP	76

/* SSC8336 */
/* Chip pin numbers */
#define PIN_SSC8336N_USB_DM1     7
#define PIN_SSC8336N_USB_DP1     8
#define PIN_SSC8336N_USB_DM      9
#define PIN_SSC8336N_USB_DP      10
#define PIN_SSC8336N_USB_CID     12
#define PIN_SSC8336N_PM_SPI_CZ   27
#define PIN_SSC8336N_PM_SPI_DI   28
#define PIN_SSC8336N_PM_SPI_WPZ  29
#define PIN_SSC8336N_PM_SPI_DO   30
#define PIN_SSC8336N_PM_SPI_CK   31
#define PIN_SSC8336N_PM_SPI_HOLD 32
#define PIN_SSC8336N_PM_GPIO8    34
#define PIN_SSC8336N_PM_GPIO6	 35
#define PIN_SSC8336N_PM_GPIO5	 36
#define PIN_SSC8336N_PM_GPIO4	 37
#define PIN_SSC8336N_PM_GPIO2	 38
#define PIN_SSC8336N_PM_GPIO0	 39
#define PIN_SSC8336N_PM_UART_TX	 40
#define PIN_SSC8336N_PM_UART_RX	 41
#define PIN_SSC8336N_PM_IRIN	 42
#define PIN_SSC8336N_PM_SD_CDZ	 43
#define PIN_SSC8336N_FUART_RX	 52
#define PIN_SSC8336N_FUART_TX	 53
#define PIN_SSC8336N_FUART_CTS	 54
#define PIN_SSC8336N_FUART_RTS	 55
#define PIN_SSC8336N_SPI0_DO	 56
#define PIN_SSC8336N_SPI0_DI	 57
#define PIN_SSC8336N_SPI0_CK	 58
#define PIN_SSC8336N_SPI0_CZ	 59
#define PIN_SSC8336N_SPI0_CZ1	 60
#define PIN_SSC8336N_I2C0_SCL	 61
#define PIN_SSC8336N_I2C0_SDA	 62
#define PIN_SSC8336N_SD_D1	 67
#define PIN_SSC8336N_SD_D0	 68
#define PIN_SSC8336N_SD_CLK	 69
#define PIN_SSC8336N_SD_CMD	 70
#define PIN_SSC8336N_SD_D3	 71
#define PIN_SSC8336N_SD_D2	 72
#define PIN_SSC8336N_SR0_D2	 73
#define PIN_SSC8336N_SR0_D3	 74
#define PIN_SSC8336N_SR0_D4	 75
#define PIN_SSC8336N_SR0_D5	 76
#define PIN_SSC8336N_SR0_D6	 77
#define PIN_SSC8336N_SR0_D7	 78
#define PIN_SSC8336N_SR0_D8	 79
#define PIN_SSC8336N_SR0_D9	 80
#define PIN_SSC8336N_SR0_D10	 81
#define PIN_SSC8336N_SR0_D11	 82
#define PIN_SSC8336N_SR0_GPIO0	 84
#define PIN_SSC8336N_SR0_GPIO1	 85
#define PIN_SSC8336N_SR0_GPIO2	 86
#define PIN_SSC8336N_SR0_GPIO3	 87
#define PIN_SSC8336N_SR0_GPIO4	 88
#define PIN_SSC8336N_SR0_GPIO5	 89
#define PIN_SSC8336N_SR0_GPIO6	 90
#define PIN_SSC8336N_SR1_GPIO0   92
#define PIN_SSC8336N_SR1_GPIO1   93
#define PIN_SSC8336N_SR1_GPIO2   94
#define PIN_SSC8336N_SR1_GPIO3   95
#define PIN_SSC8336N_SR1_GPIO4   96
#define PIN_SSC8336N_SR1_D0P     97
#define PIN_SSC8336N_SR1_D0N     98
#define PIN_SSC8336N_SR1_CKP     99
#define PIN_SSC8336N_SR1_CKN     100
#define PIN_SSC8336N_SR1_D1P     101
#define PIN_SSC8336N_SR1_D1N     102
#define PIN_SSC8336N_LCD_HSYNC   104
#define PIN_SSC8336N_LCD_VSYNC   105
#define PIN_SSC8336N_LCD_PCLK    106
#define PIN_SSC8336N_LCD_DE      107
#define PIN_SSC8336N_LCD_0       108
#define PIN_SSC8336N_LCD_1       109
#define PIN_SSC8336N_LCD_2       110
#define PIN_SSC8336N_LCD_3       111
#define PIN_SSC8336N_LCD_4       112
#define PIN_SSC8336N_LCD_5       113
#define PIN_SSC8336N_LCD_6       114
#define PIN_SSC8336N_LCD_7       115
#define PIN_SSC8336N_LCD_8       116
#define PIN_SSC8336N_LCD_9       117
#define PIN_SSC8336N_LCD_10      119
#define PIN_SSC8336N_LCD_11      120
#define PIN_SSC8336N_LCD_12      121
#define PIN_SSC8336N_LCD_13      122
#define PIN_SSC8336N_LCD_14      123
#define PIN_SSC8336N_LCD_15      124

/* SSD20[1|2D] */
/* Chip pin numbers */
#define PIN_SSD20XD_GPIO12		1
#define PIN_SSD20XD_GPIO13		2
#define PIN_SSD20XD_GPIO14		3
#define PIN_SSD20XD_GPIO85		5
#define PIN_SSD20XD_GPIO86		6
#define PIN_SSD20XD_DMIC_R		7
#define PIN_SSD20XD_DMIC_L		8
#define PIN_SSD20XD_DMIC_CLK		9
#define PIN_SSD20XD_GPIO90		10
#define PIN_SSD20XD_PM_SPI_CZ		26
#define PIN_SSD20XD_PM_SPI_CK		27
#define PIN_SSD20XD_PM_SPI_DI		28
#define PIN_SSD20XD_PM_SPI_DO		29
#define PIN_SSD20XD_PM_SPI_HLD		30
#define PIN_SSD20XD_PM_SPI_WPZ		31
#define PIN_SSD20XD_PM_IRIN		32
#define PIN_SSD20XD_PM_UART_RX		34
#define PIN_SSD20XD_PM_UART_TX		35
#define PIN_SSD20XD_GPIO47		36
#define PIN_SSD20XD_GPIO48		37
#define PIN_SSD20XD_UART1_RX		38
#define PIN_SSD20XD_UART1_TX		39
#define PIN_SSD20XD_FUART_RX		52
#define PIN_SSD20XD_FUART_TX		53
#define PIN_SSD20XD_FUART_CTS		54
#define PIN_SSD20XD_FUART_RTS		55
#define PIN_SSD20XD_TTL0		56
#define PIN_SSD20XD_TTL1		57
#define PIN_SSD20XD_TTL2		58
#define PIN_SSD20XD_TTL3		59
#define PIN_SSD20XD_TTL4		60
#define PIN_SSD20XD_TTL5		61
#define PIN_SSD20XD_USB_DP		62
#define PIN_SSD20XD_USB_DM		63
#define PIN_SSD20XD_TTL6		65
#define PIN_SSD20XD_TTL7		66
#define PIN_SSD20XD_TTL8		67
#define PIN_SSD20XD_TTL9		68
#define PIN_SSD20XD_TTL10		69
#define PIN_SSD20XD_TTL11		70
#define PIN_SSD20XD_TTL12		71
#define PIN_SSD20XD_TTL13		72
#define PIN_SSD20XD_TTL14		73
#define PIN_SSD20XD_TTL15		74
#define PIN_SSD20XD_TTL16		79
#define PIN_SSD20XD_TTL17		80
#define PIN_SSD20XD_TTL18		81
#define PIN_SSD20XD_TTL19		82
#define PIN_SSD20XD_TTL20		83
#define PIN_SSD20XD_TTL21		84
#define PIN_SSD20XD_TTL22		85
#define PIN_SSD20XD_TTL23		86
#define PIN_SSD20XD_TTL24		87
#define PIN_SSD20XD_TTL25		88
#define PIN_SSD20XD_TTL26		89
#define PIN_SSD20XD_TTL27		90
#define PIN_SSD20XD_PM_SD_CDZ		91
#define PIN_SSD20XD_SD_D1		92
#define PIN_SSD20XD_SD_D0		93
#define PIN_SSD20XD_SD_CLK		94
#define PIN_SSD20XD_SD_CMD		95
#define PIN_SSD20XD_SD_D3		96
#define PIN_SSD20XD_SD_D2		97
#define PIN_SSD20XD_VDDP_1		98
#define PIN_SSD20XD_GPIO0		99
#define PIN_SSD20XD_GPIO1		100
#define PIN_SSD20XD_GPIO2		101
#define PIN_SSD20XD_GPIO3		102
#define PIN_SSD20XD_PM_LED0		103
#define PIN_SSD20XD_PM_LED1		104
#define PIN_SSD20XD_ETH_RN		107
#define PIN_SSD20XD_ETH_RP		108
#define PIN_SSD20XD_ETH_TN		109
#define PIN_SSD20XD_ETH_TP		110
#define PIN_SSD20XD_USB1_DP		111
#define PIN_SSD20XD_USB1_DM		112
#define PIN_SSD20XD_AUD_LINEOUT_R0	115
#define PIN_SSD20XD_AUD_LINEOUT_L0	116
#define PIN_SSD20XD_AUD_MICCM0		117
#define PIN_SSD20XD_AUD_MICIN0		118
#define PIN_SSD20XD_GPIO4		121
#define PIN_SSD20XD_GPIO5		122
#define PIN_SSD20XD_GPIO6		123
#define PIN_SSD20XD_GPIO7		124
#define PIN_SSD20XD_UART2_RX		125
#define PIN_SSD20XD_UART2_TX		126
#define PIN_SSD20XD_GPIO10		127
#define PIN_SSD20XD_GPIO11		128

/* SSD203D */
/* Chip pin numbers */
#define PIN_SSD203D_GPIO85		4
#define PIN_SSD203D_GPIO86		5
#define PIN_SSD203D_HDMITX_SCL		6
#define PIN_SSD203D_HDMITX_SDA		7
#define PIN_SSD203D_HDMITXHPD		8
#define PIN_SSD203D_PM_SPI_CZ		29
#define PIN_SSD203D_PM_SPI_CK		30
#define PIN_SSD203D_PM_SPI_DI		31
#define PIN_SSD203D_PM_SPI_DO		32
#define PIN_SSD203D_PM_SPI_HLD		33
#define PIN_SSD203D_PM_SPI_WPZ		34
#define PIN_SSD203D_PM_IRIN		35
#define PIN_SSD203D_PM_UART_RX		37
#define PIN_SSD203D_PM_UART_TX		38
#define PIN_SSD203D_HDMI2TXCN		111
#define PIN_SSD203D_HDMI2TXCP		112
#define PIN_SSD203D_HDMI2TX0N		113
#define PIN_SSD203D_HDMI2TX0P		114
#define PIN_SSD203D_HDMI2TX1N		116
#define PIN_SSD203D_HDMI2TX1P		117
#define PIN_SSD203D_HDMI2TX2N		118
#define PIN_SSD203D_HDMI2TX2P		119

/* SSD210 */
/* Chip pin numbers */
#define PIN_SSD210_PM_SPI_CZ		2
#define PIN_SSD210_PM_SPI_CK		3
#define PIN_SSD210_PM_SPI_DI		4
#define PIN_SSD210_PM_SPI_DO		5
#define PIN_SSD210_PM_SPI_HOLD		6
#define PIN_SSD210_PM_SPI_WPZ		7
#define PIN_SSD210_SR_GPIO0		9
#define PIN_SSD210_SR_GPIO1		10
#define PIN_SSD210_SR_GPIO2		11
#define PIN_SSD210_SR_GPIO3		12
#define PIN_SSD210_SR_GPIO4		13
#define PIN_SSD210_SR_GPIO5		14
#define PIN_SSD210_SR_GPIO6		15
#define PIN_SSD210_SR_GPIO7		16
#define PIN_SSD210_SR_GPIO8		17
#define PIN_SSD210_SR_GPIO9		18
#define PIN_SSD210_SR_GPIO10		19
#define PIN_SSD210_SR_GPIO11		20
#define PIN_SSD210_TTL0			29
#define PIN_SSD210_TTL1			30
#define PIN_SSD210_TTL2			31
#define PIN_SSD210_TTL3			32
#define PIN_SSD210_TTL4			33
#define PIN_SSD210_TTL5			34
#define PIN_SSD210_TTL6			35
#define PIN_SSD210_TTL7			36
#define PIN_SSD210_TTL8			37
#define PIN_SSD210_TTL11		39
#define PIN_SSD210_TTL12		41
#define PIN_SSD210_TTL13		42
#define PIN_SSD210_TTL14		43
#define PIN_SSD210_TTL15		44
#define PIN_SSD210_TTL16		45
#define PIN_SSD210_TTL17		46
#define PIN_SSD210_TTL18		47
#define PIN_SSD210_TTL19		48
#define PIN_SSD210_TTL20		49
#define PIN_SSD210_TTL21		50
#define PIN_SSD210_PM_UART_TX		58
#define PIN_SSD210_PM_UART_RX		59

/* SSD212 */
/* Chip pin numbers */
#define PIN_SSD212_PM_SPI_CZ		17
#define PIN_SSD212_PM_SPI_CK		18
#define PIN_SSD212_PM_SPI_DI		19
#define PIN_SSD212_PM_SPI_DO		20
#define PIN_SSD212_PM_SPI_HOLD		21
#define PIN_SSD212_PM_SPI_WPZ		22
#define PIN_SSD212_PM_SD_CDZ		90
#define PIN_SSD212_PM_UART_TX		122
#define PIN_SSD212_PM_UART_RX		123

#endif
