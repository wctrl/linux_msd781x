/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019 Daniel Palmer
 */

#define MAKEMASK(_what) \
	GENMASK((SHIFT_##_what + WIDTH_##_what) - 1, SHIFT_##_what)

#define REG_UARTS		0xc
#define REG_PWMS		0x1c
#define REG_SDIO_NAND		0x20
#define REG_I2CS		0x24
#define REG_SPIS		0x30
#define REG_ETH_JTAG		0x3c
#define REG_SENSOR_CONFIG	0x54
#define REG_TX_MIPI_UART2	0x58

#define REG_I2C1_PULL_EN	0x94
#define REG_I2C1_PULL_DIR	0x98
#define REG_I2C1_DRIVE		0x9c
#define REG_SPI_DRIVE		0xa8
#define REG_SDIO_PULLDRIVE	0xc8
#define REG_SR_INPUTENABLE0	0xe0
#define REG_SR_INPUTENABLE1	0xe4
#define REG_SR_PULL_EN0		0xe8
#define REG_SR_PULL_EN1		0xec
#define REG_SR_PULL_DIR0	0xf0
#define REG_SR_PULL_DIR1	0xf4
#define REG_SR_DRIVE0		0xf8
#define REG_SR_DRIVE1		0xfc

/* common group select registers and masks */
#define REG_FUART	REG_UARTS
#define MASK_FUART	(BIT(1) | BIT(0))
#define REG_UART0	REG_UARTS
#define MASK_UART0	(BIT(5) | BIT(4))
#define REG_UART1	REG_UARTS
#define MASK_UART1	(BIT(9) | BIT(8))

#define REG_PWM0	REG_PWMS
#define MASK_PWM0	(BIT(1) | BIT(0))
#define REG_PWM1	REG_PWMS
#define MASK_PWM1	(BIT(3) | BIT(2))
#define REG_PWM2	REG_PWMS
#define MASK_PWM2	(BIT(5) | BIT(4))
#define REG_PWM3	REG_PWMS
#define MASK_PWM3	(BIT(7) | BIT(6))
#define REG_PWM4	REG_PWMS
#define MASK_PWM4	(BIT(9) | BIT(8))
#define REG_PWM5	REG_PWMS
#define MASK_PWM5	(BIT(11) | BIT(10))
#define REG_PWM6	REG_PWMS
#define MASK_PWM6	(BIT(13) | BIT(11))
#define REG_PWM7	REG_PWMS
#define MASK_PWM7	(BIT(15) | BIT(14))

#define REG_SDIO	REG_SDIO_NAND
#define MASK_SDIO	BIT(8)

#define REG_I2C0	REG_I2CS
#define MASK_I2C0	(BIT(1) | BIT(0))
#define REG_I2C1	REG_I2CS
#define MASK_I2C1	(BIT(5) | BIT(4))

#define REG_SPI0	REG_SPIS
#define MASK_SPI0	(BIT(1) | BIT(0))
#define REG_SPI1	REG_SPIS
#define MASK_SPI1	(BIT(5) | BIT(4))

#define REG_JTAG	REG_ETH_JTAG
#define MASK_JTAG	(BIT(1) | BIT(0))

#define REG_ETH		REG_ETH_JTAG
#define MASK_ETH	BIT(2)

#define REG_SR0_MIPI	REG_SENSOR_CONFIG
#define MASK_SR0_MIPI	(BIT(9) | BIT(8))

#define REG_SR1_BT656	REG_SENSOR_CONFIG
#define MASK_SR1_BT656	BIT(12)

#define REG_SR1_MIPI	REG_SENSOR_CONFIG
#define MASK_SR1_MIPI	(BIT(15) | BIT(14) | BIT(13))

#define REG_TX_MIPI	REG_TX_MIPI_UART2
#define MASK_TX_MIPI	(BIT(1) | BIT(0))

/* ssd201/202d */
/*
 * for ssd20xd the uart registers are at the same place but
 * there are more muxing options
 */
#define REG_SSD20XD_FUART	REG_FUART
#define SHIFT_SSD20XD_FUART	0
#define WIDTH_SSD20XD_FUART	3
#define MASK_SSD20XD_FUART	(BIT(2) | BIT(1) | BIT(0))
#define REG_SSD20XD_UART0	REG_UART0
#define SHIFT_SSD20XD_UART0	4
#define WIDTH_SSD20XD_UART0	3
#define MASK_SSD20XD_UART0	(BIT(6) | BIT(5) | BIT(4))
#define REG_SSD20XD_UART1	REG_UART1
#define SHIFT_SSD20XD_UART1	8
#define WIDTH_SSD20XD_UART1	3
#define MASK_SSD20XD_UART1	(BIT(10) | BIT(9) | BIT(8))
#define REG_SSD20XD_UART2	REG_UARTS
#define SHIFT_SSD20XD_UART2	12
#define WIDTH_SSD20XD_UART2	3
#define MASK_SSD20XD_UART2	(BIT(14) | BIT(13) | BIT(12))
/*
 * for ssd20xd the i2c registers are at the same place but
 * there are more muxing options
 */
#define REG_SSD20XD_I2C1	REG_I2C1
#define SHIFT_SSD20XD_I2C1	4
#define WIDTH_SSD20XD_I2C1	3
#define MASK_SSD20XD_I2C1	MAKEMASK(SSD20XD_I2C1)

#define REG_SSD20XD_TTL		0x34
#define SHIFT_SSD20XD_TTL	8
#define WIDTH_SSD20XD_TTL	4
#define MASK_SSD20XD_TTL	MAKEMASK(SSD20XD_TTL)
#define REG_SSD20XD_ETH		0x38
#define MASK_SSD20XD_ETH0	BIT(0)
#define MASK_SSD20XD_ETH1	(BIT(11) | BIT(10) | BIT(9) | BIT(8))


/* common pin group names */
#define GROUPNAME_PM_UART		"pm_uart"
#define GROUPNAME_PM_SPI		"pm_spi"
#define GROUPNAME_PM_LED_MODE1		"pm_led_mode1"
#define GROUPNAME_PM_IRIN		"pm_irin"
#define GROUPNAME_SD			"sd"
#define GROUPNAME_SD_D0_D1_D2_D3	"sd_d0_d1_d2_d3"
#define GROUPNAME_USB			"usb"
#define GROUPNAME_USB1			"usb1"
#define GROUPNAME_I2C0			"i2c0"
#define GROUPNAME_I2C1			"i2c1"
#define GROUPNAME_I2C1_MODE1		"i2c1_mode1"
#define GROUPNAME_I2C1_MODE3		"i2c1_mode3"
#define GROUPNAME_I2C1_MODE4		"i2c1_mode4"
#define GROUPNAME_I2C1_MODE5		"i2c1_mode5"
#define GROUPNAME_FUART			"fuart"
#define GROUPNAME_FUART_RX		"fuart_rx"
#define GROUPNAME_FUART_TX		"fuart_tx"
#define GROUPNAME_FUART_CTS		"fuart_cts"
#define GROUPNAME_FUART_RTS		"fuart_rts"
#define GROUPNAME_FUART_RX_TX		"fuart_rx_tx"
#define GROUPNAME_FUART_RX_TX_RTS	"fuart_rx_tx_rts"
#define GROUPNAME_FUART_CTS_RTS		"fuart_cts_rts"
#define GROUPNAME_FUART_CTS		"fuart_cts"
#define GROUPNAME_FUART_MODE1		"fuart_mode1"
#define GROUPNAME_FUART_MODE2		"fuart_mode2"
#define GROUPNAME_FUART_MODE3		"fuart_mode3"
#define GROUPNAME_FUART_MODE4		"fuart_mode4"
#define GROUPNAME_FUART_MODE5		"fuart_mode5"
#define GROUPNAME_FUART_MODE6		"fuart_mode6"
#define GROUPNAME_FUART_MODE7		"fuart_mode7"
#define GROUPNAME_UART0			"uart0"
#define GROUPNAME_UART1			"uart1"
#define GROUPNAME_UART1_MODE1		"uart_mode1"
#define GROUPNAME_ETH			"eth"
#define GROUPNAME_PWM0			"pwm0"
#define GROUPNAME_PWM1			"pwm1"
#define GROUPNAME_PWM2			"pwm2"
#define GROUPNAME_PWM3			"pwm3"
#define GROUPNAME_PWM4			"pwm4"
#define GROUPNAME_PWM5			"pwm5"
#define GROUPNAME_PWM6			"pwm6"
#define GROUPNAME_PWM7			"pwm7"
#define GROUPNAME_SPI0			"spi0"
#define GROUPNAME_SPI0_CZ		"spi0_cz"
#define GROUPNAME_SPI0_CK		"spi0_ck"
#define GROUPNAME_SPI0_DI		"spi0_di"
#define GROUPNAME_SPI0_DO		"spi0_do"
#define GROUPNAME_SPI1			"spi1"

#ifdef CONFIG_MACH_MERCURY
#define GROUPNAME_SR0_MIPI_MODE1	"sr0_mipi_mode1"
#define GROUPNAME_SR0_MIPI_MODE2	"sr0_mipi_mode2"
#define GROUPNAME_SR1_BT656		"sr1_bt656"
#define GROUPNAME_SR1_MIPI_MODE4	"sr1_mipi_mode4"
#define GROUPNAME_TX_MIPI_MODE1		"tx_mipi_mode1"
#define GROUPNAME_TX_MIPI_MODE2		"tx_mipi_mode2"
#endif

#define GROUPNAME_TTL_MODE1		"ttl_mode1"

/* common group function names */
#define FUNCTIONNAME_PM_UART	GROUPNAME_PM_UART
#define FUNCTIONNAME_PM_SPI	GROUPNAME_PM_SPI
#define FUNCTIONNAME_PM_LED	"pm_led"
#define FUNCTIONNAME_PM_IRIN	GROUPNAME_PM_IRIN
#define FUNCTIONNAME_USB	GROUPNAME_USB
#define FUNCTIONNAME_USB1	GROUPNAME_USB1
#define FUNCTIONNAME_FUART	GROUPNAME_FUART
#define FUNCTIONNAME_UART0	GROUPNAME_UART0
#define FUNCTIONNAME_UART1	GROUPNAME_UART1
#define FUNCTIONNAME_UART2	"uart2"
#define FUNCTIONNAME_ETH	GROUPNAME_ETH
#define FUNCTIONNAME_JTAG	"jtag"
#define FUNCTIONNAME_PWM0	GROUPNAME_PWM0
#define FUNCTIONNAME_PWM1	GROUPNAME_PWM1
#define FUNCTIONNAME_PWM2	GROUPNAME_PWM2
#define FUNCTIONNAME_PWM3	GROUPNAME_PWM3
#define FUNCTIONNAME_PWM4	GROUPNAME_PWM4
#define FUNCTIONNAME_PWM5	GROUPNAME_PWM5
#define FUNCTIONNAME_PWM6	GROUPNAME_PWM6
#define FUNCTIONNAME_PWM7	GROUPNAME_PWM7
#define FUNCTIONNAME_SDIO	"sdio"
#define FUNCTIONNAME_I2C0	GROUPNAME_I2C0
#define FUNCTIONNAME_I2C1	GROUPNAME_I2C1
#define FUNCTIONNAME_SPI0	GROUPNAME_SPI0
#define FUNCTIONNAME_SPI1	GROUPNAME_SPI1

#define FUNCTIONNAME_SR0_MIPI	"sr0_mipi"
#define FUNCTIONNAME_SR1_BT656	GROUPNAME_SR1_BT656
#define FUNCTIONNAME_SR1_MIPI	"sr1_mipi"

#define FUNCTIONNAME_TX_MIPI	"tx_mipi"
#define FUNCTIONNAME_TTL	"ttl"

/* shared structures */
struct msc313_pinctrl {
	struct device *dev;
	struct pinctrl_desc desc;
	struct pinctrl_dev *pctl;
	struct regmap *regmap;
	const struct msc313_pinctrl_info *info;
};

struct msc313_pinctrl_function {
	const char *name;
	int reg;
	u16 mask;
	const char * const *groups;
	const u16 *values;
	int numgroups;
};

struct msc313_pinctrl_group {
	const char *name;
	const int *pins;
	const int numpins;
};

/*
 * Not all pins have "pinconf" so we only
 * carry this extra info for those that do.
 *
 * For some pins all of these bits in the same
 * register, for others they are split across
 * registers. Not all pins have all of the
 * registers.
 */
struct msc313_pinctrl_pinconf {
	const int pin;
	const int pull_en_reg;
	const int pull_en_bit;
	const int pull_dir_reg;
	const int pull_dir_bit;
	const int input_reg;
	const int input_bit;
	const int drive_reg;
	const int drive_lsb;
	const int drive_width;
	const unsigned int *drivecurrents;
	const int ndrivecurrents;
};

/*
 * Per-chip info that describes all of the pins,
 * the pin groups, the mappable functions and
 * pins that support pinconf.
 */
struct msc313_pinctrl_info {
	const struct pinctrl_pin_desc *pins;
	const int npins;
	const struct msc313_pinctrl_group *groups;
	const int ngroups;
	const struct msc313_pinctrl_function *functions;
	const int nfunctions;
	const struct msc313_pinctrl_pinconf *pinconfs;
	const int npinconfs;
};

/* There isn't a register for the function for this pin */
#define NOREG		-1
/*
 * If used for pull_en_reg this means there is an always
 * on pull up, if used for pull_dir_reg there is an optional
 * pull up.
 */
#define ALWAYS_PULLUP	-2
/* See above but for pull down. */
#define ALWAYS_PULLDOWN	-3

#define MSTAR_PINCTRL_PIN(_pin, _pull_en_reg, _pull_en_bit, \
		_pull_dir_reg, _pull_dir_bit, \
		_drive_reg, _drive_lsb, _drive_width, _drivecurrents) \
	{ \
		.pin = _pin, \
		.pull_en_reg = _pull_en_reg, \
		.pull_en_bit = _pull_en_bit, \
		.pull_dir_reg = _pull_dir_reg, \
		.pull_dir_bit = _pull_dir_bit, \
		.drive_reg = _drive_reg, \
		.drive_lsb = _drive_lsb, \
		.drive_width = _drive_width, \
		.drivecurrents = _drivecurrents, \
		.ndrivecurrents = ARRAY_SIZE(_drivecurrents) \
	}

/* shared struct helpers */
#define MSTAR_PINCTRL_FUNCTION(n, r, m, g, v) \
	{ \
		.name = n,\
		.reg = r,\
		.mask = m,\
		.groups = g,\
		.values = v,\
		.numgroups = ARRAY_SIZE(g)\
	}

#define MSTAR_PINCTRL_GROUP(n, p) \
	{\
		.name = n,\
		.pins = p,\
		.numpins = ARRAY_SIZE(p)\
	}

#define MSTAR_PINCTRL_INFO(_chip) static const struct msc313_pinctrl_info _chip##_info = { \
	.pins = _chip##_pins, \
	.npins = ARRAY_SIZE(_chip##_pins), \
	.groups = _chip##_pinctrl_groups, \
	.ngroups = ARRAY_SIZE(_chip##_pinctrl_groups), \
	.functions = _chip##_pinctrl_functions, \
	.nfunctions = ARRAY_SIZE(_chip##_pinctrl_functions), \
	.pinconfs = _chip##_configurable_pins, \
	.npinconfs = ARRAY_SIZE(_chip##_configurable_pins),\
}

/* shared functions */
int mstar_set_mux(struct pinctrl_dev *pctldev, unsigned int func, unsigned int group);
int mstar_dt_node_to_map(struct pinctrl_dev *pctldev, struct device_node *np,
		struct pinctrl_map **map, unsigned int *num_maps);
void mstar_dt_free_map(struct pinctrl_dev *pctldev, struct pinctrl_map *map,
		unsigned int num_maps);
int msc313_pinctrl_parse_groups(struct msc313_pinctrl *pinctrl);
int msc313_pinctrl_parse_functions(struct msc313_pinctrl *pinctrl);

/* Helpers for pins that have the same on the different chips */
#define COMMON_PIN(_model, _pinname) \
	PINCTRL_PIN(PIN_##_model##_##_pinname, PINNAME_##_pinname)

#define COMMON_FIXED_FUNCTION(_NAME, _name) \
	MSTAR_PINCTRL_FUNCTION(FUNCTIONNAME_##_NAME, -1, 0, _name##_groups, NULL)
#define COMMON_FUNCTION(_NAME, _name) \
	MSTAR_PINCTRL_FUNCTION(FUNCTIONNAME_##_NAME, REG_##_NAME, MASK_##_NAME, _name##_groups, _name##_values)
#define COMMON_FUNCTION_NULLVALUES(_NAME, _name) \
	MSTAR_PINCTRL_FUNCTION(FUNCTIONNAME_##_NAME, REG_##_NAME, MASK_##_NAME, _name##_groups, NULL)

/* Helpers for msc313/msc313e pins and groups */
#define MSC313_COMMON_PIN(_pinname) COMMON_PIN(MSC313, _pinname)
#define MSC313_PINCTRL_GROUP(_NAME, _name) \
	MSTAR_PINCTRL_GROUP(GROUPNAME_##_NAME, msc313_##_name##_pins)

/* for ssd20xd pins */
#define SSD20XD_COMMON_PIN(_pinname) COMMON_PIN(SSD20XD, _pinname)

#define SSD20XD_PINCTRL_GROUP(_NAME, _name) \
	MSTAR_PINCTRL_GROUP(GROUPNAME_##_NAME, ssd20xd_##_name##_pins)

#define SSD20XD_MODE(_func, _modenum) (_modenum << SHIFT_SSD20XD_##_func)

#define SSD20XD_FUNCTION(_NAME, _name) \
	MSTAR_PINCTRL_FUNCTION(FUNCTIONNAME_##_NAME, REG_SSD20XD_##_NAME, \
	MASK_SSD20XD_##_NAME, ssd20xd_##_name##_groups, ssd20xd_##_name##_values)
