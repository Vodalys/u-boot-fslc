/*
 * Copyright (C) 2014 Vodalys
 *
 * Author: Jean-Michel Hautbois <jean-michel.hautbois@vodalys.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <asm/imx-common/mxc_i2c.h>
#include <i2c.h>

#ifdef CONFIG_IMX_SPI_CDCM6208
#include "common/cdcm6208.h"
#endif

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)
		      
#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

	return 0;
}

iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT0__GPIO1_IO16 | MUX_PAD_CTRL(USDHC_PAD_CTRL), /* CD */
};

iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

iomux_v3_cfg_t const ecspi3_pads[] = {
	MX6_PAD_DISP0_DAT0__ECSPI3_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT1__ECSPI3_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT2__ECSPI3_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT3__ECSPI3_SS0 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT4__ECSPI3_SS1 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT7__GPIO4_IO28 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT8__GPIO4_IO29 | MUX_PAD_CTRL(SPI_PAD_CTRL),	
};

iomux_v3_cfg_t const extra_pads[] = {
	MX6_PAD_GPIO_3__GPIO1_IO03 | MUX_PAD_CTRL(SPI_PAD_CTRL),			/* SDI reset_n */
	MX6_PAD_GPIO_7__GPIO1_IO07 | MUX_PAD_CTRL(SPI_PAD_CTRL),			/* USB reset_n */
	MX6_PAD_GPIO_8__GPIO1_IO08 | MUX_PAD_CTRL(SPI_PAD_CTRL),			/* CDCM6208 reset_n */
	MX6_PAD_SD1_CMD__GPIO1_IO18 | MUX_PAD_CTRL(SPI_PAD_CTRL),		/* ADV7604 reset_n */
	MX6_PAD_SD1_CLK__GPIO1_IO20 | MUX_PAD_CTRL(SPI_PAD_CTRL),		/* ADV7611 reset_n */
	MX6_PAD_SD1_DAT2__GPIO1_IO19 | MUX_PAD_CTRL(SPI_PAD_CTRL),		/* ADV7604 powerdown_n */
	MX6_PAD_GPIO_9__GPIO1_IO09 | MUX_PAD_CTRL(SPI_PAD_CTRL),			/* FPGA reset_n */	
};

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

/* I2C2 SGTL5000, MMFP0100, FPGA, DS1342, LMH0303 */
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | PC,
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | PC,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | PC,
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | PC,
		.gp = IMX_GPIO_NR(4, 13)
	}
};


#ifdef CONFIG_IMX_ECSPI
s32 spi_get_cfg(struct imx_spi_dev_t *dev)
{
	switch (dev->slave.cs) 
	{
		case 0:
			/* CDCM6208 */
			dev->base = ECSPI3_BASE_ADDR;
			dev->freq = 2500000;
			dev->ss_pol = IMX_SPI_ACTIVE_LOW;
			dev->ss = 0;
			dev->fifo_sz = 64 * 4;
			dev->us_delay = 0;
			break;
		default:
			printf("Invalid Bus ID!\n");
	}

	return 0;
}

void spi_io_init(struct imx_spi_dev_t *dev)
{
	u32 reg;

	switch (dev->base)
	{
		case ECSPI3_BASE_ADDR:
			/* Enable clock */
			reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR1);
			reg |= (0x3 << 4);
			writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR1);

			/* SCLK */
			imx_iomux_v3_setup_pad(MX6_PAD_DISP0_DAT0__ECSPI3_SCLK);

			/* MISO */
			imx_iomux_v3_setup_pad(MX6_PAD_DISP0_DAT2__ECSPI3_MISO);

			/* MOSI */
			imx_iomux_v3_setup_pad(MX6_PAD_DISP0_DAT1__ECSPI3_MOSI);

			/* SS0 */
			imx_iomux_v3_setup_pad(MX6_PAD_DISP0_DAT3__ECSPI3_SS0);
			break;
		default:
			break;
	}
}
#endif

struct spi_slave *cdcm8208_spi_slave = (struct spi_slave *)NULL;

static void setup_spi(void)
{
	imx_iomux_v3_setup_multiple_pads(ecspi3_pads, ARRAY_SIZE(ecspi3_pads));
}



static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

static void extra_init(void)
{
	unsigned int reg;
	puts("Reset all peripherals...\n");
	
	imx_iomux_v3_setup_multiple_pads(extra_pads, ARRAY_SIZE(extra_pads));	
	//imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));

	/* All resets simultaneously including Ethernet PHY */
	reg = readl(GPIO1_BASE_ADDR + 0x0);
	reg &= ~((1 << 3) | (1 << 4) | (1 << 7) | (1 << 8) | (1 << 9) | (1 << 18) | (1 << 19) | (1 << 20));
	writel(reg, GPIO1_BASE_ADDR + 0x0);

	reg = readl(GPIO1_BASE_ADDR + 0x4);
	reg |= ((1 << 3) | (1 << 4) | (1 << 7) | (1 << 8) | (1 << 9) | (1 << 18) | (1 << 19) | (1 << 20));
	writel(reg, GPIO1_BASE_ADDR + 0x4);

	udelay(1000);

	reg = readl(GPIO1_BASE_ADDR + 0x0);
	reg |= ((1 << 3) | (1 << 4) | (1 << 7) | (1 << 8) | (1 << 9) | (1 << 18) | (1 << 19) | (1 << 20));
	writel(reg, GPIO1_BASE_ADDR + 0x0);	
}

#ifdef CONFIG_FSL_ESDHC

struct fsl_esdhc_cfg usdhc_cfg[] = {
	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

#define USDHC3_CD_GPIO	IMX_GPIO_NR(1, 16)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC3_BASE_ADDR:
		ret = gpio_get_value(USDHC3_CD_GPIO);
		break;
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	int i;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    SD3
	 * mmc1                    SD4
	 */
	
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			usdhc_cfg[0].max_bus_width = 4;
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			usdhc_cfg[1].max_bus_width = 8;
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) than supported by the board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return status;
		}
		
		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[i]);

	}

	return status;
}

#endif

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_early_init_f(void)
{

	setup_iomux_uart();

	return 0;
}

int board_init(void)
{

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	
	/* need set Power Supply Glitch to 0x41736166
	 * and need clear Power supply Glitch Detect bit
	 * when POR or reboot or power on Otherwise system
	 * could not be power off anymore*/
	u32 reg;
	writel(0x41736166, SNVS_BASE_ADDR + 0x64);/*set LPPGDR*/
	udelay(10);
	reg = readl(SNVS_BASE_ADDR + 0x4c);
	reg |= (1 << 3);
	writel(reg, SNVS_BASE_ADDR + 0x4c);/*clear LPSR*/
	
#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif

	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	
#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif

	/* Reset all chips */
	extra_init();
	
	/* Power off LEDs */
	gpio_direction_output(IMX_GPIO_NR(6, 11), 0);
	gpio_direction_output(IMX_GPIO_NR(6, 14), 0);
	gpio_direction_output(IMX_GPIO_NR(6, 15), 0);
	gpio_direction_output(IMX_GPIO_NR(6, 16), 0);

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd3",	 MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
	puts("Entering late board init...\n");

#ifdef CONFIG_IMX_SPI_CDCM6208
	puts("Probing CDCM6208...\n");
	if (cdcm8208_spi_slave = spi_cdcm6208_probe()) {
		show_cdcm6208_info(cdcm8208_spi_slave);
	}
#endif
	
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	return 0;
}

int checkboard(void)
{
	puts("Board: Vodabox3\n");
	return 0;
}
