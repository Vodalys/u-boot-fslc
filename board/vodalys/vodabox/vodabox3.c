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
#include <asm/arch/crm_regs.h>
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
#include <spi.h>

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
	MX6_PAD_DISP0_DAT3__GPIO4_IO24	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT4__GPIO4_IO25	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT7__GPIO4_IO28	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT8__GPIO4_IO29	| MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const extra_pads[] = {
	MX6_PAD_GPIO_3__GPIO1_IO03 | MUX_PAD_CTRL(NO_PAD_CTRL),			/* SDI reset_n */
	MX6_PAD_GPIO_7__GPIO1_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL),			/* USB reset_n */
	MX6_PAD_GPIO_8__GPIO1_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),			/* CDCM6208 reset_n */
	MX6_PAD_SD1_CMD__GPIO1_IO18 | MUX_PAD_CTRL(NO_PAD_CTRL),		/* ADV7604 reset_n */
	MX6_PAD_SD1_CLK__GPIO1_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL),		/* ADV7611 reset_n */
	MX6_PAD_SD1_DAT2__GPIO1_IO19 | MUX_PAD_CTRL(NO_PAD_CTRL),		/* ADV7604 powerdown_n */
	MX6_PAD_GPIO_9__GPIO1_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL),			/* FPGA reset_n */
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

struct spi_slave *cdcm8208_spi_slave = (struct spi_slave *)NULL;

static void setup_spi(void)
{
	int reg = 0;
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	puts("Setup SPI...\n");
	imx_iomux_v3_setup_multiple_pads(ecspi3_pads, ARRAY_SIZE(ecspi3_pads));
	/* Enable clock of ecspi3 */
	reg = readl(&mxc_ccm->CCGR1);
	reg |= MXC_CCM_CCGR1_ECSPI3S_MASK;
	writel(reg, &mxc_ccm->CCGR1);
}

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

#ifdef CONFIG_FEC_MXC

iomux_v3_cfg_t enet_pads[] =
{
	/* LAN8720A */
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TX_EN__ENET_TX_EN		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TXD0__ENET_TX_DATA0		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TXD1__ENET_TX_DATA1		| MUX_PAD_CTRL(ENET_PAD_CTRL),

	MX6_PAD_ENET_CRS_DV__ENET_RX_EN		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RX_ER__ENET_RX_ER		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RXD0__ENET_RX_DATA0		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RXD1__ENET_RX_DATA1		| MUX_PAD_CTRL(ENET_PAD_CTRL),

	MX6_PAD_GPIO_16__ENET_REF_CLK		| MUX_PAD_CTRL(ENET_PAD_CTRL),

	MX6_PAD_GPIO_4__GPIO1_IO04		| MUX_PAD_CTRL(NO_PAD_CTRL), /* Reset */
};

#define ETH_PHY_RESET	IMX_GPIO_NR(1, 4)

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
	/* Reset LAN8720A PHY */
	gpio_direction_output(ETH_PHY_RESET, 0);
	udelay(1000);
	gpio_set_value(ETH_PHY_RESET, 1);
}

int board_eth_init(bd_t *bis)
{
	setup_iomux_enet();

	return cpu_eth_init(bis);
}

#define ANATOP_PLL_LOCK                 0x80000000
#define ANATOP_PLL_PWDN_MASK            0x00001000
#define ANATOP_PLL_BYPASS_MASK          0x00010000
#define ANATOP_FEC_PLL_ENABLE_MASK      0x00002000

static int setup_fec(void)
{
	u32 reg = 0;
	s32 timeout = 100000;

	/*
     * get enet tx reference clk from internal clock from anatop
     * GPR1[21] = 1
     */
	reg =  readl(IOMUXC_BASE_ADDR + 0x4);
	reg |= (0x1 << 21);
	writel(reg, IOMUXC_BASE_ADDR + 0x4);

	/* Set fast slew rate and medium speed for clock pin */
	reg =  readl(IOMUXC_BASE_ADDR + 0x618);
	reg |= PAD_CTL_SPEED_MED | PAD_CTL_SRE_FAST;
	writel(reg, IOMUXC_BASE_ADDR + 0x618);

	/* Set daisy bit in IOMUXC_ENET_REF_CLK_SELECT_INPUT register */
	writel(1, IOMUXC_BASE_ADDR + 0x83c);

	/* Enable PLLs */
	reg = readl(ANATOP_BASE_ADDR + 0xe0); /* ENET PLL */
	if ((reg & ANATOP_PLL_PWDN_MASK) || (!(reg & ANATOP_PLL_LOCK)))
	{
		reg &= ~ANATOP_PLL_PWDN_MASK;
		writel(reg, ANATOP_BASE_ADDR + 0xe0);
		while (timeout--)
		{
			if (readl(ANATOP_BASE_ADDR + 0xe0) & ANATOP_PLL_LOCK)
			{
				break;
			}
		}

		if (timeout <= 0)
		{
			return -1;
		}
	}

	/* Enable FEC clock */
	reg |= ANATOP_FEC_PLL_ENABLE_MASK;
	reg &= ~ANATOP_PLL_BYPASS_MASK;
	writel(reg, ANATOP_BASE_ADDR + 0xe0);

	return 0;
}

#endif


static void extra_init(void)
{
	unsigned int reg;
	puts("Reset all peripherals...\n");

	imx_iomux_v3_setup_multiple_pads(extra_pads, ARRAY_SIZE(extra_pads));

	/* All resets simultaneously */
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

#if defined(CONFIG_VIDEO_IPUV3)

iomux_v3_cfg_t const di0_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,	/* DISP0_CLK */
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02,		/* DISP0_HSYNC */
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03,		/* DISP0_VSYNC */
};

struct display_info_t {
	int	bus;
	int	addr;
	int	pixfmt;
	int	(*detect)(struct display_info_t const *dev);
	void	(*enable)(struct display_info_t const *dev);
	struct	fb_videomode mode;
};

static int detect_hdmi(struct display_info_t const *dev)
{
	struct hdmi_regs *hdmi	= (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
	return readb(&hdmi->phy_stat0) & HDMI_DVI_STAT;
}


static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg = readl(&iomux->gpr[2]);

	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
		 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);

	writel(reg, &iomux->gpr[2]);
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
	disable_lvds(dev);
	imx_enable_hdmi_phy();
}

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)
				IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT |
	       IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT;
	writel(reg, &iomux->gpr[2]);
}

static struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "Hannstar-XGA",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_hdmi,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} } };

int board_video_skip(void)
{
	int i;
	int ret;
	char const *panel = getenv("panel");
	if (!panel) {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			struct display_info_t const *dev = displays+i;
			if (dev->detect && dev->detect(dev)) {
				panel = dev->mode.name;
				printf("auto-detected panel %s\n", panel);
				break;
			}
		}
		if (!panel) {
			panel = displays[0].mode.name;
			printf("No panel detected: default to %s\n", panel);
			i = 0;
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(displays); i++) {
			if (!strcmp(panel, displays[i].mode.name))
				break;
		}
	}
	if (i < ARRAY_SIZE(displays)) {
		ret = ipuv3_fb_init(&displays[i].mode, 0,
				    displays[i].pixfmt);
		if (!ret) {
			displays[i].enable(displays+i);
			printf("Display: %s (%ux%u)\n",
			       displays[i].mode.name,
			       displays[i].mode.xres,
			       displays[i].mode.yres);
		} else
			printf("LCD %s cannot be configured: %d\n",
			       displays[i].mode.name, ret);
	} else {
		printf("unsupported panel %s\n", panel);
		return -EINVAL;
	}

	return 0;
}

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	/* Setup HSYNC, VSYNC, DISP_CLK for debugging purposes */
	imx_iomux_v3_setup_multiple_pads(di0_pads, ARRAY_SIZE(di0_pads));

	enable_ipu_clock();
	imx_setup_hdmi();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     | IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED
	     | IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS1_MUX_CTL_MASK
			| IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif /* CONFIG_VIDEO_IPUV3 */


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
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif
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

#ifdef CONFIG_FEC_MXC
	setup_fec();
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

#ifdef CONFIG_IMX_SPI_CDCM6208
typedef struct
{
	unsigned short reg;
	unsigned short value;
} CDCM6208_T_CFG;

CDCM6208_T_CFG cdmc6208Config [] =
{
	{0, 0x0079},
	{1, 0x0004},
	{2, 0x0036},
	{3, 0x00F0},
	{4, 0x30EB},/* 0x00EF */
	{5, 0x0133},
	{6, 0x0004},
	{7, 0x0199},
	{8, 0x0004},
	{9, 0x4253},
	{10, 0x00DD},
	{11, 0x9800},
	{12, 0x0253},
	{13, 0x00DD},
	{14, 0x9A00},
	{15, 0x0259},
	{16, 0x00CC},
	{17, 0x0000},
	{18, 0x0243},
	{19, 0x0066},
	{20, 0xCD00},
};

static u32 cdcm6208_tx, cdcm6208_rx;

static int cdcm6208_read(struct spi_slave *slave, u8 reg, u16 val)
{
	int ret = 0;
	// Check register value for read access
	if (reg > 21 && reg != 40)
	{
		printf("<reg num> = %d is invalide. Should be less then 22 or equal to 40\n", reg);
		ret = -1;
		goto finish;
	}

	// CSPI SS1, SS2 & SS3 must be high to avoid bus conflicts
	gpio_direction_output(IMX_GPIO_NR(4, 25), 1);
	gpio_direction_output(IMX_GPIO_NR(4, 28), 1);
	gpio_direction_output(IMX_GPIO_NR(4, 29), 1);

	// SS0 needs to be up and down between each read / write
	gpio_direction_output(IMX_GPIO_NR(4, 24), 0);

	cdcm6208_tx = ntohl((1 << 31) | (reg << 16) | (val & 0xFFFF));

	// Initiate SPI transfer
	if (spi_xfer(slave, 4 << 3, (u8 *)&cdcm6208_tx, (u8 *)&cdcm6208_rx, SPI_XFER_BEGIN | SPI_XFER_END))
	{
		ret = -1;
		debug("cdcm6208_reg : Read failed 0x%x @ 0x%x\n", ntohl(cdcm6208_rx), reg);
		goto finish;
	}
	ret = (htonl(cdcm6208_rx) & 0xFFFF);
	debug("cdcm6208_reg : Read 0x%x @ 0x%x\n", ntohl(cdcm6208_rx), reg);

finish:
	gpio_direction_output(IMX_GPIO_NR(4, 24), 1);
	return ret;
}

static int cdcm6208_write(struct spi_slave *slave, u8 reg, u16 val)
{
	int ret = 0;
	// Check register value for write access
	if (reg > 20)
	{
		printf("<reg num> = %d is invalide. Should be less then 21\n", reg);
		ret = -1;
		goto finish;
	}

	// CSPI SS1, SS2 & SS3 must be high to avoid bus conflicts
	gpio_direction_output(IMX_GPIO_NR(4, 25), 1);
	gpio_direction_output(IMX_GPIO_NR(4, 28), 1);
	gpio_direction_output(IMX_GPIO_NR(4, 29), 1);

	// SS0 needs to be up and down between each read / write
	gpio_direction_output(IMX_GPIO_NR(4, 24), 0);


	cdcm6208_tx = ntohl((0 << 31) | (reg << 16) | (val & 0xFFFF));

	// Initiate SPI transfer
	debug("Initiate SPI transfer cmd=%X\n", cdcm6208_tx);
	if (spi_xfer(slave, 4 << 3, (u8 *)&cdcm6208_tx, (u8 *)&cdcm6208_rx, SPI_XFER_BEGIN | SPI_XFER_END))
	{
		debug("cdcm6208_reg : Write failed 0x%x @ 0x%x\n", ntohl(cdcm6208_tx) & 0xFFFF, reg);
		ret = -1;
		goto finish;
	}

	debug("cdcm6208_reg : Write 0x%x @ 0x%x\n", ntohl(cdcm6208_tx) & 0xFFFF, reg);

finish:
	gpio_direction_output(IMX_GPIO_NR(4, 24), 1);
	return ret;
}


int cdcm6208_config (struct spi_slave *slave)
{
	unsigned int index;
	u32 reg = 0;

	for (index = 0; index < sizeof (cdmc6208Config) / sizeof (*cdmc6208Config);index ++)
	{
		if (cdcm6208_write(slave, cdmc6208Config[index].reg, cdmc6208Config[index].value))
		{
			return -1;
		} else {
			/* Write seems ok, re-read it */
			reg = cdcm6208_read(slave, cdmc6208Config[index].reg, 0);
			if (reg != cdmc6208Config[index].value) {
				printf("Read 0x%x instead of 0x%x\n", reg, cdmc6208Config[index].value);
				return -1;
			}
		}
	}

	return 0;
}

static int cdcm6208_read_config (struct spi_slave *slave)
{
	unsigned int index;
	u32 reg = 0;

	for (index = 0; index < sizeof (cdmc6208Config) / sizeof (*cdmc6208Config);index ++)
	{
		reg = cdcm6208_read(slave, cdmc6208Config[index].reg, 0);
		if (reg != cdmc6208Config[index].value) {
			printf("Read 0x%x instead of 0x%x\n", reg, cdmc6208Config[index].value);
			return -1;
		}
	}

	return 0;
}

static int cdcm6208_reset(void)
{
	puts("Reset CDCM6208...\n");
	gpio_direction_output(IMX_GPIO_NR(1, 8), 0);
	udelay(1000);
	gpio_direction_output(IMX_GPIO_NR(1, 8), 1);
	udelay(1000);
	return 0;
}

int cdcm6208_calibrate (struct spi_slave *slave)
{
	unsigned int cpt = 100;
	// Toggle reset_n pin
	cdcm6208_reset();

	puts("Checking register configuration... ");
	if (cdcm6208_read_config(slave) != 0) {
		puts("KO !\n");
		return -1;
	}
	puts("OK !\n");

	// Check if PLL is locked
	while (cpt --)
	{
		if (!((cdcm6208_read(slave, 21, 0) >> 1) & 0x3))
		{
			return 0;
		}

		udelay (10000);
	}

	return -1;
}

struct spi_slave *spi_cdcm6208_probe(void)
{
	return spi_setup_slave(CONFIG_IMX_SPI_CDCM6208_BUS, CONFIG_IMX_SPI_CDCM6208_CS, 2500000, 0);
}

void spi_cdcm6208_free(struct spi_slave *slave)
{
	if (slave)
		spi_free_slave(slave);
}

#endif

int board_late_init(void)
{
	puts("Entering late board init...\n");

#ifdef CONFIG_IMX_SPI_CDCM6208
	int i;
	puts("Probing CDCM6208...\n");
	gpio_direction_output(IMX_GPIO_NR(4, 24), 0);

	if (cdcm8208_spi_slave = spi_cdcm6208_probe()) {
		puts("Claiming SPI bus... ");
		if (!spi_claim_bus(cdcm8208_spi_slave)) {
			puts("OK !\n");
			cdcm6208_reset();
			volatile u32 rev_id;
			rev_id = cdcm6208_read(cdcm8208_spi_slave, 40, 0);

			printf("CDCM6208: Version = %s, Revision = %s\n", ((rev_id >> 3) & 0x7) ? "CDCM6208V2" : "CDCM6208V1", ((rev_id & 0x7) == 2) ? "Production" : "Engineering");

			if (cdcm6208_config(cdcm8208_spi_slave)) {
				printf ("Error during configuring CDCM6208\n");
			}
			else {
				if (cdcm6208_calibrate(cdcm8208_spi_slave))	{
					printf ("Timeout during CDCM6208 PLL locking, PLL is not locked\n");
				}
				else {
					printf ("CDCM6208 configured\n");
				}
			}
		}
		else {
			puts("Failed !\n");
		}
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
