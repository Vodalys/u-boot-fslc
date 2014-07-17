/*
 * Copyright (C) 2014 Vodalys
 *
 * Configuration settings for the Vodalys Vodabox3 board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __VODABOX3_CONFIG_H
#define __VODABOX3_CONFIG_H
#include "mx6_common.h"

#define CONFIG_MX6
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#define CONFIG_MACH_TYPE	3980
#define CONFIG_CONSOLE_DEV		"ttymxc0"
#define CONFIG_MMCROOT			"/dev/mmcblk0p2"
#define CONFIG_DEFAULT_FDT_FILE	"imx6q-vodabox3.dtb"

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>

#define	CONFIG_BOARD_EARLY_INIT_F

#define BOARD_LATE_INIT

#define CONFIG_CMDLINE_TAG	/* enable passing of ATAGs */
#define CONFIG_REVISION_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_MXC_GPIO

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE	UART1_BASE
/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX	       1
#define CONFIG_BAUDRATE			       115200
#define CONFIG_SYS_BAUDRATE_TABLE	{9600, 19200, 38400, 57600, 115200}

#define PHYS_SDRAM_SIZE		(1u * 1024 * 1024 * 1024)

//#define CONFIG_SUPPORT_EMMC_BOOT /* eMMC specific */

//#define CONFIG_CMD_BMODE

#include "mx6sabre_common.h"
/*
#define	CONFIG_EXTRA_ENV_SETTINGS					\
		"netdev=eth0\0"						\
		"ethprime=FEC0\0"					\
		"uboot=u-boot.bin\0"			\
		"kernel=uImage\0"				\
		"nfsroot=/opt/eldk/arm\0"				\
		"bootargs_base=setenv bootargs console=ttymxc1,115200\0"\
		"bootargs_nfs=setenv bootargs ${bootargs} root=/dev/nfs "\
			"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0"\
		"bootcmd_net=run bootargs_base bootargs_nfs; "		\
			"tftpboot ${loadaddr} ${kernel}; bootm\0"	\
		"bootargs_mmc=setenv bootargs ${bootargs} ip=dhcp "     \
			"root=/dev/mmcblk0p1 rootwait\0"                \
		"bootcmd_mmc=run bootargs_base bootargs_mmc; "   \
		"mmc dev 0; "	\
		"mmc read ${loadaddr} 0x800 0x2000; bootm\0"	\
		"bootcmd=run bootcmd_net\0"                             \
*/

#define CONFIG_SYS_FSL_USDHC_NUM	2
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_SYS_MMC_ENV_DEV		0
#endif
#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION


/* I2C Configs */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000

/*
 * SATA Configs
 */
#ifdef CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#endif

/* Framebuffer */
#if 0
#define CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IPUV3_CLK 260000000
#define CONFIG_IMX_HDMI
#endif

/* Command definition */
#include <config_cmd_default.h>

#undef CONFIG_CMD_IMLS

#undef CONFIG_BOOTDELAY
#define CONFIG_BOOTDELAY	       3

#undef CONFIG_LOADADDR
#define CONFIG_LOADADDR		0x10800000	/* loadaddr env var */
#define CONFIG_RD_LOADADDR	(0x1300000)

#define CONFIG_CMD_ENV

#define CONFIG_PREBOOT                 ""

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT	       "Vodabox3 > "
#define CONFIG_AUTO_COMPLETE

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS	16	/* max number of command args */
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE /* Boot Argument Buffer Size */

#define CONFIG_SYS_MEMTEST_START	0x10000000	/* memtest works on */
#define CONFIG_SYS_MEMTEST_END		0x10010000


#define CONFIG_SYS_LOAD_ADDR	       CONFIG_LOADADDR

#define CONFIG_CMDLINE_EDITING

/* Add support of LAN8720 */
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#undef CONFIG_FEC_XCV_TYPE
#define CONFIG_FEC_XCV_TYPE		RMII
#define CONFIG_ETHPRIME			"FEC"
#undef CONFIG_FEC_MXC_PHYADDR
#define CONFIG_FEC_MXC_PHYADDR		0

#define CONFIG_PHYLIB
#define CONFIG_PHY_SMSC
#define CONFIG_IPADDR			192.168.1.250
#define CONFIG_SERVERIP			192.168.1.192
#define CONFIG_NETMASK			255.255.255.0

/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS	1
#define PHYS_SDRAM_1		CSD0_DDR_BASE_ADDR
#define PHYS_SDRAM_1_SIZE	(1u * 1024 * 1024 * 1024)
#define iomem_valid_addr(addr, size) (addr >= PHYS_SDRAM_1 && addr <= (PHYS_SDRAM_1 + PHYS_SDRAM_1_SIZE))

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */
#define CONFIG_SYS_NO_FLASH

/* Monitor at beginning of flash */
#define CONFIG_FSL_ENV_IN_MMC

//#define CONFIG_ENV_SECT_SIZE    (8 * 1024)
//#define CONFIG_ENV_SIZE         CONFIG_ENV_SECT_SIZE

#if defined(CONFIG_FSL_ENV_IN_MMC)
	#define CONFIG_ENV_IS_IN_MMC
	#undef CONFIG_ENV_OFFSET
	#define CONFIG_ENV_OFFSET	(768 * 1024)
#else
	#define CONFIG_ENV_IS_NOWHERE	1
#endif


#define CONFIG_CMD_SPI
#define	CONFIG_MXC_ECSPI
/* CDCM6208 */
#if 1
#define	CONFIG_IMX_SPI_CDCM6208
#define	CONFIG_IMX_SPI_CDCM6208_BUS	2
#define CONFIG_IMX_SPI_CDCM6208_CS	0
#endif

#endif                         /* __VODABOX3_CONFIG_H */
