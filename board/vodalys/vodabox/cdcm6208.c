#include <config.h>
#include <common.h>
#include <spi.h>
#include <asm/errno.h>
#include <linux/types.h>

#include <asm/io.h>
#include <asm/arch/mx6-pins.h>
//#include <asm/arch/mx6dl-pins.h>

static u32 cdcm6208_tx, cdcm6208_rx;

/*!
 * To read/write to a cdcm6208 register.
 *
 * @param	reg			register number inside the cdcm6208
 * @param	val			data to be written to the register; don't care for read
 * @param	write		0 for read; 1 for write
 *
 * @return				the actual data in the cdcm6208 register
 */
u32 cdcm6208_reg(struct spi_slave *slave, u32 reg, u32 val, u32 write)
{
	if (!slave)
	{
		return -1;
	}

	// Check register value for write access
	if (reg > 20 && write == 1) 
	{
		printf("<reg num> = %d is invalide. Should be less then 21\n", reg);
		return -1;
	}
	
	// Check register value for read access
	if ((reg > 21 && reg != 40) && write == 0) 
	{
		printf("<reg num> = %d is invalide. Should be less then 22 or equal to 40\n", reg);
		return -1;
	}
	
	cdcm6208_tx = (!write << 31) | (reg << 16) | (val & 0xFFFF);

	// Initiate SPI transfer
	printf("Initiate SPI transfer cmd=%X\n", cdcm6208_tx);
	if (spi_xfer(slave, 4 << 3, (u8 *)&cdcm6208_tx, (u8 *)&cdcm6208_rx, SPI_XFER_BEGIN | SPI_XFER_END))
	{
		return -1;
	}

	if (write) 
	{
		printf("cdcm6208_reg : Write 0x%04x @ 0x%x\n", cdcm6208_tx & 0xFFFF, reg);
		return 0;
	}
	else
	{
		printf("cdcm6208_reg : Read 0x%04x @ 0x%x\n", cdcm6208_rx & 0xFFFF, reg);
		return (cdcm6208_rx & 0xFFFF);
	}
}

u32 show_cdcm6208_info(struct spi_slave *slave)
{
	volatile u32 rev_id, i;

	if (!slave)
	{
		return -1;
	}

	rev_id = cdcm6208_reg(slave, 40, 0, 0);
	printf("show_cdcm6208_info : Version = %s, Revision = %s\n", ((rev_id >> 3) & 0x7) ? "CDCM6208V2" : "CDCM6208V1", ((rev_id & 0x7) == 2) ? "Production" : "Engineering");
		
	return 0;
}

u32 cdcm6208_calibrate (struct spi_slave *slave)
{
	unsigned int reg, cpt = 1000;

	#if defined CONFIG_MX6Q

	// RESET_N
	imx_iomux_v3_setup_pad(MX6_PAD_GPIO_8__GPIO1_IO08);

	#elif defined CONFIG_MX6DL

	// RESET_N
	imx_iomux_v3_setup_pad(MX6_PAD_GPIO_8__GPIO1_IO08);

	#endif
	
	// Toggle reset_n pin	
	reg = readl(GPIO1_BASE_ADDR + 0x0);
	
	reg &= ~(1 << 8);
	writel(reg, GPIO1_BASE_ADDR + 0x0);
	
	reg = readl(GPIO1_BASE_ADDR + 0x4);
	reg |= (1 << 8);

	udelay(1000);

	reg = readl(GPIO1_BASE_ADDR + 0x0);
	reg |= (1 << 8);
	writel(reg, GPIO1_BASE_ADDR + 0x0);
	
	// Check if PLL is locked
	while (cpt --)
	{
		if (!((cdcm6208_reg(slave, 21, 0, 0) >> 1) & 0x3))
		{
			return 0;
		}
		
		udelay (1000);
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
