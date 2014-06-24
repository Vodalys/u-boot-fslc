#ifndef _IMX_SPI_CDCM6208_H_
#define _IMX_SPI_CDCM6208_H_

#include <linux/types.h>

extern struct spi_slave *spi_cdcm6208_probe(void);
extern void spi_cdcm6208_free(struct spi_slave *slave);
extern u32 cdcm6208_reg(struct spi_slave *slave, u32 reg, u32 val, u32 write);
extern u32 show_cdcm6208_info(struct spi_slave *slave);
extern u32 cdcm6208_calibrate (struct spi_slave *slave);

#endif /* _IMX_SPI_CDCM6208_H_ */
