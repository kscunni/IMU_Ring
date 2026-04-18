#include <ti/drivers/SPI.h>

extern SPI_Handle spiHandle; // Needs to be accessible by your icm_func_impl.h functions

/* * SPI Read Implementation 
 * Matches signature: int (*inv_imu_read_reg_t)(uint8_t reg, uint8_t *buf, uint32_t len);
 */
int spi_read_reg(uint8_t reg, uint8_t *buf, uint32_t len);

/* * SPI Write Implementation 
 * Matches signature: int (*inv_imu_write_reg_t)(uint8_t reg, const uint8_t *buf, uint32_t len);
 */
int spi_write_reg(uint8_t reg, const uint8_t *buf, uint32_t len);

/* * Microsecond Delay Implementation 
 * Matches signature: void (*sleep_us)(uint32_t us);
 */
void delay_us(uint32_t us);

