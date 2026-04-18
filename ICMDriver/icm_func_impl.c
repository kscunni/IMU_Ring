#include <ti/drivers/SPI.h>
#include <unistd.h>           // For TI standard POSIX usleep
#include "ti_drivers_config.h"
#include "imu/inv_imu_driver.h"
#include <ti/drivers/GPIO.h>

// Global SPI handle to be populated during initialization
SPI_Handle spiHandle;

// /* * SPI Read Implementation 
//  * Matches signature: int (*inv_imu_read_reg_t)(uint8_t reg, uint8_t *buf, uint32_t len);
//  */
// int spi_read_reg(uint8_t reg, uint8_t *buf, uint32_t len) {
//     SPI_Transaction spiTransaction;
//     uint8_t txBuffer[256]; // Adjust size based on your max expected read length
//     uint8_t rxBuffer[256];

//     if (len > 255) return -1; // Safeguard

//     // Set MSB to 1 for SPI Read operation
//     txBuffer[0] = reg | 0x80;
    
//     // The rest of txBuffer doesn't matter for a read, but initializes the clocks
//     memset(&txBuffer[1], 0, len); 

//     SPI_Transaction_init(&spiTransaction);
//     spiTransaction.count = len + 1;       // 1 byte for reg address + data length
//     spiTransaction.txBuf = (void *)txBuffer;
//     spiTransaction.rxBuf = (void *)rxBuffer;

//     if (SPI_transfer(spiHandle, &spiTransaction)) {
//         // Copy the received payload back to the driver's buffer
//         memcpy(buf, &rxBuffer[1], len);
//         return 0; // 0 on success
//     }
//     return -1; // Negative on error
// }

// /* * SPI Write Implementation 
//  * Matches signature: int (*inv_imu_write_reg_t)(uint8_t reg, const uint8_t *buf, uint32_t len);
//  */
// int spi_write_reg(uint8_t reg, const uint8_t *buf, uint32_t len) {
//     SPI_Transaction spiTransaction;
//     uint8_t txBuffer[256];

//     if (len > 255) return -1;

//     // Set MSB to 0 for SPI Write operation
//     txBuffer[0] = reg & 0x7F;
//     memcpy(&txBuffer[1], buf, len);

//     SPI_Transaction_init(&spiTransaction);
//     spiTransaction.count = len + 1;       // 1 byte for reg address + data length
//     spiTransaction.txBuf = (void *)txBuffer;
//     spiTransaction.rxBuf = NULL;          // We don't care about incoming data during a write

//     if (SPI_transfer(spiHandle, &spiTransaction)) {
//         return 0; // 0 on success
//     }
//     return -1; // Negative on error
// }

/* * SPI Read Implementation */
int spi_read_reg(uint8_t reg, uint8_t *buf, uint32_t len) {
    // Zero-initialize the structure right at declaration
    SPI_Transaction spiTransaction = {0}; 
    uint8_t txBuffer[256]; 
    uint8_t rxBuffer[256];

    if (len > 255) return -1; // Safeguard

    // Set MSB to 1 for SPI Read operation
    txBuffer[0] = reg | 0x80;
    
    // The rest of txBuffer doesn't matter for a read, but it clocks the bus
    memset(&txBuffer[1], 0, len); 

    spiTransaction.count = len + 1;       // 1 byte for reg address + data length
    spiTransaction.txBuf = (void *)txBuffer;
    spiTransaction.rxBuf = (void *)rxBuffer;

    GPIO_write(CONFIG_GPIO_0, 0);
    bool success = SPI_transfer(spiHandle, &spiTransaction);
    GPIO_write(CONFIG_GPIO_0, 1);

    if (success) {
        // Copy the received payload back to the driver's buffer
        memcpy(buf, &rxBuffer[1], len);
        return 0; // 0 on success
    }
    return -1; // Negative on error
}

/* * SPI Write Implementation */
int spi_write_reg(uint8_t reg, const uint8_t *buf, uint32_t len) {
    // Zero-initialize the structure right at declaration
    SPI_Transaction spiTransaction = {0}; 
    uint8_t txBuffer[256];

    if (len > 255) return -1;

    // Set MSB to 0 for SPI Write operation
    txBuffer[0] = reg & 0x7F;
    memcpy(&txBuffer[1], buf, len);

    spiTransaction.count = len + 1;       // 1 byte for reg address + data length
    spiTransaction.txBuf = (void *)txBuffer;
    spiTransaction.rxBuf = NULL;          // We don't care about incoming data during a write

    GPIO_write(CONFIG_GPIO_0, 0);
    bool success = SPI_transfer(spiHandle, &spiTransaction);
    GPIO_write(CONFIG_GPIO_0, 1);
    if (success) {
        return 0; // 0 on success
    }
    return -1; // Negative on error
}

/* * Microsecond Delay Implementation 
 * Matches signature: void (*sleep_us)(uint32_t us);
 */
void delay_us(uint32_t us) {
    // Standard POSIX delay function provided by TI-RTOS / TI SDK
    usleep(us);
}