#ifndef BMP280
#define BMP280

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico/binary_info.h"


#ifndef SPI_INTERFACE
#define SPI_INTERFACE
    #define SPI_INTERFACE spi0
    #define SPI_BOUNDRATE 1000*1000
    #define SPI_MISO 0
    #define SPI_CLK  2
    #define SPI_MOSI 3
#endif
#define SPI_CS_BMP280  1

#ifndef INT_TO_BIN
#define INT_TO_BIN
    #define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
    #define BYTE_TO_BINARY(byte)  \
    (byte & 0x80 ? '1' : '0'), \
    (byte & 0x40 ? '1' : '0'), \
    (byte & 0x20 ? '1' : '0'), \
    (byte & 0x10 ? '1' : '0'), \
    (byte & 0x08 ? '1' : '0'), \
    (byte & 0x04 ? '1' : '0'), \
    (byte & 0x02 ? '1' : '0'), \
    (byte & 0x01 ? '1' : '0') 
#endif


namespace BMP280 {
    void readRegister(uint8_t *dataIn, uint8_t *dataOut, uint8_t len);

    void commandStart();
    void commandEnd();

    void test1() {
        const uint8_t len = 2;
        uint8_t command[len] {0xF7};
        uint8_t dest[len];
        readRegister(command, dest, len);
        printf(BYTE_TO_BINARY_PATTERN,  BYTE_TO_BINARY(dest[1]));
        printf("dsad\n");
        sleep_ms(1000);
    }

    void readRegister(uint8_t *dataIn, uint8_t *dataOut, uint8_t len) {
        for (int i = 0; i < len; i++) {
            dataIn[i] |= 0b1000'0000;
        }
        commandStart();
        spi_write_read_blocking(SPI_INTERFACE, dataIn, dataOut, len);
        commandEnd();


    }

    void commandStart() {
        gpio_put(SPI_CS_BMP280, 0);
    }

    void commandEnd() {
        gpio_put(SPI_CS_BMP280, 1);
    }

    void spiInit() {
        spi_init(SPI_INTERFACE, SPI_BOUNDRATE);

        gpio_set_function(SPI_MISO  ,  GPIO_FUNC_SPI);
        gpio_set_function(SPI_CLK   ,  GPIO_FUNC_SPI);
        gpio_set_function(SPI_MOSI  ,  GPIO_FUNC_SPI);
        bi_decl(bi_3pins_with_func(SPI_MISO, SPI_MOSI, SPI_CLK, GPIO_FUNC_SPI));

    }


    void init(bool spi_init = true){
        if (spi_init) spiInit();
        
        gpio_init(SPI_CS_BMP280);
        gpio_set_dir(SPI_CS_BMP280, GPIO_OUT);
        commandEnd();
    }

}

#endif