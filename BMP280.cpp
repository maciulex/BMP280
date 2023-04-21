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
    bool debug = true;

    void setRegisters(uint8_t *dataIn, uint8_t len);
    void spi_send(uint8_t *dataIn, uint8_t *dataOut, uint8_t len);
    void spi_just_send(uint8_t *dataIn, uint8_t len);
    void readRegister(uint8_t *dataIn, uint8_t *dataOut, uint8_t len);

    uint8_t getStatusRegister();
    bool isInUpdate(uint8_t status = 0xff);
    bool isMesurmentInProgres(uint8_t status = 0xff);
    
    const uint8_t CONFIG_REGISTER    = 0xF5;//
    const uint8_t CTRL_MEAS_REGISTER = 0xF4;//OVERSAMPLING temp/press, Power mode
    const uint8_t STATUS_REGISTER    = 0xF3;

    uint8_t CTRL_MEAS_REGISTER_CONTENT[2] {0x00};
    uint8_t CONFIG_REGISTER_CONTENT[2]    {0x00};


    //DATAREGISTERAS
    // xlsb -> lsb -> msb
    const uint8_t TEMP_MESSURMENT_REGISTERS[3] = {0xFC, 0xFB, 0xFA};
    const uint8_t TEMP_PRESSURE_REGISTERS[3] =   {0xF9, 0xF8, 0xF7};



    //POWER_MODE
    //sleep no measurment | forced one measurment | normal measurment every TIME_STANDBY
    enum POWER_MODE {SLEEP = 0b0000'0000, FORCED_MODE = 0b0000'0010, NORMAL_MODE = 0b0000'0000};
    //oversampling temp
    enum OVERSAMPLING_TEMPERATURE {OSRS_T_0 = 0b0000'0000, OSRS_T_1 = 0b0010'0000, OSRS_T_2 = 0b0100'0000, OSRS_T_4 = 0b0110'0000, OSRS_T_8 = 0b1000'0000, OSRS_T_16 = 0b1110'0000};
    //skipping mesurment
    //16 bit / 0.0050 °C
    //17 bit / 0.0025 °C
    //18 bit / 0.0012 °C
    //19 bit / 0.0006 °C
    //20 bit / 0.0003 °C
   
    //oversampling press
    enum OVERSAMPLING_PRESSURE {OSRS_P_0 = 0b0000'0000, OSRS_P_1 = 0b0000'0100, OSRS_P_2 = 0b0000'1000, OSRS_P_4 = 0b0000'1100, OSRS_P_8 = 0b0001'0000, OSRS_P_16 = 0b0001'1100};
    //skipping mesurment
    //16 bit / 2.62 Pa
    //17 bit / 1.31 Pa
    //18 bit / 0.66 Pa
    //19 bit / 0.33 Pa
    //20 bit / 0.16 Pa

    //time beetween auto measurments in normal mode
    enum TIME_STANDBY {t05 = 0b0000'0000, t62 = 0b0010'0000, t125 = 0b0100'0000, t250 = 0b0110'0000, t500 = 0b1000'0000, t1000 = 0b1010'0000, t2000 = 0b1100'0000, t4000 = 0b1110'0000};

    const uint8_t RESET_WORLD    = 0xB6;
    const uint8_t RESET_REGISTER = 0xE0;

    void commandStart();
    void commandEnd();

    void setOversampling(OVERSAMPLING_TEMPERATURE tmpOversampling, OVERSAMPLING_PRESSURE pressOversampling) {
        CTRL_MEAS_REGISTER_CONTENT[0] = CTRL_MEAS_REGISTER & 0b0111'1111;
        
        CTRL_MEAS_REGISTER_CONTENT[1]  = tmpOversampling;
        CTRL_MEAS_REGISTER_CONTENT[1] |= pressOversampling;
        
        printf(BYTE_TO_BINARY_PATTERN,  BYTE_TO_BINARY(CTRL_MEAS_REGISTER_CONTENT[0]));
        printf("12\n");


        setRegisters(CTRL_MEAS_REGISTER_CONTENT, 2);
    }

    void test1() {
        const uint8_t len = 2;
        uint8_t command[len] {0xF7};
        uint8_t dest[len];


        sleep_ms(1000);
    }



    uint8_t getStatusRegister() {
        uint8_t dIn[1] {STATUS_REGISTER}, status[1];
        readRegister(dIn, status, 1);
        return status[0];
    }

    bool isInUpdate(uint8_t status) {
        if (status == 0xff) status = getStatusRegister();
        return status  & 0b0000'0001;
    }

    bool isMesurmentInProgres(uint8_t status) {
        if (status == 0xff) status = getStatusRegister();
        return status & 0b0000'1000;
    }

    void setRegisters(uint8_t *dataIn, uint8_t len) {
        spi_just_send(dataIn, len);
    }

    void readRegister(uint8_t *dataIn, uint8_t *dataOut, uint8_t len) {
        spi_send(dataIn, dataOut, len);
    }

    void spi_just_send(uint8_t *dataIn, uint8_t len) {
        commandStart();
        spi_write_blocking(SPI_INTERFACE, dataIn, len);
        commandEnd();
    }

    void spi_send(uint8_t *dataIn, uint8_t *dataOut, uint8_t len) {
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