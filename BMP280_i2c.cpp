#ifndef BMP280
#define BMP280

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include <cinttypes>


#ifndef I2C_INTERFACE
    #define I2C_INTERFACE 
    #define I2C_PORT i2c0
    #define I2C_PIN_SDA 0
    #define I2C_PIN_SCL 1
    #define I2C_CLOCK 50*1000
#endif

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
    uint8_t ADDRESS = 0b1110'1100  >> 1;
    uint8_t ADDRESS_BONUS_IF_CE_HIGHT = 0b0000'0010;

    int32_t temp ;
    int32_t press;
    uint8_t lastTemp[4];
    uint8_t lastPressure[4];

    void setRegisters(uint8_t *dataIn, uint8_t len);
    void readRegister(uint8_t *dataIn, uint8_t *dataOut, uint8_t len);


    uint8_t getStatusRegister();
    bool isInUpdate(uint8_t status = 0xff);
    bool isMesurmentInProgres(uint8_t status = 0xff);
    
    void printStatus();
    void readModeRegister();

    const uint8_t CONFIG_REGISTER    = 0xF5;//
    const uint8_t CTRL_MEAS_REGISTER = 0xF4;//OVERSAMPLING temp/press, Power mode
    const uint8_t STATUS_REGISTER    = 0xF3;

    uint8_t CTRL_MEAS_REGISTER_CONTENT[2] {0x00};
    uint8_t CONFIG_REGISTER_CONTENT[2]    {0x00};


    //DATAREGISTERAS
    // xlsb -> lsb -> msb
    const uint8_t TEMP_MESSURMENT_REGISTERS[3]  = {0xFA, 0xFB, 0xFC};
    const uint8_t PRESS_MESSURMENT_REGISTERS[3] =   {0xF7, 0xF8, 0xF9};

    struct bmp280_calib_param
        {
            uint16_t dig_t1;
            int16_t dig_t2;
            int16_t dig_t3;
            uint16_t dig_p1;
            int16_t dig_p2;
            int16_t dig_p3;
            int16_t dig_p4;
            int16_t dig_p5;
            int16_t dig_p6;
            int16_t dig_p7;
            int16_t dig_p8;
            int16_t dig_p9;
            int32_t t_fine;
        };
    bmp280_calib_param CALIBRATION;
    //POWER_MODE
    //sleep no measurment | forced one measurment | normal measurment every TIME_STANDBY
    enum POWER_MODE {SLEEP = 0b0000'0000, FORCED_MODE = 0b0000'0010, NORMAL_MODE = 0b0000'0011};
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

    void getCalibrationRegisters() {
        uint8_t registers[24] = {
            0x88, 0x89,
            0x8A, 0x8B,
            0x8C, 0x8D,
            0x8E, 0x8F,
            0x90, 0x91,
            0x92, 0x93,
            0x94, 0x95,
            0x96, 0x97,
            0x98, 0x99,
            0x9A, 0x9B,
            0x9C, 0x9D,
            0x9E, 0x9F,
        };
        uint8_t content[24];
        
        readRegister(registers, content, 24);

        CALIBRATION.dig_t1 = ((uint16_t)(content[1] << 8)) | ((uint16_t)(content[0]));
        CALIBRATION.dig_t2 = ((int16_t)(content[3] << 8))  | ((int16_t)(content[2]));
        CALIBRATION.dig_t3 = ((int16_t)(content[5] << 8))  | ((int16_t)(content[4]));
        CALIBRATION.dig_p1 = ((uint16_t)(content[7] << 8)) | ((uint16_t)(content[6]));
        CALIBRATION.dig_p2 = ((int16_t)(content[9] << 8)) | ((int16_t)(content[8]));
        CALIBRATION.dig_p3 = ((int16_t)(content[11] << 8)) | ((int16_t)(content[10]));
        CALIBRATION.dig_p4 = ((int16_t)(content[13] << 8)) | ((int16_t)(content[12]));
        CALIBRATION.dig_p5 = ((int16_t)(content[15] << 8)) | ((int16_t)(content[14]));
        CALIBRATION.dig_p6 = ((int16_t)(content[17] << 8)) | ((int16_t)(content[16]));
        CALIBRATION.dig_p7 = ((int16_t)(content[19] << 8)) | ((int16_t)(content[18]));
        CALIBRATION.dig_p8 = ((int16_t)(content[21] << 8)) | ((int16_t)(content[20]));
        CALIBRATION.dig_p9 = ((int16_t)(content[23] << 8)) | ((int16_t)(content[22]));
    }


    int32_t bmp280_get_comp_temp_double(int32_t uncomp_temp)
    {
        int32_t temperature;
        int32_t var1, var2;

        var1 =
            ((((uncomp_temp / 8) - ((int32_t) CALIBRATION.dig_t1 << 1))) * ((int32_t) CALIBRATION.dig_t2)) /
            2048;
        var2 =
            (((((uncomp_temp / 16) - ((int32_t) CALIBRATION.dig_t1)) *
               ((uncomp_temp / 16) - ((int32_t) CALIBRATION.dig_t1))) / 4096) *
             ((int32_t) CALIBRATION.dig_t3)) /
            16384;
        CALIBRATION.t_fine = var1 + var2;
        temperature = (CALIBRATION.t_fine * 5 + 128) / 256;

        return temperature;
    }

    uint32_t bmp280_get_comp_pres_32bit(uint32_t uncomp_pres) {
        int32_t pressure;
        int32_t var1, var2;


        var1 = (((int32_t) CALIBRATION.t_fine) / 2) - (int32_t) 64000;
        var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t) CALIBRATION.dig_p6);
        var2 = var2 + ((var1 * ((int32_t) CALIBRATION.dig_p5)) * 2);
        var2 = (var2 / 4) + (((int32_t) CALIBRATION.dig_p4) * 65536);
        var1 =
            (((CALIBRATION.dig_p3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8) +
            ((((int32_t) CALIBRATION.dig_p2) * var1) / 2)) / 262144;
        var1 = ((((32768 + var1)) * ((int32_t) CALIBRATION.dig_p1)) / 32768);
        pressure = (uint32_t)(((int32_t)(1048576 - uncomp_pres) - (var2 / 4096)) * 3125);

        /* Avoid exception caused by division with zero */
        if (var1 != 0){
            /* Check for overflows against UINT32_MAX/2; if pres is left-shifted by 1 */
            if (pressure < 0x80000000)
            {
                pressure = (pressure << 1) / ((uint32_t) var1);
            }
            else
            {
                pressure = (pressure / (uint32_t) var1) * 2;
            }
            var1 = (((int32_t) CALIBRATION.dig_p9) * ((int32_t) (((pressure / 8) * (pressure / 8)) / 8192))) /
                4096;
            var2 = (((int32_t) (pressure / 4)) * ((int32_t) CALIBRATION.dig_p8)) / 8192;
            pressure = (uint32_t) ((int32_t) pressure + ((var1 + var2 + CALIBRATION.dig_p7) / 16));
        }else{
            pressure = 0;
        }

        return pressure;
    }

    void readMeasurmentsRaw() {
        printf("start\n");
        uint8_t dataIn[6], dataOut[6];

        dataIn[0] =   TEMP_MESSURMENT_REGISTERS[0];
        readRegister(dataIn, dataOut, 3);
        int32_t tmp = (int32_t) ((((int32_t) (dataOut[0])) << 12) | (((int32_t) (dataOut[1])) << 4) | (((int32_t) (dataOut[2])) >> 4));
            
        dataIn[0] = PRESS_MESSURMENT_REGISTERS[0];
        readRegister(dataIn, dataOut, 3);
        int32_t pre = (int32_t) ((((int32_t) (dataOut[0])) << 12) | (((int32_t) (dataOut[1])) << 4) | (((int32_t) (dataOut[2])) >> 4));

        temp= bmp280_get_comp_temp_double(tmp);
        press = bmp280_get_comp_pres_32bit(pre);
        printf("%03" PRId32 "\n",temp);
        printf("%03" PRId32 "\n",press);
        printf("end\n");
        lastTemp[0] = (uint8_t)(temp & 0xFF);
        lastTemp[1] = (uint8_t)((temp >> 8) & 0xFF);
        lastTemp[2] = (uint8_t)((temp >> 16) & 0xFF);
        lastTemp[3] = (uint8_t)((temp >> 24) & 0xFF);

        lastPressure[0] = (uint8_t)(temp & 0xFF);
        lastPressure[1] = (uint8_t)((temp >> 8) & 0xFF);
        lastPressure[2] = (uint8_t)((temp >> 16) & 0xFF);
        lastPressure[3] = (uint8_t)((temp >> 24) & 0xFF);
        //printf("OUTPUT %i \n", bmp280_get_comp_temp_double(tmp));
    }

    void setConfigRegister(TIME_STANDBY tSTBD) {
        CONFIG_REGISTER_CONTENT[0] = CONFIG_REGISTER;

        CONFIG_REGISTER_CONTENT[1] = tSTBD;
        setRegisters(CONFIG_REGISTER_CONTENT, 2);
    }

    void setOversampling(OVERSAMPLING_TEMPERATURE tmpOversampling, OVERSAMPLING_PRESSURE pressOversampling) {
        CTRL_MEAS_REGISTER_CONTENT[0] = CTRL_MEAS_REGISTER & 0b0111'1111;
        
        CTRL_MEAS_REGISTER_CONTENT[1] &= 0b000'000'11;
        CTRL_MEAS_REGISTER_CONTENT[1] |= tmpOversampling;
        CTRL_MEAS_REGISTER_CONTENT[1] |= pressOversampling;

        
        printf(BYTE_TO_BINARY_PATTERN,  BYTE_TO_BINARY(CTRL_MEAS_REGISTER_CONTENT[0]));
        printf("12\n");

        setRegisters(CTRL_MEAS_REGISTER_CONTENT, 2);
    }

    void setMode(POWER_MODE mode) {
        CTRL_MEAS_REGISTER_CONTENT[0] = CTRL_MEAS_REGISTER;
        
        CTRL_MEAS_REGISTER_CONTENT[1] &= 0b111'111'00;
        CTRL_MEAS_REGISTER_CONTENT[1] |= mode; 
        
        setRegisters(CTRL_MEAS_REGISTER_CONTENT, 2);
    }
    bool MesurmentInProgress() {
        uint8_t status = getStatusRegister();
        if(isMesurmentInProgres(status) || isInUpdate(status)) return true;
        else return false;
    }
    void takeMesurment() {

        setMode(POWER_MODE::FORCED_MODE);
        sleep_ms(2);
        while (MesurmentInProgress()) {
            sleep_ms(1);
        }
        readMeasurmentsRaw();

        sleep_ms(1000);
    }
    
    void readModeRegister() {

        uint8_t tin[1] {CTRL_MEAS_REGISTER};
        
        readRegister(tin, &CTRL_MEAS_REGISTER_CONTENT[1], 1);
        printf(BYTE_TO_BINARY_PATTERN,  BYTE_TO_BINARY(CTRL_MEAS_REGISTER_CONTENT[0]));
        printf("\n");

    }

    void printStatus() {
        uint8_t status = getStatusRegister();
        if (isInUpdate(status)) printf("Update: 1, ");
        else  printf("Update: 0, ");
        if (isMesurmentInProgres(status)) printf("Measurment: 1\n");
        else  printf("Measurment: 0\n");
        printf("Status: ");
        printf(BYTE_TO_BINARY_PATTERN,  BYTE_TO_BINARY(status));
        printf("\n");

        readModeRegister();
    }

    uint8_t getStatusRegister() {
        uint8_t dIn[1] {STATUS_REGISTER}, status[1];
        readRegister(dIn, status, 1);
        printf(BYTE_TO_BINARY_PATTERN,  BYTE_TO_BINARY(status[0]));
        printf("\n");
        return status[0];
    }

    bool isInUpdate(uint8_t status) {
        if (status == 0xff) status = getStatusRegister();
        return status & 0b0000'0001;
    }

    bool isMesurmentInProgres(uint8_t status) {
        if (status == 0xff) status = getStatusRegister();
        return status & 0b0000'1000;
    }

    void setRegisters(uint8_t *dataIn, uint8_t len) {
        i2c_write_blocking(I2C_PORT, ADDRESS, dataIn, len, false);
    }

    void readRegister(uint8_t *dataIn, uint8_t *dataOut, uint8_t len) {
        i2c_write_blocking(I2C_PORT, ADDRESS, dataIn, 1, false);
        i2c_read_blocking(I2C_PORT, ADDRESS , dataOut, len, false);
    }



    void i2cInit() {
        i2c_init(I2C_PORT, I2C_CLOCK);
        gpio_set_function(I2C_PIN_SDA, GPIO_FUNC_I2C);
        gpio_set_function(I2C_PIN_SCL, GPIO_FUNC_I2C);
        gpio_pull_up(I2C_PIN_SDA);
        gpio_pull_up(I2C_PIN_SCL);
    }


    void init(bool CE_HIGHT, bool i2cInitV = true){
        i2cInit();

        if (CE_HIGHT) {
            ADDRESS |= ADDRESS_BONUS_IF_CE_HIGHT;
        }
        sleep_ms(100);
        uint8_t c[1] {CTRL_MEAS_REGISTER}, o[1] = {0};
        CTRL_MEAS_REGISTER_CONTENT[1] = o[0];
        getCalibrationRegisters();
    }

}

#endif