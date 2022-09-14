// #include "stm32f10x.h"
#include "GY521.h"

//**************************************
//向I2C设备写入一个字节数据
//**************************************
// bool mpu6050_register_write_byte(u8 REG_Address, u8 REG_data)
// {
//     IIC_Start();                //起始信号
//     IIC_SendBits(SlaveAddress); //发送设备地址+写信号

//     if (!IIC_WACK())
//         return false;

//     IIC_SendBits(REG_Address); //内部寄存器地址，

//     if (!IIC_WACK())
//         return false;

//     IIC_SendBits(REG_data); //内部寄存器数据，

//     if (!IIC_WACK())
//         return false;

//     IIC_End(); //发送停止信号
//     return true;
// }

//**************************************
//从I2C设备读取一个字节数据
//**************************************
// bool mpu6050_register_read(u8 REG_Address, u8 *a)
// {
//     IIC_Start();                //起始信号
//     IIC_SendBits(SlaveAddress); //发送设备地址+写信号
//     if (!IIC_WACK())
//         return false;
//     IIC_SendBits(REG_Address); //发送存储单元地址，从0开始
//     if (!IIC_WACK())
//         return false;
//     //IIC_End();
//     IIC_Start();                    //起始信号
//     IIC_SendBits(SlaveAddress + 1); //发送设备地址+读信号
//     if (!IIC_WACK())
//         return false;
//     *a = IIC_ReadBits(0); //读出寄存器数据	0 	NACK   1  ACK
//     IIC_End();            //停止信号
//     return true;
// }
/**
 * @brief Read a sequence of bytes from a mpu6050 sensor registers
 */
int mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, mpu6050_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

/**
 * @brief Write a byte to a mpu6050 sensor register
 */
int mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, mpu6050_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 1,
        .scl_io_num = 2,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
//**************************************
//初始化MPU6050
//**************************************
int GY521_Init(void){
    i2c_master_init();
    vTaskDelay(300);
    uint8_t H;
    mpu6050_register_read(0x01, &H,1);
    ESP_LOGE("@@@:","%d",H);
    // mpu6050_register_write_byte(PWR_MGMT_1, 0x00);	//解除休眠状态
    // mpu6050_register_write_byte(SMPLRT_DIV, 0x07);
    // mpu6050_register_write_byte(CONFIG, 0x06);
    // mpu6050_register_write_byte(GYRO_CONFIG, 0x18);
    // mpu6050_register_write_byte(ACCEL_CONFIG, 0x01);

    int a = mpu6050_register_write_byte(MPU_60X0_PWR_MGMT_1_REG_ADDR, MPU_60X0_RESET_REG_VALU);
    if (a)
        return -8;
    vTaskDelay(1);
    a = mpu6050_register_write_byte(MPU_60X0_PWR_MGMT_1_REG_ADDR, MPU_60X0_PWR_MGMT_1_REG_VALU);
    if (a)
        return -2;
    vTaskDelay(1);
    a = mpu6050_register_write_byte(MPU_60X0_USER_CTRL_REG_ADDR, MPU_60X0_USER_CTRL_REG_VALU);
    if (a)
        return -3;
    vTaskDelay(1);
    a = mpu6050_register_write_byte(MPU_60X0_SMPLRT_DIV_REG_ADDR, MPU_60X0_SMPLRT_DIV_REG_VALU);
    if (a)
        return -4;
    vTaskDelay(1);
    a = mpu6050_register_write_byte(MPU_60X0_CONFIG_REG_ADDR, MPU_60X0_CONFIG_REG_VALU);
    if (a)
        return -5;
    vTaskDelay(1);
    a = mpu6050_register_write_byte(MPU_60X0_GYRO_CONFIG_REG_ADDR, MPU_60X0_GYRO_CONFIG_REG_VALU);
    if (a)
        return -6;
    vTaskDelay(1);
    
    a = mpu6050_register_write_byte(MPU_60X0_ACCEL_CONFIG_REG_ADDR, MPU_60X0_ACCEL_CONFIG_REG_VALU);
    if (a)
        return -7;
    vTaskDelay(1);
    a = mpu6050_register_write_byte(MPU_60X0_FIFO_EN_REG_ADDR, MPU_60X0_FIFO_EN_REG_VALU);
    if (a)
        return -8;
    return 0;
}



//**************************************
//合成数据
//**************************************
short GetData(uint8_t REG_Address)
{
    uint8_t H = 0, L = 0;
    int err;
    err = mpu6050_register_read(REG_Address, &H,1);
    if (err)
        return err;
    err = mpu6050_register_read(REG_Address, &L,1);
    if (err)
        return err;
    // ESP_LOGE("!!!","H:%d,l:%d",H,L);
    return (H<<8)+L; //合成数据
}