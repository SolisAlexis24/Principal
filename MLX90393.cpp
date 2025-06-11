#include "MLX90393.h"

MLX90393::MLX90393(i2c_inst_t *i2c_port, uint sda_pin, uint scl_pin, uint i2c_freq, mutex_t* i2c_mutex)
        : i2c_port_(i2c_port), sda_pin_(sda_pin), scl_pin_(scl_pin), i2c_freq_(i2c_freq), i2c_mutex_(i2c_mutex) {
}

bool MLX90393::init_sensor(uint16_t offset_x, uint16_t offset_y, uint16_t offset_z,
                            RES res_x, RES res_y, RES res_z, DIG_FILT filt, OSR osr, OSR osr2, GAIN gain){
    if (this->verify_connection()) {
        printf("Sensor MLX90393 conectado y funcionando correctamente\n");
    } else {
        printf("Error: Problema con la conexión del sensor MLX90393\n");
        return false;
    }
    // Desactiva la comunicación SPI Registro 0x01
    //[TRIG_INT][COMM_MODE(13:14)][WOC_DIFF][EXT_TGR][TCMP_EN][BURS_SEL (6:9)][BDR (0:5)]
    //0110000000000000 -> 0x6000
    write_register(0x01, 0x6000);
    // 2 Bytes de configuracion para el registro 0x02
    // [000][OSR_2][RES_XYZ][DIG_FILT][OSR]
    uint16_t config = (osr2 << 11 ) | (res_z << 9) | (res_y << 7) | (res_x << 5) | (filt << 2) |(osr);
    write_register(0x02, config);

    // Configuracion de los offset del magnetometro
    write_register(0x04, offset_x); // Configura el offset en X
    write_register(0x05, offset_y); // Configura el offset en X
    write_register(0x06, offset_z); // Configura el offset en X

    aquisition_time_mag = calculate_aquisition_time_mag_us(filt, osr);
    aquisition_time_temp = calculate_aquisition_time_temp_us(osr2);

    res_x_ = res_x;
    res_y_ = res_y;
    res_z_ = res_z;
    gain_factor_ = gain_multipliers[int8_t(gain) & 0x07];
    base_sens_x_ = base_sens_y_ = 0.150f;
    base_sens_z_ = 0.242f;

    // Configuracion de la ganancia
    uint16_t old_data = read_register(0x00);        // Se guarda la anterior informacion del registro
    // Se calcula la nueva informacion.
    // 1. Primero se borra la anterior informacion de la ganancia: old_data & 0xFF8F (Mascara de todo menos GAIN_SEL)
    // 2. Se anade la nueva informacion de la ganancia ((uint16_t(gain) << 4) & 0x0070)
    uint16_t new_data = old_data & 0xFF8F | ((uint16_t(gain) << 4) & 0x0070);

    printf("MLX90393 inicializado. Tiempo de adquisición mag : %u us\n", this->aquisition_time_mag);
    printf("MLX90393 inicializado. Tiempo de adquisición temp : %u us\n", this->aquisition_time_temp);

    return true;
}

bool MLX90393::start_measurement_mag(){
    // Iniciar medicion
    uint8_t cmd[1];   // Comando de lectura de medicion
    uint8_t read_buf[1]; // Buffer que guardara status

    cmd[0] = SM_XYZ;

    mutex_enter_blocking(i2c_mutex_); // Asegurar acceso exclusivo al bus I2C
    // Paso 1: Comando de una sola medicion cada que el master lo solicita
    i2c_write_blocking(i2c_port_, ADDR, cmd, 1, true); // true: no enviar stop

    // Paso 2: Enviar repeated start y leer 1 byte (status)
    i2c_read_blocking(i2c_port_, ADDR, read_buf, 1, false);
    mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C

    if(read_buf[0] & 0x10){ printf("Error al iniciar la medición de magnetomero\n"); return false;}



    return true;
}

MLX90393::MLX90393Data MLX90393::read_measurement_mag(){
    // Iniciar medicion
    uint8_t cmd[1];   // Comando de lectura de medicion
    uint8_t read_buf[7]; // Buffer que guardara status y las lecturas de XYZ

    // Leer medicion
    cmd[0] = RM_XYZ;

    mutex_enter_blocking(i2c_mutex_); // Asegurar acceso exclusivo al bus I2C
    // Paso 1. Enviar comando de lectura de XYZ
    i2c_write_blocking(i2c_port_, ADDR, cmd, 1, true); // true: no enviar stop

    // Paso 2. Enviar reapeted start y leer los datos en el orden status y XYZ
    i2c_read_blocking(i2c_port_, ADDR, read_buf, 7, false);
    mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C


    if(read_buf[0] & 0x10) { printf("Error al obtener los datos del magnetometro\n"); return MLX90393Data();}

    uint16_t raw_x = (read_buf[1] << 8) | read_buf[2];
    uint16_t raw_y = (read_buf[3] << 8) | read_buf[4];
    uint16_t raw_z = (read_buf[5] << 8) | read_buf[6];

    switch (res_x_)
    {
        case RESOLUTION_MAX:
        case RESOLUTION_HIGH:
        this->last_measurement.x = int16_t(raw_x) * base_sens_x_ * gain_factor_ * (1 << (uint8_t)res_x_);
            break;
        case RESOLUTION_MEDIUM:
        this->last_measurement.x = ((raw_x - 32768.f) * base_sens_x_ * gain_factor_ * (1 << (uint8_t)res_x_));
            break;
        case RESOLUTION_LOW: 
        this->last_measurement.x = ((raw_x - 16384.f) * base_sens_x_ * gain_factor_ * (1 << (uint8_t)res_x_));
    }

    switch (res_y_)
    {
        case RESOLUTION_MAX:
        case RESOLUTION_HIGH:
        this->last_measurement.y = int16_t(raw_y) * base_sens_y_ * gain_factor_ * (1 << (uint8_t)res_y_);
            break;
        case RESOLUTION_MEDIUM:
        this->last_measurement.y = ((raw_y - 32768.f) * base_sens_y_ * gain_factor_ * (1 << (uint8_t)res_y_));
            break;
        case RESOLUTION_LOW: 
        this->last_measurement.y = ((raw_y - 16384.f) * base_sens_y_ * gain_factor_ * (1 << (uint8_t)res_y_));
    }

    switch (res_z_)
    {
        case RESOLUTION_MAX:
        case RESOLUTION_HIGH:
        this->last_measurement.z = int16_t(raw_z) * base_sens_z_ * gain_factor_ * (1 << (uint8_t)res_z_);
            break;
        case RESOLUTION_MEDIUM:
        this->last_measurement.z = ((raw_z - 32768.f) * base_sens_z_ * gain_factor_ * (1 << (uint8_t)res_z_));
            break;
        case RESOLUTION_LOW: 
        this->last_measurement.z = ((raw_z - 16384.f) * base_sens_z_ * gain_factor_ * (1 << (uint8_t)res_z_));
    }

    return this->last_measurement;
}

bool MLX90393::start_measurement_temp(){
    // Iniciar medicion
    uint8_t cmd[1];   // Comando de lectura de medicion
    uint8_t read_buf[3]; // Buffer que guardara status y las lecturas de T
    MLX90393Data datos;

    cmd[0] = SM_T;

    mutex_enter_blocking(i2c_mutex_); // Asegurar acceso exclusivo al bus I2C
    // Paso 1: Comando de una sola medicion cada que el master lo solicita
    i2c_write_blocking(i2c_port_, ADDR, cmd, 1, true); // true: no enviar stop

    // Paso 2: Enviar repeated start y leer 1 byte (status)
    i2c_read_blocking(i2c_port_, ADDR, read_buf, 1, false);
    mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C


    if(read_buf[0] & 0x10){ printf("Error al iniciar la medición de temperatura\n"); return false;}

    return true;
}

MLX90393::MLX90393Data MLX90393::read_measurement_temp(){
    // Iniciar medicion
    uint8_t cmd[1];   // Comando de lectura de medicion
    uint8_t read_buf[3]; // Buffer que guardara status y las lecturas de T

    // Leer medicion
    cmd[0] = RM_T;

    mutex_enter_blocking(i2c_mutex_); // Asegurar acceso exclusivo al bus I2C
    // Paso 1. Enviar comando de lectura de T
    i2c_write_blocking(i2c_port_, ADDR, cmd, 1, true); // true: no enviar stop

    // Paso 2. Enviar reapeted start y leer los datos en el orden status y T
    i2c_read_blocking(i2c_port_, ADDR, read_buf, 3, false);
    mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C

    if(read_buf[0] & 0x10) { printf("Error al obtener los dato de temperaturas\n"); return MLX90393Data();}

    uint16_t raw_temp = (read_buf[1] << 8) | read_buf[2];

    this->last_measurement.t = 25 + (raw_temp - 46244.f)/45.2f;

    return this->last_measurement;
}

bool MLX90393::begin_measurement_mt()
{
    // Iniciar medicion
    uint8_t cmd[1];   // Comando de lectura de medicion
    uint8_t read_buf[1]; // Buffer que guardara status

    cmd[0] = SM_XYZT;

    mutex_enter_blocking(i2c_mutex_); // Asegurar acceso exclusivo al bus I2C
    // Paso 1: Comando de una sola medicion cada que el master lo solicita
    i2c_write_blocking(i2c_port_, ADDR, cmd, 1, true); // true: no enviar stop

    // Paso 2: Enviar repeated start y leer 1 byte (status)
    i2c_read_blocking(i2c_port_, ADDR, read_buf, 1, false);
    mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C

    if(read_buf[0] & 0x10){ printf("Error al iniciar la medición de magnetomero y termometro\n"); return false;}


    return true;
}

MLX90393::MLX90393Data MLX90393::read_measurement_mt()
{
    uint8_t cmd[1];   // Comando de lectura de medicion
    uint8_t read_buf[9]; // Buffer que guardara status y las lecturas de XYZT

    // Leer medicion
    cmd[0] = RM_XYZT;

    mutex_enter_blocking(i2c_mutex_); // Asegurar acceso exclusivo al bus I2C
    // Paso 1. Enviar comando de lectura de XYZ
    i2c_write_blocking(i2c_port_, ADDR, cmd, 1, true); // true: no enviar stop

    // Paso 2. Enviar reapeted start y leer los datos en el orden status, T y XYZ
    i2c_read_blocking(i2c_port_, ADDR, read_buf, 9, false);
    mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C


    if(read_buf[0] & 0x10) { printf("Error al obtener los datos del magnetometro y termometro\n"); return MLX90393Data();}

    uint16_t raw_temp = (read_buf[1] << 8) | read_buf[2];
    uint16_t raw_x = (read_buf[3] << 8) | read_buf[4];
    uint16_t raw_y = (read_buf[5] << 8) | read_buf[6];
    uint16_t raw_z = (read_buf[7] << 8) | read_buf[8];

    switch (res_x_)
    {
        case RESOLUTION_MAX:
        case RESOLUTION_HIGH:
        this->last_measurement.x = int16_t(raw_x) * base_sens_x_ * gain_factor_ * (1 << (uint8_t)res_x_);
            break;
        case RESOLUTION_MEDIUM:
        this->last_measurement.x = ((raw_x - 32768.f) * base_sens_x_ * gain_factor_ * (1 << (uint8_t)res_x_));
            break;
        case RESOLUTION_LOW: 
        this->last_measurement.x = ((raw_x - 16384.f) * base_sens_x_ * gain_factor_ * (1 << (uint8_t)res_x_));
    }

    switch (res_y_)
    {
        case RESOLUTION_MAX:
        case RESOLUTION_HIGH:
        this->last_measurement.y = int16_t(raw_y) * base_sens_y_ * gain_factor_ * (1 << (uint8_t)res_y_);
            break;
        case RESOLUTION_MEDIUM:
        this->last_measurement.y = ((raw_y - 32768.f) * base_sens_y_ * gain_factor_ * (1 << (uint8_t)res_y_));
            break;
        case RESOLUTION_LOW: 
        this->last_measurement.y = ((raw_y - 16384.f) * base_sens_y_ * gain_factor_ * (1 << (uint8_t)res_y_));
    }

    switch (res_z_)
    {
        case RESOLUTION_MAX:
        case RESOLUTION_HIGH:
        this->last_measurement.z = int16_t(raw_z) * base_sens_z_ * gain_factor_ * (1 << (uint8_t)res_z_);
            break;
        case RESOLUTION_MEDIUM:
        this->last_measurement.z = ((raw_z - 32768.f) * base_sens_z_ * gain_factor_ * (1 << (uint8_t)res_z_));
            break;
        case RESOLUTION_LOW: 
        this->last_measurement.z = ((raw_z - 16384.f) * base_sens_z_ * gain_factor_ * (1 << (uint8_t)res_z_));
    }

    this->last_measurement.t = 25 + (raw_temp - 46244.f)/45.2f;
    return this->last_measurement;
}

uint16_t MLX90393::read_register(uint8_t reg){
    uint8_t cmd[2];      // Comando a ser enviado y registro a ser leido
    uint8_t read_buf[3]; // status + 2 data bytes

    cmd[0] = RR;        // Byte 1: Comando de lectura
    cmd[1] = reg << 2;  // Byte 2: Registro desplazado 2 bits a la izquierda

    // Paso 1: Escribir comando RR + dirección de registro
    if(i2c_write_blocking(i2c_port_, ADDR, cmd, 2, true) != 2){ // true: no enviar stop
        printf("Error al escribir el comando de lectura de registro para 0x%0X\n", reg);
        return 0;
    } 

    // Paso 2: Enviar repeated start y leer 3 bytes (status + MSB + LSB)
    if(i2c_read_blocking(i2c_port_, ADDR, read_buf, 3, false) != 3){
        printf("Error al obtener los datos");
        return 0;
    }

    // Combinar los datos
    uint16_t value = (read_buf[1] << 8) | read_buf[2];
    return value;

}

void MLX90393::write_register(uint8_t reg, uint16_t val){
    uint8_t cmd[4];     // Comando a ser enviado, registro de escritura, parte alta de valor y parte baja del valor
    uint8_t read_buf[1]; // Status

    cmd[0] = WR;                    // Byte 1: Comando de escritura
    cmd[1] = (val >> 8) & 0xFF;     // Byte 2: parte alta de informacion
    cmd[2] = val & 0xFF;            // Byte 3: parte baja de informacion
    cmd[3] = reg << 2;              // Byte 4: Registro desplazado 2 bits a la izquierda

    // Paso 1: Escribir comando WR + informacion + dirección de registro
    if(i2c_write_blocking(i2c_port_, ADDR, cmd, 4, true) != 4){ // true: no enviar stop
        printf("Error al escribir el comando de escritura de registro para 0x%0X\n", reg);
        return;
    } 

    // Paso 2: Enviar repeated start y leer 1 byte (status)
    if(i2c_read_blocking(i2c_port_, ADDR, read_buf, 1, false) != 1){
        printf("Error al obtener el byte de status");
        return;
    }

}

uint16_t MLX90393::calculate_aquisition_time_mag_us(DIG_FILT filt, OSR osr) {
    return (264 + 432 + 3 * (67 + 64 * pow(2, (int)osr) * (2 + pow(2, (int)filt))))*1.3;
}

uint16_t MLX90393::calculate_aquisition_time_temp_us(OSR osr2){
    return (67 + 192 * pow(2, (int)osr2))*1.3;
}

bool MLX90393::verify_connection() {

    uint16_t reg_0x00 = read_register(0x00);
    uint8_t hallconf = reg_0x00 & 0x0F;

    printf("Registro 0x00 leído: 0x%04X (HALLCONF = 0x%X)\n", 
          reg_0x00, hallconf);

    // 6. Verificar si HALLCONF tiene el valor por defecto (0xC)
    if (hallconf != 0xC) {
        printf("Advertencia: HALLCONF (0x%X) no tiene el valor por defecto (0xC)\n", hallconf);
        // Puedes decidir si considerar esto como error o no
        return false; // Descomentar para exigir el valor por defecto
    }

    return true;
}