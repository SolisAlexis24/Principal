
#include "MS5803-14BA.h"


MS5803::MS5803(i2c_inst_t *i2c_port, uint sda_pin, uint scl_pin, uint i2c_freq, mutex_t* i2c_mutex):
        i2c_port_(i2c_port), sda_pin_(sda_pin), scl_pin_(scl_pin), i2c_freq_(i2c_freq), i2c_mutex_(i2c_mutex),
        default_press_osr(P_OSR_256), default_temp_osr(T_OSR_256)
{
}

bool MS5803::init_sensor(TemperatureOSR t_osr, PressureOSR p_osr, Address add){
    this->ADDR = add;
    default_press_osr = p_osr;
    default_temp_osr = t_osr;
    
    if(!reset()) return false;

    // Leer todos los coeficientes de la PROM
    uint16_t prom[8];
    for (int i = 0; i < 8; ++i) {
        prom[i] = read_from_prom(0xA0 + (i * 2));
    }

    // Verificar CRC
    uint8_t crc_read = prom[7] & 0xF;
    if (!verify_crc(prom, crc_read)) {
        printf("MS5803-14BA: ¡Error de CRC en la PROM del sensor!\n");
        return false;
    }

    // Asignar coeficientes
    C1 = prom[1];
    C2 = prom[2];
    C3 = prom[3];
    C4 = prom[4];
    C5 = prom[5];
    C6 = prom[6];
    CRC = crc_read;

    //printf("C1: %hu\nC2: %hu\nC3: %hu\nC4: %hu\nC5: %hu\nC6: %hu\n", C1, C2, C3, C4, C5, C6);

    acquisition_time = calculate_acquisition_time_ms(t_osr, p_osr);

    printf("MS5803-14BA: Inicializado correctamente.\n");
    //printf("Tiempo de activacion maximo: %u ms \n", acquisition_time);

    return true;
}

bool MS5803::start_measurement_temp(){
    return start_measurement_temp(default_temp_osr);
}

bool MS5803::start_measurement_temp(TemperatureOSR t_osr){
    // Iniciar medicion de temperatura
    uint8_t cmd[1];   // Comando de inicio de conversion

    cmd[0] = (uint8_t)t_osr;

    mutex_enter_blocking(i2c_mutex_); // Asegurar acceso exclusivo al bus I2C
    // Enviar comando de conversion de temperatura
    if(i2c_write_blocking(i2c_port_, ADDR, cmd, 1, false) != 1){
        printf("MS5803-14BA: Error al escribir el comando de inicio de conversion de temperatura\n");
        mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C
        return false;
    } 
    mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C
    return true;
}

uint32_t MS5803::get_measurement(){
    // Leer la medicion hecha
    uint8_t cmd[1] = {R_ADC};   // Comando de lectura de ADC
    uint8_t read_buf[3];        // Buffer para almacenar lecturas

    mutex_enter_blocking(i2c_mutex_); // Asegurar acceso exclusivo al bus I2C
    // Paso 1: Enviar comando de lectura del ADC de temperatura
    if(i2c_write_blocking(i2c_port_, ADDR, cmd, 1, false) != 1){
        printf("MS5803-14BA: Error al escribir el comando de inicio de lectura de temperatura\n");
        mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C
        return 0;
    } 
    mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C
    
    mutex_enter_blocking(i2c_mutex_); // Asegurar acceso exclusivo al bus I2C
    // Paso 2: Leer 3 bytes de datos de temperatura
    if(i2c_read_blocking(i2c_port_, ADDR, read_buf, 3, false) != 3){
        printf("MS5803-14BA: Error al obtener los datos de la temperatura");
        mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C
        return 0;
    }
    mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C
    // read_buf[0] = Data 16-23
    // read_buf[1] = Data 8-15
    // read_buf[2] = Data 0-7
    uint32_t result = ((uint32_t)read_buf[0] << 16) | ((uint32_t)read_buf[1] << 8) | read_buf[2];
    return result;
}

MS5803::MS5803Data MS5803::read_measurement_temp(){
    // Acomodar los datos obtenidos (24 bits)
    D2 = get_measurement();
    // Calculo de la temperatura
    dT = D2 - ((int32_t)C5 << 8);
    TEMP = 2000 + (((int64_t)dT * C6) >> 23);

    // Compensacion de segundo orden. Consulta el datasheet, papito
    if(TEMP < 2000){
        T2 = 3 * (((int64_t)dT * dT) >> 33);
        OFF2 = 3 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
        SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 8;
        if(TEMP < -1500){
            OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
            SENS2 = SENS2 + 4 * ((TEMP + 1500) * (TEMP + 1500));
        }
    }else{
        T2 =  7  * ((uint64_t)dT * dT) / pow(2, 37);
        OFF2 = ((TEMP - 2000) * (TEMP - 2000)) / 16;
        SENS2 = 0;
    }
    TEMP = TEMP - T2;

    this->last_measurement.temperature = TEMP / 100.f;

    return last_measurement;
}

bool MS5803::start_measurement_press() {
    return start_measurement_press(default_press_osr);
}

bool MS5803::start_measurement_press(PressureOSR p_osr){
    // Iniciar medicion de presion
    uint8_t cmd[1];   // Comando de inicio de conversion
    uint8_t read_buf[3]; // Buffer

    cmd[0] = (uint8_t)p_osr;

    mutex_enter_blocking(i2c_mutex_); // Asegurar acceso exclusivo al bus I2C
    // Enviar comando de conversion de temperatura
    if(i2c_write_blocking(i2c_port_, ADDR, cmd, 1, false) != 1){
        printf("MS5803-14BA: Error al escribir el comando de inicio de conversion de presion\n");
        mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C
        return false;
    } 
    mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C
    return true;  
}

MS5803::MS5803Data MS5803::read_measurement_press(){

    D1 = get_measurement();

    OFF = ((int64_t)C2 << 16) + (((C4 * (int64_t)dT)) >> 7);

    SENS = ((int64_t)C1 << 15) + (((C3 * (int64_t)dT)) >> 8);

    // Por compensacion de segundo orden
    OFF = OFF - OFF2;

    // Por compensacion de segundo orden
    SENS = SENS - SENS2;

    P = (((D1 * SENS) / 2097152) - OFF) / 32768;

    this->last_measurement.pressure = P / 10.f;

    return this->last_measurement;
}


uint16_t MS5803::read_from_prom(uint8_t reg) {
    if (reg < 0xA0 || reg > 0xAE || (reg & 0x1)) {
        printf("MS5803-14BA: Registro no válido para lectura PROM: 0x%02X\n", reg);
        return 0;
    }
    uint8_t cmd[1] = {reg};     // Comando de lectura de registro
    uint8_t read_buf[2];        // Buffer

    // Enviar comando de lectura de cierto registro
    if(i2c_write_blocking(i2c_port_, ADDR, cmd, 1, false) != 1){
        printf("MS5803-14BA: Error al escribir el comando de lectura de registro para 0x%0X\n", reg);
        return 0;
    } 

    // Leer el valor del registro
    if(i2c_read_blocking(i2c_port_, ADDR, read_buf, 2, false) != 2){
        printf("MS5803-14BA: Error al obtener los datos de la PROM para el registro 0x%0X\n", reg);
        return 0;
    }
    
    return (read_buf[0] << 8) | read_buf[1];
}

uint8_t MS5803::calculate_acquisition_time_ms(TemperatureOSR t_osr, PressureOSR p_osr){
    int8_t t_time, p_time;
    switch (t_osr){
    case T_OSR_256: t_time = 1; break;
    case T_OSR_512: t_time = 2; break;
    case T_OSR_1024: t_time = 3; break;
    case T_OSR_2048: t_time = 6; break;
    case T_OSR_4096: t_time = 10; break;
    }
    switch (p_osr){
    case P_OSR_256: p_time = 1; break;
    case P_OSR_512: p_time = 2; break;
    case P_OSR_1024: p_time = 3; break;
    case P_OSR_2048: p_time = 6; break;
    case P_OSR_4096: p_time = 10; break;
    }
    return std::max(t_time, p_time);
}

bool MS5803::verify_crc(uint16_t prom_coeffs[8], uint8_t crc_read) {
    uint16_t n_rem = 0x00;
    uint16_t crc_calc;
    uint8_t cnt;
    uint8_t n_bit;

    prom_coeffs[7] = (prom_coeffs[7] & 0xFF00); // CRC byte is replaced by zero

    for (cnt = 0; cnt < 16; cnt++) {
        if (cnt % 2 == 1)
            n_rem ^= (uint8_t)(prom_coeffs[cnt >> 1] & 0x00FF);
        else
            n_rem ^= (uint8_t)(prom_coeffs[cnt >> 1] >> 8);

        for (n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000)
                n_rem = (n_rem << 1) ^ 0x3000;
            else
                n_rem = (n_rem << 1);
        }
    }
    n_rem = (n_rem >> 12) & 0xF; // final 4-bit CRC
    crc_calc = n_rem;
    return (crc_calc == crc_read);
}

bool MS5803::reset(){
    uint8_t cmd[1] = {RESET};       // Comando de reset para el sensor

    // Enviar comando de reset
    if(i2c_write_blocking(i2c_port_, ADDR, cmd, 1, false) != 1){
        printf("MS5803-14BA: Error al intentar reiniciar el controlador\n");
        return false;
    }  
    // Espera recomendada
    sleep_ms(10);
    return true;
}