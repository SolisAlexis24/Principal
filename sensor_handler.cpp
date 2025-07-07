#include "sensor_handler.h"
// Variables globales para los sensores
SensorHandler<LSM9DS1::LSM9DS1Data> LSM_handler;
SensorHandler<MLX90393::MLX90393Data> MLX_mag_handler;
SensorHandler<MLX90393::MLX90393Data> MLX_temp_handler;
SensorHandler<MS5803::MS5803Data> MS5803_handler;
SensorHandler<VEML6030::VEML6030Data> VEML_handler;
SensorHandler<AM2302::AM2302Data> AM23_handler;
SensorHandler<ISL29125::ISL29125Data> ISL_handler;

void blink_led(uint8_t count, uint16_t delay_ms) {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    for(uint8_t i = 0; i < count; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(delay_ms);
        gpio_put(LED_PIN, 0);
        sleep_ms(delay_ms);
    }
}

bool abrir_archivo(FIL* fil, const char* filename) {
    FRESULT fr = f_open(fil, filename, FA_OPEN_APPEND | FA_WRITE);
    if (FR_OK != fr && FR_EXIST != fr) {
        printf("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
        blink_led(5, 200); // Error de apertura
        return false;
    }
    return true;
}

bool cerrar_archivo(FIL* fil) {
    FRESULT fr = f_close(fil);
    if (FR_OK != fr) {
        printf("Error al cerrar archivo: %s (%d)\n", FRESULT_str(fr), fr);
        blink_led(7, 200); // Error de cierre
        return false;
    }
    return true;
}

bool guardar_mediciones_LSM9DS1() {
    if(!abrir_archivo(LSM_handler.file, LSM_handler.filename)) return false;
    if (f_printf(LSM_handler.file, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%lld\n",
                 LSM_handler.current.accel[0], LSM_handler.current.accel[1], LSM_handler.current.accel[2],
                 LSM_handler.current.gyro[0], LSM_handler.current.gyro[1], LSM_handler.current.gyro[2],
                 LSM_handler.current.mag[0], LSM_handler.current.mag[1], LSM_handler.current.mag[2], 
                 LSM_handler.current.time_ms) < 0) {
        printf("f_printf failed\n");
        blink_led(6, 200); // Error de escritura
        return false;
    }
    if(!cerrar_archivo(LSM_handler.file)) return false;
    return true;
}

bool guardar_mediciones_mag_MLX90393(){
    if(!abrir_archivo(MLX_mag_handler.file, MLX_mag_handler.filename)) return false;
    if (f_printf(MLX_mag_handler.file, "%.2f,%.2f,%.2f,%lld\n",
                 MLX_mag_handler.current.x, 
                 MLX_mag_handler.current.y, 
                 MLX_mag_handler.current.z, 
                 MLX_mag_handler.current.time_ms) < 0) {
        printf("f_printf failed\n");
        blink_led(6, 200); // Error de escritura
        return false;
    }
    if(!cerrar_archivo(MLX_mag_handler.file)) return false;
    return true;   
}

bool guardar_mediciones_temp_MLX90393(){
    if(!abrir_archivo(MLX_temp_handler.file, MLX_temp_handler.filename)) return false;
    if (f_printf(MLX_temp_handler.file, "%.2f,%lld\n",
                 MLX_temp_handler.current.t, 
                 MLX_temp_handler.current.time_ms/1000) < 0) {
        printf("f_printf failed\n");
        blink_led(6, 200); // Error de escritura
        return false;
    }
    if(!cerrar_archivo(MLX_temp_handler.file)) return false;
    return true;  
}

bool guardar_mediciones_MS5003(){
    if(!abrir_archivo(MS5803_handler.file, MS5803_handler.filename)) return false;
    if (f_printf(MS5803_handler.file, "%.2f,%.2f,%lld\n",
                 MS5803_handler.current.temperature,
                 MS5803_handler.current.pressure, 
                 MS5803_handler.current.time_ms/1000) < 0) {
        printf("f_printf failed\n");
        blink_led(6, 200); // Error de escritura
        return false;
    }
    if(!cerrar_archivo(MS5803_handler.file)) return false;
    return true;  
}

bool guardar_mediciones_VEML6030(){
    if(!abrir_archivo(VEML_handler.file, VEML_handler.filename)) return false;
    if (f_printf(VEML_handler.file, "%ld,%ld,%lld\n",
                VEML_handler.current.ambient,
                VEML_handler.current.white, 
                VEML_handler.current.time_ms/1000) < 0) {
        printf("f_printf failed\n");
        blink_led(6, 200); // Error de escritura
        return false;
    }
    if(!cerrar_archivo(VEML_handler.file)) return false;
    return true;  
}

bool guardar_mediciones_AM2302(){
    if(!abrir_archivo(AM23_handler.file, AM23_handler.filename)) return false;
    if (f_printf(AM23_handler.file, "%.1f,%.1f, %d ,%lld\n",
                 AM23_handler.current.humidity,
                 AM23_handler.current.temperature,
                 AM23_handler.buffer->st, 
                 AM23_handler.current.time_ms/1000) < 0) {
        printf("f_printf failed\n");
        blink_led(6, 200); // Error de escritura
        return false;
    }
    if(!cerrar_archivo(AM23_handler.file)) return false;
    return true;  
}

bool guardar_mediciones_ISL29125(){
    if(!abrir_archivo(ISL_handler.file, ISL_handler.filename)) return false;
    if(f_printf(ISL_handler.file,"%2X,%2X,%2X,%lld\n", 
                ISL_handler.current.red, 
                ISL_handler.current.green, 
                ISL_handler.current.blue, 
                ISL_handler.current.time_ms/1000)<0){

        printf("f_printf failed\n");
        blink_led(6, 200); // Error de escritura
        return false;        
    }
    if(!cerrar_archivo(ISL_handler.file)) return false;
    return true;  
}
