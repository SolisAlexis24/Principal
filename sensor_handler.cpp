#include "sensor_handler.h"
// Variables globales para los sensores
SensorHandler LSM_handler;
SensorHandler MLX_mag_handler;
SensorHandler MLX_temp_handler;
SensorHandler MS5803_handler;
SensorHandler VEML_handler;
SensorHandler AM23_handler;
SensorHandler ISL_handler;

bool is_connected(SensorHandler* handler)
{
    return handler->is_connected;
}

bool buffer_has_elements(SensorHandler* handler)
{
    return handler->buffer_tail != handler->buffer_head || handler->buffer_full;
}

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
                 LSM_handler.lsm_current.accel[0], LSM_handler.lsm_current.accel[1], LSM_handler.lsm_current.accel[2],
                 LSM_handler.lsm_current.gyro[0], LSM_handler.lsm_current.gyro[1], LSM_handler.lsm_current.gyro[2],
                 LSM_handler.lsm_current.mag[0], LSM_handler.lsm_current.mag[1], LSM_handler.lsm_current.mag[2], 
                 LSM_handler.lsm_current.time_ms) < 0) {
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
                 MLX_mag_handler.mlx_current.x, 
                 MLX_mag_handler.mlx_current.y, 
                 MLX_mag_handler.mlx_current.z, 
                 MLX_mag_handler.mlx_current.time_ms) < 0) {
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
                 MLX_temp_handler.mlx_current.t, 
                 MLX_temp_handler.mlx_current.time_ms/1000) < 0) {
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
                 MS5803_handler.ms5803_current.temperature,
                 MS5803_handler.ms5803_current.pressure, 
                 MS5803_handler.ms5803_current.time_ms/1000) < 0) {
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
                VEML_handler.veml_current.ambient,
                VEML_handler.veml_current.white, 
                VEML_handler.veml_current.time_ms/1000) < 0) {
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
                 AM23_handler.am23_current.humidity,
                 AM23_handler.am23_current.temperature,
                 AM23_handler.am23_buffer->st, 
                 AM23_handler.am23_current.time_ms/1000) < 0) {
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
                ISL_handler.isl_current.red, 
                ISL_handler.isl_current.green, 
                ISL_handler.isl_current.blue, 
                ISL_handler.isl_current.time_ms/1000)<0){

        printf("f_printf failed\n");
        blink_led(6, 200); // Error de escritura
        return false;        
    }
    if(!cerrar_archivo(ISL_handler.file)) return false;
    return true;  
}
