#include "sensor_handler.h"

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

bool guardar_mediciones_LSM9DS1(FIL* fil, const char* filename ,float accel[3], float gyro[3], float mag[3], uint64_t time) {
    if(!abrir_archivo(fil, filename)) return false;
    if (f_printf(fil, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%lld\n",
        accel[0], accel[1], accel[2],
        gyro[0], gyro[1], gyro[2],
        mag[0], mag[1], mag[2], time) < 0) {
        printf("f_printf failed\n");
        blink_led(6, 200); // Error de escritura
        return false;
    }
    if(!cerrar_archivo(fil)) return false;
    return true;
}

bool guardar_mediciones_mag_MLX90393(FIL* fil, const char* filename, float x, float y, float z, uint64_t time){
    if(!abrir_archivo(fil, filename)) return false;
    if (f_printf(fil, "%.2f,%.2f,%.2f,%lld\n",
        x, y, z, time) < 0) {
        printf("f_printf failed\n");
        blink_led(6, 200); // Error de escritura
        return false;
    }
    if(!cerrar_archivo(fil)) return false;
    return true;   
}

bool guardar_mediciones_temp_MLX90393(FIL *fil, const char *filename, float t, uint64_t time)
{
    if(!abrir_archivo(fil, filename)) return false;
    if (f_printf(fil, "%.2f,%lld\n",
        t, time) < 0) {
        printf("f_printf failed\n");
        blink_led(6, 200); // Error de escritura
        return false;
    }
    if(!cerrar_archivo(fil)) return false;
    return true;  
}

bool guardar_mediciones_MS5003(FIL *fil, const char *filename, float t, float p, uint64_t time)
{
    if(!abrir_archivo(fil, filename)) return false;
    if (f_printf(fil, "%.2f,%.2f,%lld\n",
        t,p, time) < 0) {
        printf("f_printf failed\n");
        blink_led(6, 200); // Error de escritura
        return false;
    }
    if(!cerrar_archivo(fil)) return false;
    return true;  
}

bool guardar_mediciones_VEML6030(FIL *fil, const char *filename, uint32_t a, uint32_t w, uint64_t time)
{
    if(!abrir_archivo(fil, filename)) return false;
    if (f_printf(fil, "%ld,%ld,%lld\n",
        a,w, time) < 0) {
        printf("f_printf failed\n");
        blink_led(6, 200); // Error de escritura
        return false;
    }
    if(!cerrar_archivo(fil)) return false;
    return true;  
}

bool guardar_mediciones_AM2302(FIL *fil, const char *filename, float h, float t, AM2302::state st, uint64_t time)
{
    if(!abrir_archivo(fil, filename)) return false;
    if (f_printf(fil, "%.1f,%.1f, %d ,%lld\n",
        h,t,st, time) < 0) {
        printf("f_printf failed\n");
        blink_led(6, 200); // Error de escritura
        return false;
    }
    if(!cerrar_archivo(fil)) return false;
    return true;  
}

bool guardar_mediciones_ISL29125(FIL *fil, const char *filename, uint16_t r, uint16_t g,  uint16_t b, uint64_t time)
{
    if(!abrir_archivo(fil, filename)) return false;
    if (f_printf(fil, "%2X,%2X,%2X,%lld\n",
        r,g,b, time) < 0) {
        printf("f_printf failed\n");
        blink_led(6, 200); // Error de escritura
        return false;
    }
    if(!cerrar_archivo(fil)) return false;
    return true;  
}
