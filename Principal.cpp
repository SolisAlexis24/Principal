#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hw_config.h"
#include "f_util.h"
#include "ff.h"
#include "LSM9DS1.h"

#define LED_PIN 25  // LED integrado de la Pico

/**
 * @brief Hace parpadear el LED un número específico de veces
 * @param count Número de parpadeos
 * @param delay_ms Tiempo entre parpadeos en milisegundos
 */
void blink_led(uint8_t count, uint16_t delay_ms);

/**
 * @brief Intenta abrir un acrchivo
 * @param fil Puntero al archivo sobre el que se trabaja
 * @param filename Nombre del archivo sobre el que se trabaja
 */
bool abrir_archivo(FIL* fil, const char* filename);
/**
 * @brief Intenta cerrar un acrchivo
 * @param fil Puntero al archivo sobre el que se trabaja
 */
bool cerrar_archivo(FIL* fil);
/**
 * @brief Intenta escribir los datos del LSM9DS1 en la SD
 * @param fil Puntero al archivo sobre el que se trabaja
 * @param accel Informacion acerca de la aceleracion
 * @param gyro Informacion acerca del giro
 * @param mag Informacion acerca del magnetometro
 */
bool escribir_LSM9DS1(FIL* fil, float accel[3], float gyro[3], float mag[3], uint64_t time);

extern sd_card_t sd_card;

int main() {
    // Inicialización del LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Inicialización USB
    if (!stdio_init_all()) {
        while(1) {
            blink_led(3, 200); // 3 parpadeos para error de USB
            sleep_ms(1000);
        }
    }

    //=============================================Montaje del sistema de archivos=============================================
    FATFS fs;
    FRESULT fr = f_mount(&fs, "", 1);
    // Error de montaje de sistema de archivos
    if (FR_OK != fr) {
        printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        while(1) {
            blink_led(4, 200); // 4 parpadeos para error de montaje
            sleep_ms(1000);
        }
    }
    //=========================================================================================================================
    // Puntero al archivo
    FIL fil;
    // Nombre del archivo
    const char* const filename = "filename.txt";

    //===================================Inicializacion imu===================================
    LSM9DS1 imu;
    // Inicializar sensores con parámetros personalizados
    imu.init_accel(LSM9DS1::SCALE_GYRO_500DPS, LSM9DS1::SCALE_ACCEL_4G, 
                  LSM9DS1::ODR_119HZ, LSM9DS1::ODR_119HZ);
    imu.init_magnetometer(LSM9DS1::MAG_SCALE_4GAUSS, LSM9DS1::MAG_ODR_80HZ);

    printf("Sensor LSM9DS1 inicializado correctamente.\n");
    
    // Matrices de datos
    float accel[3], gyro[3], mag[3];
    //==========================================================================================
    // Variables para medicion de tiempo
    absolute_time_t start_time = get_absolute_time();
    absolute_time_t now;
    int64_t elapsed_ms;
    
    // TODO: Pasar esto a una interrupcion
    while(1) {
        // TODO: Reintentar el montaje de la SD cada X tiempo por un par de intentos
        if (sd_card.sd_test_com && !sd_card.sd_test_com(&sd_card)) {
            printf("La tarjeta SD fue retirada, desmontando...\n");
            // Desmonta el sistema de archivos
            f_unmount("");
            // Puedes detener el sistema, esperar nueva inserción, etc.
            while (1) {
                blink_led(8, 500); // Indica tarjeta retirada
                sleep_ms(1000);
            }
        }
        now = get_absolute_time();
        elapsed_ms = absolute_time_diff_us(start_time, now) / 1000;
        imu.read_accelerometer(accel);
        imu.read_gyroscope(gyro);
        imu.read_magnetometer(mag);
        if (!abrir_archivo(&fil, filename)) {
            while (1); // Error manejado dentro de la función
        }
        if(!escribir_LSM9DS1(&fil, accel, gyro, mag, elapsed_ms)){
            while (1); // Error manejado dentro de la función
        }
        if (!cerrar_archivo(&fil)) {
            while (1); // Error manejado dentro de la función
        }
        // printf("[%.2f,%.2f,%.2f][%.2f,%.2f,%.2f][%.2f,%.2f,%.2f] @  %lld ms\n",
        // accel[0], accel[1], accel[2],
        // gyro[0], gyro[1], gyro[2],
        // mag[0], mag[1], mag[2], elapsed_ms);
        sleep_ms(10);
    }
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

bool escribir_LSM9DS1(FIL* fil, float accel[3], float gyro[3], float mag[3], uint64_t time) {
    if (f_printf(fil, "[%.2f,%.2f,%.2f][%.2f,%.2f,%.2f][%.2f,%.2f,%.2f] @  %lld ms\n",
        accel[0], accel[1], accel[2],
        gyro[0], gyro[1], gyro[2],
        mag[0], mag[1], mag[2], time) < 0) {
        printf("f_printf failed\n");
        blink_led(6, 200); // Error de escritura
        return false;
    }
    return true;
}