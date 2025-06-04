#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hw_config.h"
#include "hardware/i2c.h"
#include "f_util.h"
#include "ff.h"
#include "LSM9DS1.h"
#include "MLX90393.h"

#define LED_PIN 25  // LED integrado de la Pico
#define BUFFER_SIZE 32
#define SDA_PIN 16
#define SCL_PIN 17
#define I2C_FREC 400000

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

bool guardar_mediciones_MLX90393(FIL* fil, float x, float y, float z, uint64_t time);

/**
 * @brief Interrupcion que se lanza cada 10 ms para leer sensores
 */
bool capturar10ms(__unused struct repeating_timer *t);


i2c_inst_t* i2c_port = i2c0;

// Tarjeta SD
extern sd_card_t sd_card;

// Matrices de datos para almacenar informacion
float accel[3], gyro[3], mag[3];
// Variables para medir el tiempo del sensor
absolute_time_t start_time;  // Tiempo de inicio de mediciones
absolute_time_t now;         // Tiempo actual
int64_t elapsed_ms;          // Tiempo actual menos tiempo inicial

LSM9DS1 imu(i2c_port,SDA_PIN, SCL_PIN, I2C_FREC);                 // Sensor de mediciones
MLX90393 magnt(i2c_port,SDA_PIN, SCL_PIN, I2C_FREC);
/*
Este buffer se usa con el proposito de guardar mediciones
al hacer lecturas para despues escribirlas en el archivo
*/ 
LSM9DS1Data lecturaLSM[BUFFER_SIZE];   // Buffer para los datos de la IMU
MLX90393::MLX90393Data lecturasMLX[BUFFER_SIZE]; // Buffer de datos magnetometro


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
    sleep_ms(5000);
    // Inicializar hardware I2C
    i2c_init(i2c_port, 400000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

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
    FIL file1, file2;
    // Nombre del archivo
    const char* const filename1 = "LSM9DS1.txt";
    const char* const filename2 = "MLX90393.txt";

    //===================================Inicializacion imu===================================
    // Inicializar sensores con parámetros personalizados
    imu.init_accel(LSM9DS1::SCALE_GYRO_500DPS, LSM9DS1::SCALE_ACCEL_4G, 
                  LSM9DS1::ODR_119HZ, LSM9DS1::ODR_119HZ);
    imu.init_magnetometer(LSM9DS1::MAG_SCALE_4GAUSS, LSM9DS1::MAG_ODR_80HZ);
    imu.calibrate_magnetometer(0.15f, 0.08f, -0.47f);
    imu.calibrate_gyro(-0.2673f, 0.5627f, 0.8419);
    printf("Sensor LSM9DS1 conectado y funcionando correctamente\n");

    magnt.init_sensor(
        0x8000, 0x8000, 0x8000,      // Offsets X,Y,Z
        MLX90393::RESOLUTION_MAX,   // Resolución X
        MLX90393::RESOLUTION_MAX,   // Resolución Y
        MLX90393::RESOLUTION_MAX,   // Resolución Z
        MLX90393::FILT_1,           // Filtro digital
        MLX90393::OSR_MAX         // Oversampling
    );
    
    // Interrupcion para leer los sensores cada 10 ms
    struct repeating_timer timer;
    add_repeating_timer_ms(10, capturar10ms, NULL, &timer);

    // Variable auxiliar para guardar los datos a escribir desde el buffer
    LSM9DS1Data current_data_lsm;
    MLX90393::MLX90393Data current_data_mlx;
    
    // Empezamos a medir el tiempo
    start_time = get_absolute_time();

    while(1) {
        // // TODO: Reintentar el montaje de la SD cada X tiempo por un par de intentos
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

        // Se verifica si el buffer tiene informacion por leer
        if (imu.buffer_tail != imu.buffer_head || imu.buffer_full) {
            // Extrae datos del buffer (con interrupciones desactivadas)
            uint32_t save = save_and_disable_interrupts();
            current_data_lsm = lecturaLSM[imu.buffer_tail];
            imu.buffer_tail = (imu.buffer_tail + 1) % BUFFER_SIZE;
            imu.buffer_full = false;
            restore_interrupts(save);

            // Escribe en SD (fuera de la interrupción)
            if (!abrir_archivo(&file1, filename1)) {
                while (1);  // Manejo de error
            }
            if (!escribir_LSM9DS1(&file1, current_data_lsm.accel, current_data_lsm.gyro, 
                                 current_data_lsm.mag, current_data_lsm.elapsed_ms)) {
                while (1);  // Manejo de error
            }
            if (!cerrar_archivo(&file1)) {
                while (1);  // Manejo de error
            }
            // printf("[%.2f,%.2f,%.2f][%.2f,%.2f,%.2f][%.2f,%.2f,%.2f] @  %lld ms\n",
            // current_data.accel[0], current_data.accel[1], current_data.accel[2],
            // current_data.gyro[0], current_data.gyro[1], current_data.gyro[2],
            // current_data.mag[0], current_data.mag[1], current_data.mag[2], current_data.elapsed_ms);
        }

        // Se verifica si el buffer tiene informacion por leer
        if (magnt.buffer_tail != magnt.buffer_head || magnt.buffer_full) {
            // Extrae datos del buffer (con interrupciones desactivadas)
            uint32_t save = save_and_disable_interrupts();
            current_data_mlx = lecturasMLX[magnt.buffer_tail];
            magnt.buffer_tail = (magnt.buffer_tail + 1) % BUFFER_SIZE;
            magnt.buffer_full = false;
            restore_interrupts(save);

            // Escribe en SD (fuera de la interrupción)
            if (!abrir_archivo(&file2, filename2)) {
                while (1);  // Manejo de error
            }
            if (!guardar_mediciones_MLX90393(&file2, current_data_mlx.x, current_data_mlx.y, 
                current_data_mlx.z, current_data_mlx.time_ms)) {
                while (1);  // Manejo de error
            }
            if (!cerrar_archivo(&file2)) {
                while (1);  // Manejo de error
            }
        }      
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

bool guardar_mediciones_MLX90393(FIL* fil, float x, float y, float z, uint64_t time){
    if (f_printf(fil, "[%.2f,%.2f,%.2f] @  %lld ms\n",
        x, y, z, time) < 0) {
        printf("f_printf failed\n");
        blink_led(6, 200); // Error de escritura
        return false;
    }
    return true;   
}


bool capturar10ms(__unused repeating_timer *t)
{
    // Se calcula el tiempo que ha pasado
    now = get_absolute_time();
    elapsed_ms = absolute_time_diff_us(start_time, now) / 1000;
    // Se leen los datos del sensor
    imu.read_accelerometer(accel);
    imu.read_gyroscope(gyro);
    imu.read_magnetometer(mag);
    
    // Se guardan los datos del sensor
    uint32_t save = save_and_disable_interrupts();
    lecturaLSM[imu.buffer_head] = (LSM9DS1Data){ .accel = {accel[0], accel[1], accel[2]},
                                             .gyro = {gyro[0], gyro[1], gyro[2]},
                                             .mag = {mag[0], mag[1], mag[2]},
                                             .elapsed_ms = elapsed_ms };
    imu.buffer_head = (imu.buffer_head + 1) % BUFFER_SIZE;
    imu.buffer_full = (imu.buffer_head == imu.buffer_tail);

    // 1. Comienza la medicion
    magnt.begin_measurement_mag(elapsed_ms);

    // 2. Leer e imprimir la medición anterior (excepto en la primera iteración)
    if (!magnt.is_mag_measurement_ready) {
        lecturasMLX[magnt.buffer_head] = magnt.read_measurement_mag();  // Lee la medición iniciada hace 10 ms
        magnt.buffer_head = (magnt.buffer_head + 1) % BUFFER_SIZE;
        magnt.buffer_full = (magnt.buffer_head == magnt.buffer_tail);
    } else {
        magnt.is_mag_measurement_ready = false;
    }

    restore_interrupts(save);
    return true;
}
