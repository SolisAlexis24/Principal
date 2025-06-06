#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hw_config.h"
#include "hardware/i2c.h"
#include "f_util.h"
#include "ff.h"
#include "sensor_handler.h"
#include "LSM9DS1.h"
#include "MLX90393.h"
#define BUFFER_SIZE 32
#define SDA_PIN 16
#define SCL_PIN 17
#define I2C_FREC 400000

/**
 * @brief Interrupcion que se lanza cada 10 ms para leer sensores
 */
bool capturar10ms(__unused struct repeating_timer *t);


i2c_inst_t* i2c_port = i2c0;

// Tarjeta SD
extern sd_card_t sd_card;

// Variables para medir el tiempo del sensor
absolute_time_t start_time;  // Tiempo de inicio de mediciones
absolute_time_t now;         // Tiempo actual
int64_t elapsed_ms;          // Tiempo actual menos tiempo inicial

// Inicializacion de comunicacion con los sensores
LSM9DS1 lsm(i2c_port,SDA_PIN, SCL_PIN, I2C_FREC);
MLX90393 magnt(i2c_port,SDA_PIN, SCL_PIN, I2C_FREC);

// Variables globales para los sensores
SensorHandler LSM_handler;
SensorHandler MLX_handler;


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
    FIL LSM_file, MLX_file;

    LSM_handler = (SensorHandler){
        .is_connected = false,
        .filename = "LSM9DS1.csv",
        .file = &LSM_file,
        .buffer_head = 0,
        .buffer_tail = 0,
        .buffer_full = false
    };
    MLX_handler = (SensorHandler){
        .is_connected = false,
        .filename = "MLX90393.csv",
        .file = &LSM_file,
        .buffer_head = 0,
        .buffer_tail = 0,
        .buffer_full = false,
        .is_mag_data_ready = false, // Inicialmente no hay datos de magnetometro listos
        .is_temp_data_ready = false, // Inicialmente no hay datos de temperatura listos
    };
    //=================================================Inicializacion imu=================================================
    if(lsm.init_accel(
                LSM9DS1::SCALE_GYRO_500DPS, 
                LSM9DS1::SCALE_ACCEL_4G, 
                LSM9DS1::ODR_119HZ, LSM9DS1::ODR_119HZ) && 
                lsm.init_magnetometer(
                    LSM9DS1::MAG_SCALE_4GAUSS, 
                    LSM9DS1::MAG_ODR_80HZ)){
        lsm.calibrate_magnetometer(0.15f, 0.08f, -0.47f);
        lsm.calibrate_gyro(-0.2673f, 0.5627f, 0.8419);
        printf("Sensor LSM9DS1 conectado y funcionando correctamente\n");
        if(abrir_archivo(LSM_handler.file, LSM_handler.filename)){
            if (f_printf(LSM_handler.file, "A(x)[g],A(y)[g],A(z)[g],G(x)[dps],G(y)[dps],G(z)[dps],B(x)[G],B(y)[G],B(z)[G],t[ms]\n") < 0) {
                printf("f_printf failed\n");
                blink_led(6, 200); // Error de escritura
                return false;
            }       
        }
        if (!cerrar_archivo(LSM_handler.file)) {
            while (1);  // Manejo de error
        }
        LSM_handler.is_connected = true;
    }
    else{
        LSM_handler.is_connected = false;
    }
    //====================================================================================================================

    //=============================================Inicializacion magnetometro=============================================
    if(magnt.init_sensor(
        0x8000, 0x8000, 0x8000,      // Offsets X,Y,Z
        MLX90393::RESOLUTION_MAX,   // Resolución X
        MLX90393::RESOLUTION_MAX,   // Resolución Y
        MLX90393::RESOLUTION_MAX,   // Resolución Z
        MLX90393::FILT_1,           // Filtro digital
        MLX90393::OSR_MAX         // Oversampling
    )){
        if(abrir_archivo(MLX_handler.file, MLX_handler.filename)){
            if (f_printf(MLX_handler.file, "B(x)[uT],B(y)[uT],B(z)[uT],t[ms]\n") < 0) {
                printf("f_printf failed\n");
                blink_led(6, 200); // Error de escritura
                return false;
            }       
        }
        if (!cerrar_archivo(MLX_handler.file)) {
            while (1);  // Manejo de error
        }
        MLX_handler.is_connected = true;
    }
    else{
        MLX_handler.is_connected = false;
    }
    //=====================================================================================================================
    
    // Interrupcion para leer los sensores cada 10 ms
    struct repeating_timer timer;
    add_repeating_timer_ms(10, capturar10ms, NULL, &timer);
    
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
        if(is_connected(&LSM_handler)){
            if (buffer_has_elements(&LSM_handler)) {
                // Extrae datos del buffer (con interrupciones desactivadas)
                uint32_t save = save_and_disable_interrupts();
                LSM_handler.lsm_current =  LSM_handler.lsm_buffer[LSM_handler.buffer_tail];
                LSM_handler.buffer_tail = (LSM_handler.buffer_tail + 1) % BUFFER_SIZE;
                LSM_handler.buffer_full = false;
                restore_interrupts(save);
                if (!guardar_mediciones_LSM9DS1(&LSM_file, LSM_handler.filename, LSM_handler.lsm_current.accel, LSM_handler.lsm_current.gyro, LSM_handler.lsm_current.mag, LSM_handler.lsm_current.time_ms)) {
                    printf("Error al guardar mediciones LSM9DS1\n");
                }
            }
        }

        if(is_connected(&MLX_handler)){
            // Se verifica si el buffer tiene informacion por leer
            if (buffer_has_elements(&MLX_handler)) {
                // Extrae datos del buffer (con interrupciones desactivadas)
                uint32_t save = save_and_disable_interrupts();
                MLX_handler.mlx_current = MLX_handler.mlx_buffer[MLX_handler.buffer_tail];
                MLX_handler.buffer_tail = (MLX_handler.buffer_tail + 1) % BUFFER_SIZE;
                MLX_handler.buffer_full = false;
                restore_interrupts(save);
                if (!guardar_mediciones_MLX90393(&MLX_file, MLX_handler.filename, MLX_handler.mlx_current.x, MLX_handler.mlx_current.y,  MLX_handler.mlx_current.z, MLX_handler.mlx_current.time_ms)) {
                    printf("Error al guardar mediciones MLX90393\n");
                }
            } 
        }     
    }
}



bool capturar10ms(__unused repeating_timer *t)
{
    // Se calcula el tiempo que ha pasado desde la anterior interrupcion
    now = get_absolute_time();
    elapsed_ms = absolute_time_diff_us(start_time, now) / 1000;
    //===================================Leer LSM9DS1===================================
    if(is_connected(&LSM_handler)){
        lsm.read_accelerometer();
        lsm.read_gyroscope();
        lsm.read_magnetometer();
        lsm.last_measurement.time_ms = elapsed_ms;
    }
    //==================================================================================
  

    uint32_t save = save_and_disable_interrupts();

    //==================================Guardar LSM9DS1==================================
    if(is_connected(&LSM_handler)){
        LSM_handler.lsm_buffer[LSM_handler.buffer_head] = lsm.last_measurement;
        LSM_handler.buffer_head = (LSM_handler.buffer_head + 1) % BUFFER_SIZE;
        LSM_handler.buffer_full = (LSM_handler.buffer_head == LSM_handler.buffer_tail);
    }
    //===================================================================================

    //==================================Leer y Guardar MLX90393==================================
    if(is_connected(&MLX_handler)){
        if (MLX_handler.is_mag_data_ready) {
            // 1. Leer y guardar los datos en el buffer
            MLX_handler.mlx_buffer[MLX_handler.buffer_head] = magnt.read_measurement_mag();
            MLX_handler.buffer_head = (MLX_handler.buffer_head + 1) % BUFFER_SIZE;
            MLX_handler.buffer_full = (MLX_handler.buffer_head == MLX_handler.buffer_tail);
            MLX_handler.is_mag_data_ready = false;

            // 2. Comenzar una nueva medición para el siguiente ciclo
            magnt.begin_measurement_mag(elapsed_ms);
        } else {
            // Si no hay datos listos, simplemente comenzamos una nueva medición
            // Esto asegura que siempre estemos listos para la próxima medición
            magnt.begin_measurement_mag(elapsed_ms);
            MLX_handler.is_mag_data_ready = true;
        }
    }
    //====================================================================================

    restore_interrupts(save);
    return true;
}
