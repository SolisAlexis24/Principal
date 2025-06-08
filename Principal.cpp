#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hw_config.h"
#include "hardware/i2c.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "f_util.h"
#include "ff.h"
#include "sensor_handler.h"
#include "LSM9DS1.h"
#include "MLX90393.h"

#define SDA_PIN 16
#define SCL_PIN 17
#define I2C_FREC 400000

void core1_main();

/**
 * @brief Interrupcion que se lanza cada 10 ms para leer sensores
 */
bool capturar10ms(__unused struct repeating_timer *t);

/**
 * @brief Interrupcion que se lanza cada 10 s para leer sensores
 */
bool capturar10s(__unused struct repeating_timer *t);

i2c_inst_t* i2c_port = i2c0;

// Tarjeta SD
extern sd_card_t sd_card;

// Variables para medir el tiempo del sensor
absolute_time_t start_time;  // Tiempo de inicio de mediciones
volatile absolute_time_t now;         // Tiempo actual
volatile int64_t elapsed_ms;          // Tiempo actual menos tiempo inicial
volatile int64_t safe_elapsed_ms_c0;    
volatile int64_t safe_elapsed_ms_c1;

// Inicializacion de comunicacion con los sensores
LSM9DS1 lsm(i2c_port,SDA_PIN, SCL_PIN, I2C_FREC);
MLX90393 mlx(i2c_port,SDA_PIN, SCL_PIN, I2C_FREC);

// Variables globales para los sensores
SensorHandler LSM_handler;
SensorHandler MLX_mag_handler;
SensorHandler MLX_temp_handler;

mutex_t time_mutex; // Mutex para proteger el acceso a las variables de tiempo
mutex_t i2c_mutex; // Mutex para proteger el acceso al bus I2C
mutex_t spi_mutex; // Mutex para proteger el acceso al bus SPI

int main() {
    mutex_init(&time_mutex);
    mutex_init(&i2c_mutex);
    mutex_init(&spi_mutex);
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
    FIL LSM_file, MLX_mag_file, MLX_temp_file;

    LSM_handler = (SensorHandler){
        .is_connected = false,
        .filename = "LSM9DS1.csv",
        .file = &LSM_file,
        .buffer_head = 0,
        .buffer_tail = 0,
        .buffer_full = false
    };
    MLX_mag_handler = (SensorHandler){
        .is_connected = false,
        .filename = "MLX90393_mag.csv",
        .file = &MLX_mag_file,
        .buffer_head = 0,
        .buffer_tail = 0,
        .buffer_full = false,
        .fire_measurement = false, // Inicialmente no ha pasado el tiempo para medir
        .is_mag_first_time = true, // Inicialmente no hay datos de magnetometro listos
    };
    MLX_temp_handler = (SensorHandler){
        .is_connected = false,
        .filename = "MLX90393_temp.csv",
        .file = &MLX_temp_file,
        .buffer_head = 0,
        .buffer_tail = 0,
        .buffer_full = false,
        .fire_measurement = false, // Inicialmente no ha pasado el tiempo para medir
        .is_temp_first_time = true, // Inicialmente no hay datos de temperatura listos
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

    //=============================================Inicializacion magnetometro y termometro=============================================
    if(mlx.init_sensor(
        0x8000, 0x8000, 0x8000,      // Offsets X,Y,Z
        MLX90393::RESOLUTION_MAX,   // Resolución X
        MLX90393::RESOLUTION_MAX,   // Resolución Y
        MLX90393::RESOLUTION_MAX,   // Resolución Z
        MLX90393::FILT_1,           // Filtro digital
        MLX90393::OSR_MAX         // Oversampling
    )){
        if(abrir_archivo(MLX_mag_handler.file, MLX_mag_handler.filename)){
            if (f_printf(MLX_mag_handler.file, "B(x)[uT],B(y)[uT],B(z)[uT],t[ms]\n") < 0) {
                printf("f_printf failed\n");
                blink_led(6, 200); // Error de escritura
                return false;
            }       
        }
        if (!cerrar_archivo(MLX_mag_handler.file)) {
            while (1);  // Manejo de error
        }

        if(abrir_archivo(MLX_temp_handler.file, MLX_temp_handler.filename)){
            if (f_printf(MLX_temp_handler.file, "T [C],t[s]\n") < 0) {
                printf("f_printf failed\n");
                blink_led(6, 200); // Error de escritura
                return false;
            }       
        }
        if (!cerrar_archivo(MLX_temp_handler.file)) {
            while (1);  // Manejo de error
        }
        MLX_mag_handler.is_connected = true;
        MLX_temp_handler.is_connected = true;
    }
    else{
        MLX_mag_handler.is_connected = false;
        MLX_temp_handler.is_connected = false;
    }
    //========================================================================================================================================
    
    // Interrupcion para leer los sensores cada 10 ms
    struct repeating_timer timer_c_0;
    add_repeating_timer_ms(10, capturar10ms, NULL, &timer_c_0);
    
    // Empezamos a medir el tiempo
    start_time = get_absolute_time();

    multicore_launch_core1(core1_main);

    while(1) {

        // Se verifica si el buffer tiene informacion por leer
        if(is_connected(&LSM_handler)){
            if (buffer_has_elements(&LSM_handler)) {
                // Extrae datos del buffer (con interrupciones desactivadas)
                uint32_t save = save_and_disable_interrupts();
                LSM_handler.lsm_current =  LSM_handler.lsm_buffer[LSM_handler.buffer_tail];
                LSM_handler.buffer_tail = (LSM_handler.buffer_tail + 1) % BUFFER_SIZE;
                LSM_handler.buffer_full = false;
                restore_interrupts_from_disabled(save);
                mutex_enter_blocking(&spi_mutex);
                if (!guardar_mediciones_LSM9DS1(LSM_handler.file, LSM_handler.filename, LSM_handler.lsm_current.accel, LSM_handler.lsm_current.gyro, LSM_handler.lsm_current.mag, LSM_handler.lsm_current.time_ms)) {
                    printf("Error al guardar mediciones LSM9DS1\n");
                }
                mutex_exit(&spi_mutex);
            }
        }

        if(is_connected(&MLX_mag_handler)){
            // Se verifica si el buffer tiene informacion por leer
            if (buffer_has_elements(&MLX_mag_handler)) {
                // Extrae datos del buffer (con interrupciones desactivadas)
                uint32_t save = save_and_disable_interrupts();
                MLX_mag_handler.mlx_current = MLX_mag_handler.mlx_buffer[MLX_mag_handler.buffer_tail];
                MLX_mag_handler.buffer_tail = (MLX_mag_handler.buffer_tail + 1) % BUFFER_SIZE;
                MLX_mag_handler.buffer_full = false;
                restore_interrupts_from_disabled(save);
                mutex_enter_blocking(&spi_mutex);
                if (!guardar_mediciones_mag_MLX90393(MLX_mag_handler.file, MLX_mag_handler.filename, MLX_mag_handler.mlx_current.x, MLX_mag_handler.mlx_current.y,  MLX_mag_handler.mlx_current.z, MLX_mag_handler.mlx_current.time_ms)) {
                    printf("Error al guardar mediciones de magnetometro MLX90393\n");
                }
                mutex_exit(&spi_mutex);
            } 
        }
        // // TODO: Reintentar el montaje de la SD cada X tiempo por un par de intentos
        if (sd_card.sd_test_com && !sd_card.sd_test_com(&sd_card)) {
            printf("La tarjeta SD fue retirada, desmontando...\n");
            // Desmonta el sistema de archivos
            f_unmount("");           
             while (1) {
                blink_led(8, 500); // Indica tarjeta retirada
                sleep_ms(1000);
            }
        }     
    }
}



bool capturar10ms(__unused repeating_timer *t)
{
    
    // Se calcula el tiempo que ha pasado desde la anterior interrupcion
    mutex_enter_blocking(&time_mutex);
    now = get_absolute_time();
    elapsed_ms = absolute_time_diff_us(start_time, now) / 1000;
    safe_elapsed_ms_c0 = elapsed_ms;
    mutex_exit(&time_mutex);
    // Se guarda el tiempo transcurrido en una variable segura para evitar conflictos de acceso

    
    //===================================Leer LSM9DS1===================================
    if(is_connected(&LSM_handler)){
        mutex_enter_blocking(&i2c_mutex);
        lsm.read_accelerometer();
        lsm.read_gyroscope();
        lsm.read_magnetometer();
        mutex_exit(&i2c_mutex);
        mutex_enter_blocking(&time_mutex);
        lsm.last_measurement.time_ms = safe_elapsed_ms_c0;
        mutex_exit(&time_mutex);
        guardar_en_buffer(LSM_handler.lsm_buffer, LSM_handler.buffer_head, 
            LSM_handler.buffer_tail, BUFFER_SIZE, LSM_handler.buffer_full, lsm.last_measurement);   
        
    }
    //==================================================================================

    //===================================================================================
    // uint32_t save = save_and_disable_interrupts();
    // LSM_handler.fire_measurement = true; // Indicamos que se debe de hacer una medicion
    // MLX_mag_handler.fire_measurement = true; // Indicamos que se debe de hacer una medicion

    // restore_interrupts(save);
   
    

    //==================================Leer MLX90393 (magnetometro y termometro solo leer)==================================
    if(is_connected(&MLX_mag_handler)){
        if (MLX_mag_handler.is_mag_first_time) {
            // Si no hay datos listos, simplemente comenzamos una nueva medición
            // Esto asegura que siempre estemos listos para la próxima medición
            mutex_enter_blocking(&i2c_mutex);
            mlx.begin_measurement_mt();
            mutex_exit(&i2c_mutex);
            mlx.last_measurement.time_ms = safe_elapsed_ms_c0;
            MLX_mag_handler.is_mag_first_time = false;
        } else {
            // 1. Leer y guardar los datos en el buffer
            mutex_enter_blocking(&i2c_mutex);
            mlx.read_measurement_mt();
            mutex_exit(&i2c_mutex);
            guardar_en_buffer(MLX_mag_handler.mlx_buffer, MLX_mag_handler.buffer_head, 
                MLX_mag_handler.buffer_tail, BUFFER_SIZE, MLX_mag_handler.buffer_full, mlx.last_measurement);
            // 2. Comenzar una nueva medición para el siguiente ciclo
            mlx.begin_measurement_mt();     
            mlx.last_measurement.time_ms = safe_elapsed_ms_c0;
        }
    }
    //======================================================================================================================

    
    return true;
}

void core1_main() {
    printf("core1_main iniciado\n");
    // Interrupcion para leer los sensores cada 10 s
    struct repeating_timer timer_c_1;
    add_repeating_timer_ms(10000, capturar10s, NULL, &timer_c_1);
    while(1){
       if(is_connected(&MLX_temp_handler)){
            // Se verifica si el buffer tiene informacion por leer
            if (buffer_has_elements(&MLX_temp_handler)) {
                // Extrae datos del buffer (con interrupciones desactivadas)
                uint32_t save = save_and_disable_interrupts();
                MLX_temp_handler.mlx_current = MLX_temp_handler.mlx_buffer[MLX_temp_handler.buffer_tail];
                MLX_temp_handler.buffer_tail = (MLX_temp_handler.buffer_tail + 1) % BUFFER_SIZE;
                MLX_temp_handler.buffer_full = false;
                restore_interrupts(save);
                mutex_enter_blocking(&spi_mutex);
                if (!guardar_mediciones_temp_MLX90393(MLX_temp_handler.file, MLX_temp_handler.filename, MLX_temp_handler.mlx_current.t, MLX_temp_handler.mlx_current.time_ms/1000)) {
                    printf("Error al guardar mediciones de temperatura MLX90393\n");
                }
                mutex_exit(&spi_mutex);
            } 
        } 
    }

}

bool capturar10s(__unused repeating_timer *t){
    // Se calcula el tiempo que ha pasado desde la anterior interrupcion
    mutex_enter_blocking(&time_mutex);
    now = get_absolute_time();
    elapsed_ms = absolute_time_diff_us(start_time, now) / 1000;
    safe_elapsed_ms_c1 = elapsed_ms;
    mutex_exit(&time_mutex);

    //=====================================Guardar MLX90393 (temperatura)=====================================

    if(is_connected(&MLX_temp_handler)){
        guardar_en_buffer(MLX_temp_handler.mlx_buffer, MLX_temp_handler.buffer_head, 
            MLX_temp_handler.buffer_tail, BUFFER_SIZE, MLX_temp_handler.buffer_full, mlx.last_measurement);
    }
    //========================================================================================================

    return true;
}