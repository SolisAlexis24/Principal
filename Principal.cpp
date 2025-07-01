#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hw_config.h"
#include "hardware/i2c.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "f_util.h"
#include "ff.h"
#include "sensor_handler.h"
#include "LSM9DS1.h"
#include "MLX90393.h"
#include "MS5803-14BA.h"
#include "VEML6030.h"
#include "AM2302.h"
#include "ISL29125.h"


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

/**
 * @brief Función que se ejecuta cuando se recibe una alarma para leer el sensor MS5803.
 *        Dependiendo del valor de flag_3, se decide si se lee la temperatura o la presión.
 */
int64_t get_ms5803(alarm_id_t id, __unused void *userdata);

i2c_inst_t* i2c_port = i2c0;

// Tarjeta SD
extern sd_card_t sd_card;

// Variables para medir el tiempo del sensor
absolute_time_t start_time;  // Tiempo de inicio de mediciones
volatile absolute_time_t now;         // Tiempo actual
volatile int64_t elapsed_ms;          // Tiempo actual menos tiempo inicial
volatile int64_t safe_elapsed_ms_c0;    
volatile int64_t safe_elapsed_ms_c1;

mutex_t time_mutex; // Mutex para proteger el acceso a las variables de tiempo
mutex_t i2c_mutex; // Mutex para proteger el acceso al bus I2C
mutex_t spi_mutex; // Mutex para proteger el acceso al bus SPI

// Inicializacion de comunicacion con los sensores
LSM9DS1 lsm(i2c_port,SDA_PIN, SCL_PIN, I2C_FREC, &i2c_mutex);
MLX90393 mlx(i2c_port,SDA_PIN, SCL_PIN, I2C_FREC, &i2c_mutex);
MS5803 ms5803(i2c_port, SDA_PIN, SCL_PIN, I2C_FREC, &i2c_mutex);
VEML6030 veml(i2c_port, SDA_PIN, SCL_PIN, I2C_FREC, &i2c_mutex);
AM2302 am23(PIN_AM2302);
ISL29125 isl(i2c_port, SDA_PIN, SCL_PIN, I2C_FREC, &i2c_mutex);


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
    FIL C0_file, C1_file;

    LSM_handler = (SensorHandler){
        .is_connected = false,
        .filename = "LSM9DS1.csv",
        .file = &C0_file,
        .buffer_head = 0,
        .buffer_tail = 0,
        .buffer_full = false
    };
    MLX_mag_handler = (SensorHandler){
        .is_connected = false,
        .filename = "MLX90393_mag.csv",
        .file = &C0_file,
        .buffer_head = 0,
        .buffer_tail = 0,
        .buffer_full = false,
        .flag_1 = true,                     // Inidca si es la primera vez que se inicia lectura de magnetometro
    };
    MLX_temp_handler = (SensorHandler){
        .is_connected = false,
        .filename = "MLX90393_temp.csv",
        .file = &C1_file,
        .buffer_head = 0,
        .buffer_tail = 0,
        .buffer_full = false,
    };
    MS5803_handler = (SensorHandler){
        .is_connected = false,
        .filename = "MS5803.csv",
        .file = &C1_file,
        .buffer_head = 0,
        .buffer_tail = 0,
        .buffer_full = false,
        .flag_1 = true,                 // Indica que es la primera vez que se inicia lectura de temperatura
                                        // La flag de presion no es necearia, pues esta se mide despues de la temperatura
        .flag_2 = false                  //Indica que variable se mando a medir en el ultimo comando
                                        // Si es 0, se mide temperatura, si es 1, se mide presion
    };
    VEML_handler = (SensorHandler){
        .is_connected = false,
        .filename = "VEML6030.csv",
        .file = &C1_file,
        .buffer_head = 0,
        .buffer_tail = 0,
        .buffer_full = false,
    };

    AM23_handler = (SensorHandler){
        .is_connected = false,
        .filename = "AM2302.csv",
        .file = &C1_file,
        .buffer_head = 0,
        .buffer_tail = 0,
        .buffer_full = false,
        .flag_1 = true
    };

    ISL_handler = (SensorHandler){
        .is_connected = false,
        .filename = "ISL20125.csv",
        .file = &C1_file,
        .buffer_head = 0,
        .buffer_tail = 0,
        .buffer_full = false
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
        MLX90393::RESOLUTION_MAX,   // Resolucion X
        MLX90393::RESOLUTION_MAX,   // Resolucion Y
        MLX90393::RESOLUTION_MAX,   // Resolucion Z
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
    
    if(ms5803.init_sensor(
        MS5803::T_OSR_4096,
        MS5803::P_OSR_4096 ,
        MS5803::ADDRESS_HIGH
    )){
        if(abrir_archivo(MS5803_handler.file, MS5803_handler.filename)){
            if (f_printf(MS5803_handler.file, "T[C],P[mbar],t[s]\n") < 0) {
                printf("f_printf failed\n");
                blink_led(6, 200); // Error de escritura
                return false;
            }       
        }
        if (!cerrar_archivo(MS5803_handler.file)) {
            while (1);  // Manejo de error
        }
        MS5803_handler.is_connected = true;
    }
    else{
        MS5803_handler.is_connected = false;
    }

    if(veml.init_sensor(VEML6030::HIGH, VEML6030::GAIN_0_125, VEML6030::IT_100MS)){
        if(abrir_archivo(VEML_handler.file, VEML_handler.filename)){
            if (f_printf(VEML_handler.file, "Ambiental[lux],Blanca[lux],t[s]\n") < 0) {
                printf("f_printf failed\n");
                blink_led(6, 200); // Error de escritura
            }       
        }
        if (!cerrar_archivo(VEML_handler.file)) {
            while (1);  // Manejo de error
        }
        VEML_handler.is_connected = true;
    }
    else{
        VEML_handler.is_connected = false;
    }

    if(am23.init_sensor()){
        if(abrir_archivo(AM23_handler.file, AM23_handler.filename)){
            if (f_printf(AM23_handler.file, "Humedad relativa [%%], Temperatura [C], Status ,t[s]\n") < 0) {
                printf("f_printf failed\n");
                blink_led(6, 200); // Error de escritura
            }       
        }
        if (!cerrar_archivo(AM23_handler.file)) {
            while (1);  // Manejo de error
        }    
        AM23_handler.is_connected = true;
    }else{
        AM23_handler.is_connected = false;
    }

    if(isl.init_sensor()){
        if(abrir_archivo(ISL_handler.file, ISL_handler.filename)){
            if (f_printf(ISL_handler.file, "Rojo [lux],Verde [lux],Azul [lux],t[s]\n") < 0) {
                printf("f_printf failed\n");
                blink_led(6, 200); // Error de escritura
            }       
        }
        if (!cerrar_archivo(ISL_handler.file)) {
            while (1);  // Manejo de error
        }     
        ISL_handler.is_connected = true;
    }
    else{
        ISL_handler.is_connected = false;        
    }

    // Interrupcion para leer los sensores cada 10 ms
    struct repeating_timer timer_c_0;
    add_repeating_timer_ms(10, capturar10ms, NULL, &timer_c_0);
    
    // Empezamos a medir el tiempo
    start_time = get_absolute_time();

    multicore_launch_core1(core1_main);

    while(1) {

        // Se verifica si el buffer tiene informacion por leer
        if(is_connected(&LSM_handler) && buffer_has_elements(&LSM_handler)){
            actualizar_buffer(&LSM_handler, LSM_handler.lsm_buffer, LSM_handler.lsm_current);
            mutex_enter_blocking(&spi_mutex);
            if (!guardar_mediciones_LSM9DS1()) {
                printf("Error al guardar mediciones LSM9DS1\n");
            }
            mutex_exit(&spi_mutex);
        }

        if(is_connected(&MLX_mag_handler) && buffer_has_elements(&MLX_mag_handler)){
            actualizar_buffer(&MLX_mag_handler, MLX_mag_handler.mlx_buffer, MLX_mag_handler.mlx_current);
            mutex_enter_blocking(&spi_mutex);
            if (!guardar_mediciones_mag_MLX90393()) {
                printf("Error al guardar mediciones de magnetometro MLX90393\n");
            }
            mutex_exit(&spi_mutex);
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
        lsm.read_accelerometer();
        lsm.read_gyroscope();
        lsm.read_magnetometer();
        mutex_enter_blocking(&time_mutex);
        lsm.last_measurement.time_ms = safe_elapsed_ms_c0;
        mutex_exit(&time_mutex);
        guardar_en_buffer(&LSM_handler, LSM_handler.lsm_buffer, lsm.last_measurement);  
    }
    //==================================================================================

    //==================================Leer MLX90393 (magnetometro y termometro solo leer)==================================
    if(is_connected(&MLX_mag_handler)){
        if (MLX_mag_handler.flag_1) {                   // Indica que es la primera vez que se mide el magnetometro
            // 1. Comenzar una nueva medición del magnetometro
            mlx.begin_measurement_mt();
            mlx.last_measurement.time_ms = safe_elapsed_ms_c0;
            MLX_mag_handler.flag_1 = false;
        } else {
            // 2. Leer y guardar los datos en el buffer
            mlx.read_measurement_mt();
            guardar_en_buffer(&MLX_mag_handler, MLX_mag_handler.mlx_buffer, mlx.last_measurement);
            // 3. Comenzar una nueva medición para el siguiente ciclo
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
       if(is_connected(&MLX_temp_handler) && buffer_has_elements(&MLX_temp_handler)){
            actualizar_buffer(&MLX_temp_handler, MLX_temp_handler.mlx_buffer, MLX_temp_handler.mlx_current);
            mutex_enter_blocking(&spi_mutex);
            if (!guardar_mediciones_temp_MLX90393()) {
                printf("Error al guardar mediciones de temperatura MLX90393\n");
            }
            mutex_exit(&spi_mutex);
        } 
        if(is_connected(&MS5803_handler) && buffer_has_elements(&MS5803_handler)){
            actualizar_buffer(&MS5803_handler, MS5803_handler.ms58903_buffer, MS5803_handler.ms5803_current);
            mutex_enter_blocking(&spi_mutex);
            if(!guardar_mediciones_MS5003()){
                printf("Error al guardar mediciones de temperatura MS5803\n");
            }
            mutex_exit(&spi_mutex);
        }
        if(is_connected(&VEML_handler) && buffer_has_elements(&VEML_handler)){
            actualizar_buffer(&VEML_handler, VEML_handler.veml_buffer, VEML_handler.veml_current);
            mutex_enter_blocking(&spi_mutex);
            if(!guardar_mediciones_VEML6030()){
                printf("Error al guardar mediciones luminicas\n");
            }
            mutex_exit(&spi_mutex);                
        }

        if(is_connected(&AM23_handler) && buffer_has_elements(&AM23_handler)){
            actualizar_buffer(&AM23_handler, AM23_handler.am23_buffer, AM23_handler.am23_current);
            mutex_enter_blocking(&spi_mutex);
            if(!guardar_mediciones_AM2302()){
                printf("Error al guardar mediciones AM2302\n");
            }     
            mutex_exit(&spi_mutex);        
        }

        if(is_connected(&ISL_handler) && buffer_has_elements(&ISL_handler)){
            actualizar_buffer(&ISL_handler, ISL_handler.isl_buffer, ISL_handler.isl_current);
            mutex_enter_blocking(&spi_mutex);
            if(!guardar_mediciones_ISL29125()){
                printf("Error al guardar mediciones ISL29125\n");
            }     
            mutex_exit(&spi_mutex); 
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
        guardar_en_buffer(&MLX_temp_handler, MLX_temp_handler.mlx_buffer, mlx.last_measurement);
    }
    //========================================================================================================

    if(is_connected(&MS5803_handler)){
        if(MS5803_handler.flag_1){                                                      // Es la primera vez que se mide cualquiera de las dos
            ms5803.start_measurement_temp();
            MS5803_handler.flag_2 = false;                                              // Se indica que se esta midiendo la temperatura
            ms5803.last_measurement.time_ms = safe_elapsed_ms_c1;
            MS5803_handler.flag_1 = false;                                              // Ya se ha medido temperatura una vez
            add_alarm_in_ms(ms5803.aquisition_time, get_ms5803, NULL, false);         // Se lanza la alarma para guardar medicion de temperatura
        }
        else{ 
            ms5803.start_measurement_temp();                                        // Se esta midiendo temperatura
            MS5803_handler.flag_2 = false;                                          // Se indica que se esta midiendo temperatura
            ms5803.last_measurement.time_ms = safe_elapsed_ms_c1;    
            add_alarm_in_ms(ms5803.aquisition_time, get_ms5803, NULL, false);      // Se lanza la alarma para guardar medicion de temperatura
        }
    }

    if(is_connected(&VEML_handler)){
        veml.read_ambient();
        veml.read_white();
        veml.last_measurement.time_ms = safe_elapsed_ms_c1;
        guardar_en_buffer(&VEML_handler, VEML_handler.veml_buffer, veml.last_measurement);
    }

    if(is_connected(&AM23_handler)){
        if(AM23_handler.flag_1){
            am23.start_measurement();
            AM23_handler.flag_1 = false;
            am23.last_measurement.time_ms = safe_elapsed_ms_c1;
        }else{
            am23.read_measurement();
            guardar_en_buffer(&AM23_handler, AM23_handler.am23_buffer, am23.last_measurement);
            am23.start_measurement();
            am23.last_measurement.time_ms = safe_elapsed_ms_c1;
        }
    }

        if(is_connected(&ISL_handler)){
            isl.read_data();
            isl.last_measurement.time_ms = safe_elapsed_ms_c1;
            guardar_en_buffer(&ISL_handler,ISL_handler.isl_buffer,isl.last_measurement);
        }

    return true;
}

int64_t get_ms5803(alarm_id_t id, __unused void *userdata){
    if(MS5803_handler.flag_2){            // En este bloque se guarda la medicion de la presion
        ms5803.read_measurement_press();    // Se lee la medicion de la presion          
        MS5803_handler.flag_2 = false;    // Para la siguiente ocasion se medira temperatura
        guardar_en_buffer(&MS5803_handler, MS5803_handler.ms58903_buffer, ms5803.last_measurement);
        return 0;        // Desactiva la alarma
    }
    else{                                   // En este bloque se guarda la medicion de la temperatura
        ms5803.read_measurement_temp();     // Se lee la medicion de la temperatura

        ms5803.start_measurement_press();   // Se comienza la medicion de la presion
        MS5803_handler.flag_2 = true;     // Se esta midiendo presion
        return (ms5803.aquisition_time) * 1000;     // La alarma se activara en 10 ms de nuevo para guardar presion
    }
}