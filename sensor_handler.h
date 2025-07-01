#pragma once
#include "ff.h"
#include "f_util.h"
#include "LSM9DS1.h"
#include "MLX90393.h"
#include "MS5803-14BA.h"
#include "VEML6030.h"
#include "AM2302.h"
#include "ISL29125.h"

#define BUFFER_SIZE 32
#define LED_PIN 25  // LED integrado de la Pico
#define PIN_AM2302 6

// Estructura que contiene los elementos que un sensor usa para funcionar
typedef struct {
    bool is_connected;      // Indica si el sensor esta conectado
    const char* filename;   // Nombre del archivo donde se guardan los datos
    FIL* file;              // Puntero al archivo donde se guardan los datos
    uint8_t buffer_head;    // Indices que indican donde se encuentra el buffer
    uint8_t buffer_tail;  
    bool buffer_full;       // Indica si el buffer esta lleno
    volatile bool fire_measurement; // Indica si se debe de hacer una medicion
    union { // Estructura que contiene el buffer de datos
        LSM9DS1::LSM9DS1Data lsm_buffer[BUFFER_SIZE];
        MLX90393::MLX90393Data mlx_buffer[BUFFER_SIZE];
        MS5803::MS5803Data ms58903_buffer[BUFFER_SIZE];
        VEML6030::VEML6030Data veml_buffer[BUFFER_SIZE];
        AM2302::AM2302Data am23_buffer[BUFFER_SIZE];
        ISL29125::ISL29125Data isl_buffer[BUFFER_SIZE];
        
    };
    union {     // Estructura que contiene los datos de la ultima medicion
        LSM9DS1::LSM9DS1Data lsm_current;
        MLX90393::MLX90393Data mlx_current;
        MS5803::MS5803Data ms5803_current;
        VEML6030::VEML6030Data veml_current;
        AM2302::AM2302Data am23_current;
        ISL29125::ISL29125Data isl_current;
    };
    // Estas variables se deben de modificar manualmente por la falta del pin de interrupcion

    bool flag_1; 
    bool flag_2;

} SensorHandler;

// Variables globales para los sensores
extern SensorHandler LSM_handler;
extern SensorHandler MLX_mag_handler;
extern SensorHandler MLX_temp_handler;
extern SensorHandler MS5803_handler;
extern SensorHandler VEML_handler;
extern SensorHandler AM23_handler;
extern SensorHandler ISL_handler;

/**
 * @brief Verifica si el sensor esta conectado
 * @param handler Puntero al SensorHandler que contiene la informacion del sensor
 */
bool is_connected(SensorHandler* handler);

/**
 * @brief Verifica si el buffer del sensor tiene elementos
 * @param handler Puntero al SensorHandler que contiene la informacion del sensor
 */
bool buffer_has_elements(SensorHandler* handler);

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
 * @param filename Nombre del archivo que se va a escribir
 * @param accel Informacion acerca de la aceleracion
 * @param gyro Informacion acerca del giro
 * @param mag Informacion acerca del magnetometro
 * @param time Tiempo de la medicion
 * @return true si se guardaron los datos correctamente, false en caso contrario
 */
bool guardar_mediciones_LSM9DS1();

/**
 * @brief Intenta escribir los datos del MLX90393_mag en la SD
 * @param fil Puntero al archivo sobre el que se trabaja
 * @param filename Nombre del archivo que se va a escribir
 * @param x Valor de la medicion en X
 * @param y Valor de la medicion en Y
 * @param z Valor de la medicion en Z
 * @param time Tiempo de la medicion
 * @return true si se guardaron los datos correctamente, false en caso contrario
 */
bool guardar_mediciones_mag_MLX90393();

/**
 * @brief Intenta escribir los datos del MLX90393_temp en la SD
 * @param fil Puntero al archivo sobre el que se trabaja
 * @param filename Nombre del archivo que se va a escribir
 * @param x Valor de la medicion en X
 * @param y Valor de la medicion en Y
 * @param z Valor de la medicion en Z
 * @param time Tiempo de la medicion
 * @return true si se guardaron los datos correctamente, false en caso contrario
 */
bool guardar_mediciones_temp_MLX90393();

bool guardar_mediciones_MS5003();

bool guardar_mediciones_VEML6030();

bool guardar_mediciones_AM2302();

bool guardar_mediciones_ISL29125();

template<typename T>
void guardar_en_buffer(SensorHandler* manejador, T* buffer ,const T& medicion){
    buffer[manejador->buffer_head] = medicion;
    manejador->buffer_head = (manejador->buffer_head + 1) % BUFFER_SIZE;
    manejador->buffer_full = (manejador->buffer_head == manejador->buffer_tail);
}

template <typename T>
inline void actualizar_buffer(SensorHandler *handler, T *buffer, T& current){
    uint32_t save = save_and_disable_interrupts();
    current = buffer[handler->buffer_tail];
    handler->buffer_tail = (handler->buffer_tail + 1) % BUFFER_SIZE;
    handler->buffer_full = false;
    restore_interrupts_from_disabled(save);
}
