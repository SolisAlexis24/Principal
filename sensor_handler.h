#pragma once
#include "ff.h"
#include "f_util.h"
#include "LSM9DS1.h"
#include "MLX90393.h"

#define BUFFER_SIZE 32
#define LED_PIN 25  // LED integrado de la Pico

// Estructura que contiene los elementos que un sensor usa para funcionar
typedef struct {
    bool is_connected;
    const char* filename;
    FIL* file;
    uint8_t buffer_head;
    uint8_t buffer_tail;
    bool buffer_full;
    // Add a union for the buffer
    union {
        LSM9DS1::LSM9DS1Data lsm_buffer[BUFFER_SIZE];
        MLX90393::MLX90393Data mlx_buffer[BUFFER_SIZE];
    };
    union {
        LSM9DS1::LSM9DS1Data lsm_current;
        MLX90393::MLX90393Data mlx_current;
    };
    // Estas variables se deben de modificar manualmente por la falta del pin de interrupcion
    bool is_mag_data_ready; // Variable que indica que la medicion del magnetrometro esta lista
    bool is_temp_data_ready; // Variable que indica que la medicion del termometro esta lista
} SensorHandler;


bool is_connected(SensorHandler* handler);

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
 */
bool guardar_mediciones_LSM9DS1(FIL* fil, const char* filename ,float accel[3], float gyro[3], float mag[3], uint64_t time);

bool guardar_mediciones_MLX90393(FIL* fil, const char* filename, float x, float y, float z, uint64_t time);