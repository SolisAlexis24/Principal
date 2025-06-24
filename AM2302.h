#pragma once
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "AM2302.pio.h"

#define MAX_ATTEMPS 4

class AM2302{
    public:
        enum state{
            OK = 1,
            NOT_OK = 0
        };

        typedef struct {
            float temperature;
            float humidity;
            state st;
            int64_t time_ms;
        }AM2302Data;

        /**
         * @brief Constructor de la clase
         * @param pin Es el pin (GPIO) con el que se va a enviar y recibir informacion con ayuda del sensor
         * @note El valor por defecto asignado es el pin designado para la OBD
         */
        AM2302(uint pin=6);

        /**
         * @brief Este metodo inicializa la maquina de estados dentro de alguno
         * de los slots disponibles, ademas de que realiza una medicion de prueba para verificar
         * la conexion con el sensor.
         */
        bool init_sensor();

        void start_measurement();
    
        /**
         * @brief Esta funcion lee la medicion realizada
         * @return La medicion realizada
         * @note Modifica la variable de medicion interna
         */
        AM2302Data read_measurement();

        AM2302Data last_measurement;

    
    private:
        uint pin_;
        PIO pio_;
        uint sm_;
        uint offset_;
    // Coso de depuracion
    void print_byte_binary(uint8_t byte) {
        for (int i = 7; i >= 0; i--) {
            printf("%d", (byte >> i) & 1);
        }
        printf("\n");
    }
};