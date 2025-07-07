#include "AM2302.h"


AM2302::AM2302(uint pin):pin_(pin){}

bool AM2302::init_sensor(){

    // Se instancia la maquina de estados en uno de los slots disponibles
    if(!pio_claim_free_sm_and_add_program_for_gpio_range(&AM2302_program, &pio_, &sm_, &offset_, pin_, 1, true)){
        printf("AM2302: No se pudo instanciar la maquina de estados");
        return false;
    }
    // Se inicializa el comportamiento de la maquina
    AM2302_program_init(pio_, sm_, offset_, pin_);
    pio_sm_clear_fifos(pio_, sm_);

    start_measurement();
    read_measurement(); // La primera lectura siempre es basura
    sleep_ms(2000);
    start_measurement();
    read_measurement();

    // Se verifica si la informacion obtenida es correcta
    // Si lo es, lo mas probable es que el sensor este conectado correctamente
    if(this->last_measurement.st == OK){
        printf("AM2302: Iniciado correctamente\n");
        return true;
    }
    else{
        // Se intenta hacer una lectura exitosa del sensor, si se hace
        // se indica que se pudo conectar correctamente, sino, se indica que no se pudo
        for(int attemp = 1; attemp <= MAX_ATTEMPS; attemp++){
            printf("AM2302: No se pudo iniciar, intentando de nuevo: %u/%u\n", attemp, MAX_ATTEMPS);
            sleep_ms(2000);             // Delay minimo necesario entre lecturas
            start_measurement();
            read_measurement();   // Se hace una nueva lectura
            if(last_measurement.st == OK){
                printf("AM2302: Iniciado correctamente al intento: %u\n", attemp);
                return true;
            }
        }
        printf("AM2302: No se pudo iniciar definitivamente\n");
        return false;
    }   
}

void AM2302::start_measurement(){
    // Se inicia una lectura mediante colocar 7 en la FIFO RX
    // El valor 7 es totalmente arbitrario, pero si se modifica
    // aqui, se debe modificar dentro del programa .pio
    pio_sm_put(pio_, sm_, 7);
}

AM2302::AM2302Data AM2302::read_measurement(){
    // Se obtiene la informacion del sensor
    //pio_sm_put(pio_, sm_, 7);
    int32_t data = pio_sm_get(pio_, sm_);
    uint8_t byte1 = (data >> 24) & 0xFF;    // Primer byte
    uint8_t byte2 = (data >> 16) & 0xFF;    // Segundo byte
    uint8_t byte3 = (data >> 8) & 0xFF;     // Tercer byte
    uint8_t byte4 = data & 0xFF;            // Cuarto byte
    uint8_t checksum = pio_sm_get(pio_, sm_) & 0xFF;  // Checksum byte
    uint8_t checksum_calc = (byte1+byte2+byte3+byte4) & 0xFF;
    // printf("Checksum: %02X\nChecksum calculado: %02X\n", checksum, checksum_calc);
    // Se verifica si la informacion obtenida es correcta
    // Si lo es, lo mas probable es que el sensor este funcionando correctamente
    if(checksum == checksum_calc){
        this->last_measurement.humidity = ((byte1 << 8) | byte2)/10.f;
        this->last_measurement.temperature = ((byte3 << 8) | byte4)/10.f;
        this->last_measurement.st = OK;
        return this->last_measurement;
    }
    else{
        this->last_measurement.humidity = ((byte1 << 8) | byte2)/10.f;
        this->last_measurement.temperature = ((byte3 << 8) | byte4)/10.f;
        this->last_measurement.st = NOT_OK;
        //pio_sm_clear_fifos(pio_, sm_);
        // pio_sm_restart(pio_, sm_);
        // AM2302_program_init(pio_, sm_, offset_, pin_);
        return last_measurement;
    }  
}
