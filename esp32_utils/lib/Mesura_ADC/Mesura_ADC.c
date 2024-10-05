#include "Mesura_ADC.h"

#include "Placa.h"

#include "Arduino.h"

void init_ADC(int num_ESP){
    // Comprovem que el número d'ESP és vàlid
    if(num_ESP != 1 && num_ESP != 2) return;

    // Resolució per defecte, de 12 bits (màxima)
    analogReadResolution(12);

    // Configurem l'atenuació per pin. La de fins a 3,1 V, a més de tenir molt d'error a zero,
    // és poc lineal. Es limita a 100-1250 mV, DB_2_5.
    if(num_ESP == 1){
        analogSetPinAttenuation(GPIO_I_SENSE, ADC_2_5db);
    } else{
        analogSetPinAttenuation(GPIO_TEMP, ADC_2_5db);
        analogSetPinAttenuation(GPIO_SENSE_ALM, ADC_2_5db);
    }
}

float corrent_bateria(){
    return analogReadMilliVolts(GPIO_I_SENSE) / 1000.0 * RATIO_I_SENSE_V;
}

float tensio_alimentacio(){
    return analogReadMilliVolts(GPIO_SENSE_ALM) * RATIO_SENSE_ALM / 1000.0;
}

float temperatura_placa(){
    return ((analogReadMilliVolts(GPIO_TEMP) - TEMP_OFFSET_ZERO_MV) / TEMP_COEFICIENT_MV);
}