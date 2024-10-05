#ifdef __cplusplus
extern "C"
{
#endif

#ifndef MESURA_ADC
#define MESURA_ADC

/**
 * @brief Inicia l'ADC (configura l'atenuació)
 * 
 * @param num_ESP Número d'ESP, 1 o 2
 */
void init_ADC(int num_ESP);

/**
 * @brief Llegeix el corrent que està subministrant la bateria
 * 
 * @returns Corrent de la bateria en ampers
 */
float corrent_bateria();

/**
 * @brief Llegeix la tensió de subministrament, de bateria o USB-C
 * 
 * @returns Tensió de subministrament en volts
 */
float tensio_alimentacio();

/**
 * @brief Llegeix la temperatura del sensor en placa
 * 
 * @returns Temperatura en graus
 */
float temperatura_placa();

#endif MESURA_ADC

#ifdef __cplusplus
}
#endif