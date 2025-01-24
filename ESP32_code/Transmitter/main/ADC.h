#ifndef ADC_H
#define ADC_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// SPI pin configurations (modify as necessary)
#define SPI_CS   5  // Chip Select (CS)
#define SPI_CLK  18 // Clock
#define SPI_MISO 19 // Master In Slave Out
#define SPI_MOSI 21 // Master Out Slave In

// Define sensor channels (e.g., AIN1, AIN2, AIN3, AIN4)
typedef enum {
    AIN1_AMMONIA,
    AIN2_H2S,
    AIN3_CO2,
    AIN4_METHANE
} sensor_channel_t;

//Define sensor data structure
typedef struct {
    float ammonia;
    float h2s;
    float co2;
    float methane;
} sensor_data_t;

// Function prototypes
void adc_init();
float adc_read_sensor(sensor_channel_t channel);
void read_sensor_data_csv();
sensor_data_t get_current_data();

#endif