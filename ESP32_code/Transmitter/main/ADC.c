#include "ADC.h" // ADC library to access sensor data handling functions
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_spiffs.h" // SPIFFS (SPI Flash File System) for file handling
#include "esp_log.h" // Logging functionality for esp32

// For reading from SPIFFS
static const char *TAG = "ADC"; // Log tag to identify logs related to ADC operations

static sensor_data_t current_data; // Use the struct defined in ADC.h
FILE *sensor_data_file = NULL; // Declare a file pointer to handle CSV file

void adc_init() {
    // Initialize SPIFFS
    esp_vfs_spiffs_conf_t config = {
        .base_path = "/storage", // Base path for SPIFFS storage
        .partition_label = NULL, // Use default partition table
        .max_files = 5, // Max number of open files allowed
        .format_if_mount_failed = true // Format SPIFFS if mount fails
    };
    esp_err_t result = esp_vfs_spiffs_register(&config); // Register SPIFFS with the esp32 Virtual File System

    if (result != ESP_OK) { // Check if SPIFFS registration failed
        ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(result)); // Log the error
        return;
    }
    // Get and log the partition information (total and used space in SPIFFS)
    size_t total = 0, used = 0;
    result = esp_spiffs_info(config.partition_label, &total, &used);
    if (result != ESP_OK) { // Check if partition info retreival failed
        ESP_LOGE(TAG, "Failed to get partition info (%s)", esp_err_to_name(result)); // Log the error
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used); // Log partition size and usage
    }

    // Initialize SPI, chip select pin, and open CSV file
    sensor_data_file = fopen("/storage/sensor_data.csv", "r"); // Open CSV file in read mode
    if (sensor_data_file == NULL) { // Check if the file opening fails
        printf("Failed to open CSV file.\n"); // print it
    } else {
        char header[256]; // Array to hold header line
        fgets(header, sizeof(header), sensor_data_file); // Skip the header line that shows sensor type
    }
    //Print message indicating that the ADC setup is complete
    printf("ADC set up complete. Reading test data from CSV file.\n");
}

void read_sensor_data_csv() {
    char line[256]; // Array to hold each line read from CSV file
    if (fgets(line, sizeof(line), sensor_data_file) != NULL) { // Read one line from the CSV file
        
        // Read sensor data values from the CSV and store them in current_data structure
        sscanf(line, "%*[^,],%f,%f,%f,%f", 
               &current_data.ammonia, &current_data.methane, &current_data.co2, &current_data.h2s);

        // Print the read sensor data to the console for debugging
        printf("Sensor data read from CSV: %.2f, %.2f, %.2f, %.2f\n", 
                current_data.ammonia, current_data.methane, current_data.co2, current_data.h2s);
    } else { // If end of file reached 
        printf("End of CSV file reached.\n");
        //rewind(sensor_data_file);  // Restart reading from the beginning if at the end
    }
}

// Adding this function to fetch current data 
sensor_data_t get_current_data() {
    return current_data;
}