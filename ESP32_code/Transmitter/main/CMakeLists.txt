idf_component_register(SRCS "hello_world_main.c" "si7021.c" "ADC.c"
                    INCLUDE_DIRS ".")
spiffs_create_partition_image(storage ../partition FLASH_IN_PROJECT)