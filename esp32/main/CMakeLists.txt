idf_component_register(SRCS "main.c" "libfec/init_rs_char.c" "libfec/encode_rs_char.c"
                       PRIV_REQUIRES driver spi_flash esp_timer
                       INCLUDE_DIRS "libfec")

