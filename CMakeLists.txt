cmake_minimum_required(VERSION 3.30)
idf_component_register(SRCS "gps_test.c"
                       PRIV_REQUIRES driver spi_flash esp_timer
                       INCLUDE_DIRS "")

