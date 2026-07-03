cmake_minimum_required(VERSION 3.5)

list(APPEND SOURCES src/esp_ms5607.c)

list(APPEND requires esp_driver_gpio esp_driver_spi esp_timer)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS include
    PRIV_INCLUDE_DIRS src
    REQUIRES ${requires}
)