cmake_minimum_required(VERSION 3.5)

list(APPEND SOURCES src/esp_ms5607.c)

list(APPEND priv_requires)
list(APPEND pub_requires)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS include
    PRIV_INCLUDE_DIRS src
    REQUIRES ${pub_requires}
    PRIV_REQUIRES ${priv_requires}
)