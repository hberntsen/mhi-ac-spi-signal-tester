idf_component_register(SRCS "main.cpp"
                       PRIV_REQUIRES esp_driver_gptimer
                       PRIV_REQUIRES esp_driver_spi
                       PRIV_REQUIRES esp_driver_gpio
                       INCLUDE_DIRS "")
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++26)
