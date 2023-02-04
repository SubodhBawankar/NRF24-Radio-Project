set(include_dirs include)

list(APPEND srcs     "src/NRF24L01.c")


idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "${include_dirs}"
                   REQUIRES driver efuse esp32 esp_adc_cal esp_common esp_event 
                                esp_rom freertos heap log soc spiffs)
