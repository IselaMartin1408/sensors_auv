idf_component_register(
    SRCS  "main_sensors.c" "flow.c" "BNO055.c" "GPS_EVK.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver           # Controladores (I2C, UART, GPIO, PWM, etc.)
        freertos         # Tareas RTOS y temporizadores
        esp_rom          # Funciones ROM y utilidades
        esp_timer        # Temporizadores de alta resolución
        esp_system       # Funciones del sistema (reset, info del chip)
        esp_hw_support   # Soporte de hardware (interrupciones, periféricos)
        log              # Sistema de logs ESP_LOGI / ESP_LOGE
        esp_event        # Sistema de eventos
            
)
