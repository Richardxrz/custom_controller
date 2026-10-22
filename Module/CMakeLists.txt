add_library(rx_dma STATIC)
target_sources(rx_dma
PRIVATE
  rx_dma.c
)
target_include_directories(rx_dma PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(rx_dma PUBLIC stm32cubemx)

add_library(tx_uart_it STATIC)
target_sources(tx_uart_it
PRIVATE
  tx_uart_it.c
)
target_include_directories(tx_uart_it PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(tx_uart_it PUBLIC stm32cubemx)

add_library(timer STATIC)
target_sources(timer
PRIVATE
  timer.c
)
target_include_directories(timer PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(timer PUBLIC stm32cubemx)

add_library(servo STATIC)
target_sources(servo
PRIVATE
  servo.c
)
target_include_directories(servo PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(servo PUBLIC stm32cubemx)

add_library(ringbuf STATIC)
target_sources(ringbuf
PRIVATE
  ringbuf.c
)
target_include_directories(ringbuf PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(bus_servo STATIC)
target_sources(bus_servo
PRIVATE
  bus_servo.c
)
target_include_directories(bus_servo PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(bus_servo PUBLIC stm32cubemx servo_codec)

add_library(servo_codec STATIC)
target_sources(servo_codec
PRIVATE
  servo_codec.c
)
target_include_directories(servo_codec PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(servo_codec PUBLIC stm32cubemx)

add_library(settings INTERFACE)
target_include_directories(settings INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
