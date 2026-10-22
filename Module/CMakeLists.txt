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

add_library(rm_frame STATIC)
target_sources(rm_frame PRIVATE rm_frame.c)
target_include_directories(rm_frame PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(app_payload STATIC)
target_sources(app_payload PRIVATE app_payload.c)
target_include_directories(app_payload PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(app_payload PUBLIC rm_frame)

add_library(dm_can STATIC)
target_sources(dm_can PRIVATE dm_can.c)
target_include_directories(dm_can PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(bus_servo_group STATIC)
target_sources(bus_servo_group PRIVATE bus_servo_group.c)
target_include_directories(bus_servo_group PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(bus_servo_group PUBLIC bus_servo)
