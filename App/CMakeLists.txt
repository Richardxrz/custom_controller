add_subdirectory(test)
option(CC_ROLE_ARM "Build the robot-side (arm) firmware" OFF)

add_library(app STATIC)
target_sources(app
PRIVATE
  app.c
  board_io.c
  uart_rx.c
  master_app.c
  arm_app.c
  irq_handlers.c
)
target_include_directories(app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(app PUBLIC
    stm32cubemx
    rx_dma
    tx_uart_it
    timer
    test
    ringbuf
    servo_codec
    bus_servo
    bus_servo_group
    rm_frame
    app_payload
    dm_can
)
if(CC_ROLE_ARM)
  target_compile_definitions(app PUBLIC CC_ROLE_ARM)
endif()
