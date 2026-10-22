add_subdirectory(test)
add_library(app STATIC)
target_sources(app
PRIVATE
  app.c
)
target_include_directories(app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(app PUBLIC stm32cubemx rx_dma tx_uart_it timer test)
