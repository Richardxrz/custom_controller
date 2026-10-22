add_library(test STATIC)
target_sources(test
PRIVATE
  test_tx.c
  test_rx.c
)
target_include_directories(test PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test PRIVATE tx_uart_it)
