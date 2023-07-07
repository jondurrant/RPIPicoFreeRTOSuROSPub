add_executable(${NAME}
        main.cpp
        pico_usb_transport.c
        Agent.cpp
        BlinkAgent.cpp
        freertos_allocators.c
        uRosBridge.cpp
        uRosEntities.cpp
        PubEntities.cpp
        )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} 
	 pico_stdlib
	 pico_rand
     FreeRTOS-Kernel-Heap4
     micro_ros
	)
	
target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
     )
     
target_compile_definitions(${NAME} PRIVATE
    configNUM_CORES=1
)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 1)
add_compile_definitions(PICO_USB_ENABLE_CRLF_SUPPORT=0)
#add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
#add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)
target_compile_definitions(${NAME} PRIVATE
    PICO_DEFAULT_UART_RX_PIN=16
    PICO_DEFAULT_UART_TX_PIN=17
)