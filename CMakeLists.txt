PROJECT(stm32f4_canpie-fd)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
ENABLE_LANGUAGE(ASM)

FIND_PACKAGE(CMSIS REQUIRED)
#FIND_PACKAGE(STM32LL COMPONENTS usart REQUIRED)
FIND_PACKAGE(STM32HAL COMPONENTS gpio can tim uart usart dma rcc flash pwr cortex REQUIRED)

INCLUDE_DIRECTORIES(
    ${CMSIS_INCLUDE_DIRS}
    ${STM32HAL_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/source
    ${CMAKE_CURRENT_SOURCE_DIR}/source/printf
    ${CMAKE_CURRENT_SOURCE_DIR}/source/canpie/source/canpie-fd
# use this for our driver    
    ${CMAKE_CURRENT_SOURCE_DIR}/source/canpie-fd_device/stm32/f4/hal
# use the next line for the microcontrol driver    
#    ${CMAKE_CURRENT_SOURCE_DIR}/source/canpie_stm32f4xx_canfd
)

SET(PROJECT_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/source/main.c
#    ${CMAKE_CURRENT_SOURCE_DIR}/source/newlib.c
    ${CMAKE_CURRENT_SOURCE_DIR}/source/stm32f4xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/source/stm32f4xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/source/printf/printf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/source/canpie/source/canpie-fd/cp_msg.c
    ${CMAKE_CURRENT_SOURCE_DIR}/source/canpie/source/canpie-fd/cp_fifo.c   
# use this for our driver
    ${CMAKE_CURRENT_SOURCE_DIR}/source/canpie-fd_device/stm32/hal/stm32_canpie-fd.c
# use the next line for the microcontrol driver
#	${CMAKE_CURRENT_SOURCE_DIR}/source/canpie_stm32f4xx_canfd/stm32f4xx_canfd.c
       
)

add_definitions(-DSTM32F407xx)
add_definitions(-DCP_DEBUG=1)
add_definitions(-DCP_DEVICE_TX_STRATEGY_ROLL)
# add this for the microcontrol driver
# add_definitions(-DDISABLE_HAL_CAN_IRQ)
# overrule the flag to no optimize
SET(CMAKE_C_FLAGS_DEBUG "-O0 -g3" CACHE INTERNAL "c compiler flags debug")

ADD_EXECUTABLE(${CMAKE_PROJECT_NAME} ${PROJECT_SOURCES} ${CMSIS_SOURCES} ${STM32HAL_SOURCES})

STM32_SET_TARGET_PROPERTIES(${CMAKE_PROJECT_NAME})
STM32_ADD_HEX_BIN_TARGETS(${CMAKE_PROJECT_NAME})
STM32_PRINT_SIZE_OF_TARGETS(${CMAKE_PROJECT_NAME})