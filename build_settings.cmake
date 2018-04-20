SET(TOOLCHAIN_PREFIX "c:/Program Files (x86)/GNU Tools ARM Embedded/6 2017-q2-update" CACHE PATH "toolchain prefix")
SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../project/stm32-cmake/cmake CACHE PATH "cmake module path")
SET(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../project/stm32-cmake/cmake/gcc_stm32.cmake CACHE FILEPATH "toolchain file")
SET(STM32_CHIP "STM32F407VGT6" CACHE STRING "stm32 chip")
SET(STM32Cube_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../source/stm32_cube CACHE PATH "cube path")

#SET(STM32_LINKER_SCRIPT "./project/linker/linker.ld" CACHE FILEPATH "linker file")

#SET(CMAKE_C_FLAGS "-mthumb -fno-builtin -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=soft -Wall -std=gnu11 -ffunction-sections -fdata-sections -fomit-frame-pointer -mabi=aapcs -fno-unroll-loops -ffast-math -ftree-vectorize" CACHE INTERNAL "c compiler flags")
#SET(CMAKE_CXX_FLAGS "-mthumb -fno-builtin -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=soft -Wall -std=c++11 -ffunction-sections -fdata-sections -fomit-frame-pointer -mabi=aapcs -fno-unroll-loops -ffast-math -ftree-vectorize" CACHE INTERNAL "cxx compiler flags")
#SET(CMAKE_ASM_FLAGS "-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -x assembler-with-cpp" CACHE INTERNAL "asm compiler flags")


