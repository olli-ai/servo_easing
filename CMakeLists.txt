project(servo_easing LANGUAGES C)
set(CMAKE_C_FLAGS "-Wall")
cmake_minimum_required(VERSION 3.10)
option(EASING_HOST_BUILD "Build servo easing library on host machine" OFF)
option(EASING_TARGET_BUILD "Build servo easing library for target by cross" ON)
option(EASING_USE_FLOAT "Build servo easing library with no floating point op" OFF)
option(EASING_BUILD_TEST "Buil test app for library with dymmy controller" ON)
add_definitions(-DUSE_OLLI_LOG)

set(servo_easing_src    ${CMAKE_CURRENT_SOURCE_DIR}/src/SE_servo.c
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/SE_controller.c
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/SE_errors.c
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/SE_ticks.c
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/servo_easing.c
                        )

set(third_party_src     ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/logging/log.c)

if(EASING_USE_FLOAT)
    list(APPEND servo_easing_src ${CMAKE_CURRENT_SOURCE_DIR}/src/SE_algorithm.c)
elseif(NOT USE_FLOAT)
    list(APPEND servo_easing_src ${CMAKE_CURRENT_SOURCE_DIR}/src/SE_algorithm_no_fp.c)
endif(EASING_USE_FLOAT)

if(EASING_HOST_BUILD)
    list(APPEND servo_easing_src ${CMAKE_CURRENT_SOURCE_DIR}/src/Dummy/dummy_controller.c)
endif(EASING_HOST_BUILD)

if(EASING_TARGET_BUILD)
    list(APPEND servo_easing_src ${CMAKE_CURRENT_SOURCE_DIR}/src/PCA9685_linux/pca9685_linux_controller.c
                                 ${CMAKE_CURRENT_SOURCE_DIR}/src/MTK_9050/mtk_9050_linux_controller.c)
endif(EASING_TARGET_BUILD)

if (MCU_WITH_EXPANSION)
    list(APPEND servo_easing_src ${CMAKE_CURRENT_SOURCE_DIR}/src/PCA9685/pca9685_controller.c)
    target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/PCA9685)
endif(MCU_WITH_EXPANSION)

if (STM32_BOARD)
    list(APPEND servo_easing_src ${CMAKE_CURRENT_SOURCE_DIR}/src/STM32/stm32_controller.c)
    target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/STM32)
endif(STM32_BOARD)

add_library(${PROJECT_NAME} STATIC ${servo_easing_src} ${third_party_src})
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_include_directories(${PROJECT_NAME} PRIVATE "internal")
target_include_directories(${PROJECT_NAME} PRIVATE 3rd_party/logging)

if (EASING_USE_FLOAT)
    target_link_libraries(${PROJECT_NAME} m)
endif (EASING_USE_FLOAT)

if(EASING_HOST_BUILD)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_DUMMY_CONTROLLER)
endif(EASING_HOST_BUILD)

if(EASING_TARGET_BUILD)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_PCA9685_LINUX_CONTROLLER)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_MTK_9050_LINUX_CONTROLLER)
    target_link_libraries(${PROJECT_NAME} utils_olli)
endif(EASING_TARGET_BUILD)

if (MCU_WITH_EXPANSION)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_PCA9685_CONTROLLER)
endif(MCU_WITH_EXPANSION)

if (STM32_BOARD)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_STM32_CONTROLLER)
endif(STM32_BOARD)

if (EASING_BUILD_TEST)
add_subdirectory(test)
endif (EASING_BUILD_TEST)