project(servo_easing LANGUAGES C)
set(CMAKE_C_FLAGS "-Wall")
cmake_minimum_required(VERSION 3.10)

if ((CMAKE_SYSTEM_PROCESSOR MATCHES "AMD64.*") OR (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64"))
    add_definitions(-DHOST_BUILD)
    add_definitions(-DUSE_PRINTF_LOG)
    set(HOST_BUILD ON)
else()   
    add_definitions(-DTARGET_BUILD)
    # add_definitions(-DUSE_NONE_LOG)
    set(TARGET_BUILD ON)
endif()

set(servo_easing_src    ${CMAKE_CURRENT_SOURCE_DIR}/src/SE_servo.c
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/SE_controller.c
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/SE_errors.c
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/SE_ticks.c
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/servo_easing.c
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/SE_algorithm.c
                        )

set(third_party_src     ${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/logging/log.c)

if(HOST_BUILD)
    list(APPEND servo_easing_src ${CMAKE_CURRENT_SOURCE_DIR}/src/Dummy/dummy_controller.c)
endif(HOST_BUILD)

if(TARGET_BUILD)
    list(APPEND servo_easing_src ${CMAKE_CURRENT_SOURCE_DIR}/src/PCA9685_linux/pca9685_linux_controller.c
                                 ${CMAKE_CURRENT_SOURCE_DIR}/src/MTK_9050/mtk_9050_linux_controller.c)
endif(TARGET_BUILD)

if (MCU_WITH_EXPANSION)
    list(APPEND servo_easing_src ${CMAKE_CURRENT_SOURCE_DIR}/src/PCA9685/pca9685_controller.c)
    target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/PCA9685)
endif(MCU_WITH_EXPANSION)

if (STM32_BOARD)
    list(APPEND servo_easing_src ${CMAKE_CURRENT_SOURCE_DIR}/src/STM32/stm32_controller.c)
    target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/STM32)
endif(STM32_BOARD)

add_library(${PROJECT_NAME} STATIC ${servo_easing_src} ${third_party_src})
add_executable(servo_easing_test ${CMAKE_CURRENT_SOURCE_DIR}/test/test_servo_easing.c)
target_link_libraries(servo_easing_test ${PROJECT_NAME} m)
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_include_directories(${PROJECT_NAME} PRIVATE "internal")
target_include_directories(${PROJECT_NAME} PRIVATE 3rd_party/logging)
target_include_directories(servo_easing_test PRIVATE include)
target_include_directories(servo_easing_test PRIVATE 3rd_party/logging)
target_include_directories(servo_easing_test PRIVATE "internal")

if(HOST_BUILD)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_DUMMY_CONTROLLER)
endif(HOST_BUILD)

if(TARGET_BUILD)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_PCA9685_LINUX_CONTROLLER)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_MTK_9050_LINUX_CONTROLLER)
    target_link_libraries(${PROJECT_NAME} utils_olli)
endif(TARGET_BUILD)

if (MCU_WITH_EXPANSION)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_PCA9685_CONTROLLER)
endif(MCU_WITH_EXPANSION)

if (STM32_BOARD)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_STM32_CONTROLLER)
endif(STM32_BOARD)