cmake_minimum_required(VERSION 3.10)
project(udp_json_listener C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0")
set(FREERTOS_SRC
    FreeRTOS/croutine.c
    FreeRTOS/list.c
    FreeRTOS/queue.c
    FreeRTOS/tasks.c
    FreeRTOS/timers.c
    FreeRTOS/event_groups.c
    FreeRTOS/portable/ThirdParty/GCC/Posix/port.c
    FreeRTOS/portable/MemMang/heap_3.c 
    FreeRTOS/portable/ThirdParty/GCC/Posix/utils/wait_for_event.c
)
add_library(freertos STATIC ${FREERTOS_SRC})
target_include_directories(freertos PUBLIC
    FreeRTOS/include
    FreeRTOS/portable/ThirdParty/GCC/Posix
)

# -------------------------------
# Your main executable
# -------------------------------
add_executable(udp_json_listener main.c receiver.c health.c detector.c)
target_include_directories(udp_json_listener PRIVATE 
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/jsmn
)

# Link FreeRTOS and pthread for POSIX port
target_link_libraries(udp_json_listener PRIVATE freertos pthread m)

