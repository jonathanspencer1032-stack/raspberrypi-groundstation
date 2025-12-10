cmake_minimum_required(VERSION 3.10)
project(adafruit_lora_hat C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(adafruit_lora_hat adafruit_lora_hat.c)
target_include_directories(adafruit_lora_hat PRIVATE /usr/include)
target_link_libraries(adafruit_lora_hat PRIVATE gpiod)
