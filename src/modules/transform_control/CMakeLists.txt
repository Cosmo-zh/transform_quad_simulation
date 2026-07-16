px4_add_module(
    MODULE modules__transform_control
    MAIN transform_control
    STACK_MAIN 2000
    SRCS
        transform_control.cpp
)
