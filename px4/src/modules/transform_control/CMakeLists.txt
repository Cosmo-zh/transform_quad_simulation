px4_add_module(
    MODULE modules__transform_control
    MAIN transform_control
    STACK_MAIN 2000
    SRCS
        transform_control.cpp
    DEPENDS
        # 💡 新版 PX4 中参数系统和 uORB 已经被全局包含，无需显式在此处写出
        # 移除 parameters 和 uORB 即可避免报错
)
