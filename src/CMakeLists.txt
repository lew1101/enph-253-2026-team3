idf_component_register(
    SRC_DIRS
        "."
        "app_driver"
        "control"
        "comms"
        "tasks"
    INCLUDE_DIRS
        "../include"
)

target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++20)
