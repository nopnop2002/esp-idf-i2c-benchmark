set(component_srcs "main.c" "mcp23017.c")

if (CONFIG_LEGACY_DRIVER)
    list(APPEND component_srcs "i2cdev_legacy.c")
elseif (CONFIG_NEW_DRIVER)
    list(APPEND component_srcs "i2cdev_new.c")
endif()

idf_component_register(SRCS "${component_srcs}" INCLUDE_DIRS ".")
