file(GLOB MAIN "*.c*" "*/*.c*" "*/**/*.c*")
idf_component_register(SRCS ${MAIN} INCLUDE_DIRS ".")

