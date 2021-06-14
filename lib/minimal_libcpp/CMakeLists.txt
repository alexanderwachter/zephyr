if(CONFIG_MINIMAL_LIB_CPP)

zephyr_library()

zephyr_system_include_directories(include)

zephyr_library_sources(
	src/cxa_guard.cpp
	src/cxa_handlers.cpp
)

endif()