# SPDX-License-Identifier: Apache-2.0

zephyr_library()

if (CONFIG_INA229)
  zephyr_library_sources(ina229.c)
endif()
