# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(max86178.c)
zephyr_library_sources_ifdef(CONFIG_MAX86178_TRIGGER max86178_trigger.c)
zephyr_library_sources_ifdef(CONFIG_SENSOR_ASYNC_API max86178_rtio.c max86178_decoder.c)
zephyr_library_sources_ifdef(CONFIG_MAX86178_STREAM max86178_stream.c max86178_decoder.c)
