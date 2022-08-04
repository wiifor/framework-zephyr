#
# Copyright (c) 2022 Wiifor SAS
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_S11 s11.c)