/*
 * Copyright (c) 2024 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TENSTORRENT_SYS_INIT_DEFINES_H_
#define TENSTORRENT_SYS_INIT_DEFINES_H_

#include <zephyr/init.h>

/* SYS_INIT APPLICATION defines */
/* Note: register_interrupt_handlers is at PRE_KERNEL_2 */
/* Note: ARC/NOC DMA driver is at POST_KERNEL, uses CONFIG_DMA_INIT_PRIORITY */
#define bh_arc_init_start_PRIO                1
#define CATEarlyInit_PRIO                     2
#define CalculateHarvesting_PRIO              3
#define DeassertTileResets_PRIO               4
#define PLLInit_PRIO                          5
#define PVTInit_PRIO                          6
#define NocInit_PRIO                          7
#define AssertSoftResets_PRIO                 8
#define DeassertRiscvResets_PRIO              9
#define InitAiclkPPM_PRIO                     10
#define pcie_init_PRIO                        11
#define tensix_init_PRIO                      12
#define InitMrisc_PRIO                        13
#define eth_init_PRIO                         14
#define InitSmbusTarget_PRIO                  15
#define regulator_init_PRIO                   16
#define avs_init_PRIO                         17
#define InitNocTranslationFromHarvesting_PRIO 18
#define gddr_training_PRIO                    19
#define CATInit_PRIO                          20
#define bh_arc_init_end_PRIO                  21

#define SYS_INIT_APP(func) SYS_INIT(func, APPLICATION, func##_PRIO)

#endif
