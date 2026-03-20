/* SPDX-License-Identifier: GPL-2.0 */
/* Mindgrove Custom PLIC Register Offsets */

#define MG_PLIC_NUM_SOURCES         82

/* Priority: one u32 per source */
#define MG_PLIC_PRIORITY_BASE       0x000000
#define MG_PLIC_PRIORITY_PER_ID     4

/* Pending (read-only bitmaps) */
#define MG_PLIC_PENDING_0_31        0x001000
#define MG_PLIC_PENDING_32_63       0x001004
#define MG_PLIC_PENDING_64_81       0x001008

/* Enable (read-write bitmaps) */
#define MG_PLIC_INTR_EN_0_31        0x002000
#define MG_PLIC_INTR_EN_32_63       0x002004
#define MG_PLIC_INTR_EN_64_81       0x002008

/* Control */
#define MG_PLIC_THRESHOLD           0x200000
#define MG_PLIC_CLAIM_COMPLETE      0x200004

#define MG_PLIC_DISABLE_THRESHOLD   0x7
#define MG_PLIC_ENABLE_THRESHOLD    0x0
