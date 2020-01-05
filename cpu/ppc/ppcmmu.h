//DingusPPC
//Written by divingkatae and maximum
//(c)2018-20 (theweirdo)     spatium
//Please ask for permission
//if you want to distribute this.
//(divingkatae#1017 or powermax#2286 on Discord)

// The opcodes for the processor - ppcopcodes.cpp

#ifndef PPCMEMORY_H
#define PPCMEMORY_H

#include <vector>
#include <array>

/* Uncomment this to exhaustive MMU integrity checks. */
//#define MMU_INTEGRITY_CHECKS

/** generic PowerPC BAT descriptor (MMU internal state) */
typedef struct PPC_BAT_entry {
    uint8_t     access;   /* copy of Vs | Vp bits */
    uint8_t     prot;     /* copy of PP bits */
    uint32_t    phys_hi;  /* high-order bits for physical address generation */
    uint32_t    lo_mask;  /* mask for low-order logical address bits */
    uint32_t    bepi;     /* copy of Block effective page index */
} PPC_BAT_entry;


extern void ibat_update(uint32_t bat_reg);
extern void dbat_update(uint32_t bat_reg);

extern void ppc_set_cur_instruction(const uint8_t *ptr);
extern void address_quickinsert_translate(uint32_t value_insert, uint32_t address_grab, uint8_t num_bytes);
extern void address_quickgrab_translate(uint32_t address_grab, uint8_t num_bytes);
extern uint8_t *quickinstruction_translate(uint32_t address_grab);

#endif // PPCMEMORY_H