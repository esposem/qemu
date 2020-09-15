#include "qemu/osdep.h"
#include "cpu.h"
#include "qemu/host-utils.h"
#include "exec/exec-all.h"
#include "exec/helper-proto.h"
#include "exec/helper-head.h"
#include "exec/cpu_ldst.h"
#include "fpu/softfloat.h"
#include "internals.h"

/*
    8 banks
    2^11 column per row
    2^14 rows
    1 channel

    [31] 		channel
    [30:17]		row
    [16:9]		high col
    [8:6]		bank
    [5:3]		low col
    [2:0]		byte in bus
*/

target_ulong helper_bbop(CPURISCVState *env, target_ulong src1,
                         target_ulong src2, target_ulong dest,
                         target_ulong size)
{
//     printf("hallo src %lx src2 %lx dest %lx size %lx\n", (uint64_t) src1, (uint64_t) src2, (uint64_t) dest, (uint64_t) size);


// #if defined(CONFIG_USER_ONLY)
//     char *src1_addr = (char *) g2h(src1); // read
//     if(src2){

//     }
//     char *src2_addr = (char *) g2h(src2); // read
//     printf("Read |%s|\n", addr);
// #else
//         char *addr = (char *)tlb_vaddr_to_host(env, src1, 0, cpu_mmu_index(env, 0)); // read
// #endif

//     char *ddest = (char *)tlb_vaddr_to_host(env, dest, 0, cpu_mmu_index(env, 0));


//     memcpy(ddest, addr, size);

// #ifdef CONFIG_SOFTMMU
//     // TODO: test this
//     helper_ret_stb_mmu(env, dest, 0, cpu_mmu_index(env, 0), GETPC());
// #endif

    return size;
}


target_ulong helper_rcc(CPURISCVState *env, target_ulong src,
                        target_ulong dest, target_ulong size)
{
    void *src_addr;
    void *dest_addr;

    // printf("rcc src %lx dest %lx size %lx\n",
    //         (uint64_t) src,
    //         (uint64_t) dest,
    //         (uint64_t) size);

#if defined(CONFIG_USER_ONLY)
    src_addr = g2h(src);
    dest_addr = g2h(dest);
    memcpy(dest_addr, src_addr, size);
#else
    src_addr = tlb_vaddr_to_host(env, src, MMU_DATA_LOAD,
                                 cpu_mmu_index(env, 0));
    dest_addr = tlb_vaddr_to_host(env, dest, MMU_DATA_STORE,
                                  cpu_mmu_index(env, 0));

    memcpy(dest_addr, src_addr, size);

    // TCGMemOpIdx oi = make_memop_idx(MO_UB, mmu_idx);
    // int i;

    // if (likely(haddr)) {
    //     memset(haddr, byte, size);
    // } else {
    //     /*
    //      * Do a single access and test if we can then get access to the
    //      * page. This is especially relevant to speed up TLB_NOTDIRTY.
    //      */
    //     g_assert(size > 0);
    //     helper_ret_stb_mmu(env, vaddr, byte, oi, ra);
    //     haddr = tlb_vaddr_to_host(env, vaddr, MMU_DATA_STORE, mmu_idx);
    //     if (likely(haddr)) {
    //         memset(haddr + 1, byte, size - 1);
    //     } else {
    //         for (i = 1; i < size; i++) {
    //             helper_ret_stb_mmu(env, vaddr + i, byte, oi, ra);
    //         }
    //     }
    // }
#endif
    return size;
}


target_ulong helper_rci(CPURISCVState *env, target_ulong dest,
                        target_ulong size)
{
    void *dest_addr;

    // printf("rci src %lx size %lx\n",
    //         (uint64_t) dest,
    //         (uint64_t) size);

#if defined(CONFIG_USER_ONLY)
    dest_addr = g2h(dest);
    memset(dest_addr, 0, size);
#else
    dest_addr = tlb_vaddr_to_host(env, dest, MMU_DATA_STORE,
                                  cpu_mmu_index(env, 0));

    memset(dest_addr, 0, size);

    // TCGMemOpIdx oi = make_memop_idx(MO_UB, mmu_idx);
    // int i;

    // if (likely(haddr)) {
    //     memset(haddr, byte, size);
    // } else {
    //     /*
    //      * Do a single access and test if we can then get access to the
    //      * page. This is especially relevant to speed up TLB_NOTDIRTY.
    //      */
    //     g_assert(size > 0);
    //     helper_ret_stb_mmu(env, vaddr, byte, oi, ra);
    //     haddr = tlb_vaddr_to_host(env, vaddr, MMU_DATA_STORE, mmu_idx);
    //     if (likely(haddr)) {
    //         memset(haddr + 1, byte, size - 1);
    //     } else {
    //         for (i = 1; i < size; i++) {
    //             helper_ret_stb_mmu(env, vaddr + i, byte, oi, ra);
    //         }
    //     }
    // }
#endif
    return size;
}