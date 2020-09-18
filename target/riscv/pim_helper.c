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
#if !defined(CONFIG_USER_ONLY)
/* An access can cover an arbitrary # of pages, so try to save them all */
typedef struct RISCVAccess {
    void **h_pages;
    target_ulong *addr_pages;
    target_ulong *sz_pages;
    target_ulong n_pages;
} RISCVAccess;

static void init_riscv_access(RISCVAccess *access, int n_pages)
{
    access->n_pages = n_pages;
    /* multiply by 2 because it can cross page boundary */
    access->h_pages = malloc(sizeof(void *) * n_pages * 2);
    g_assert(access->h_pages);
    access->addr_pages = malloc(sizeof(target_ulong *) * n_pages * 2);
    g_assert(access->addr_pages);
    access->sz_pages = malloc(sizeof(target_ulong *) * n_pages * 2);
    g_assert(access->sz_pages);
}

static void del_riscv_access(RISCVAccess *access)
{
    access->n_pages = 0;
    free(access->h_pages);
    free(access->addr_pages);
    free(access->sz_pages);
}

// #if 0
static uint32_t find_all_pages(CPURISCVState *env, RISCVAccess *access,
                           target_ulong dest, target_ulong size,
                           int mmu_idx, int ra)
{
    uint32_t k = 0;
    target_ulong sz;
    // printf("MAX Pages %ld\n", (uint64_t) n_pages *2);

    while (size > 0) {
        /* size that fits inside a page (taking into account offset) */
        sz = MIN(size, -(dest | TARGET_PAGE_MASK));

        /*
         * probe_access: tries to see in the TLB the hw address
         * (that corresponds to a host pointer)
         */
        printf("####Address %lu size %lu\n", (uint64_t) dest, (uint64_t) sz);
        access->h_pages[k] = probe_write(env, dest, sz,
                                    mmu_idx, GETPC());
        access->addr_pages[k] = dest;
        access->sz_pages[k++] = sz;

        size -= sz;
        dest += sz;
    }

    g_assert(size == 0);

    return k;
}
// #endif

#endif

target_ulong helper_bbop(CPURISCVState *env, target_ulong src1,
                         target_ulong src2, target_ulong dest,
                         target_ulong size)
{
    return size;
}


target_ulong helper_rcc(CPURISCVState *env, target_ulong src,
                        target_ulong dest, target_ulong size)
{
    void *src_addr;
    void *dest_addr;

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
#endif
    return size;
}


target_ulong helper_rci(CPURISCVState *env, target_ulong dest,
                        target_ulong size)
{

#define MEMSET_BYTE 1

    // void *dest_addr;
    target_ulong old_size = size;

    g_assert(size != 0);

#if defined(CONFIG_USER_ONLY)
    void *dest_addr;

    /* User: no MMU, just copy the data from one side to the other */
    dest_addr = g2h(dest);
    memset(dest_addr, MEMSET_BYTE, size);

#else
    /*
     * System: MMU, make sure to get the correct pages (host addresses)
     * via the TLB, and load missing pages if needed.
     */

    RISCVAccess access;
    int n_pages, mmu_idx, found_pages;
    TCGMemOpIdx oi;

    mmu_idx = cpu_mmu_index(env, false);
    oi = make_memop_idx(MO_UB, mmu_idx);

    n_pages = (size / TARGET_PAGE_SIZE) + 1;
    init_riscv_access(&access, n_pages);

    found_pages = find_all_pages(env, &access, dest, size, mmu_idx, GETPC());
    printf("Found %d pages\n", found_pages);

    // int k = 0;
    // target_ulong sz;

    // while (size > 0) {
    //     /* size that fits inside a page (taking into account offset) */
    //     sz = MIN(size, -(dest | TARGET_PAGE_MASK));

    //     /*
    //      * probe_access: tries to see in the TLB the hw address
    //      * (that corresponds to a host pointer)
    //      */
    //     access.h_pages[k] = probe_access(env, dest, sz, MMU_DATA_STORE,
    //                                 mmu_idx, GETPC());
    //     access.addr_pages[k] = dest;
    //     access.sz_pages[k++] = sz;

    //     size -= sz;
    //     dest += sz;
    // }

    // g_assert(size == 0);

    // found_pages = k;

    for (int i=0; i < found_pages; i++) {
        if (likely(access.h_pages[i])) {
            /* There is a page in TLB, just memset it */
            memset(access.h_pages[i], MEMSET_BYTE, access.sz_pages[i]);
        } else {
            /*
            * Do a single access and test if we can then get access to the
            * page. This is especially relevant to speed up TLB_NOTDIRTY.
            */
            g_assert(access.sz_pages[i] > 0);
            helper_ret_stb_mmu(env, access.addr_pages[i], MEMSET_BYTE, oi,
                               GETPC());
            access.h_pages[i] = tlb_vaddr_to_host(env, access.addr_pages[i],
                                          MMU_DATA_STORE, mmu_idx);
            if (likely(access.h_pages[i])) {
                /* Now there is a page in TLB, just memset it */
                memset(access.h_pages[i] + 1, MEMSET_BYTE, access.sz_pages[i] - 1);
            } else {
                /* No luck, do it manually */
                for (int j = 1; j < access.sz_pages[i]; j++) {
                    helper_ret_stb_mmu(env, access.addr_pages[i] + j, MEMSET_BYTE, oi, GETPC());
                }
            }
        }
    }

    del_riscv_access(&access);
#endif
    return old_size;
}