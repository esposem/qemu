#include "qemu/osdep.h"
#include "cpu.h"
#include "qemu/host-utils.h"
#include "exec/exec-all.h"
#include "exec/helper-proto.h"
#include "exec/helper-head.h"
#include "exec/cpu_ldst.h"
#include "fpu/softfloat.h"
#include "internals.h"

#define MEMSET_BYTE 1

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

static void init_riscv_access(RISCVAccess *access, target_ulong size)
{
    uint64_t n_pages;

    n_pages = (size / TARGET_PAGE_SIZE) + 1;
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
    access->h_pages = NULL;
    access->addr_pages = NULL;
    access->sz_pages = NULL;
}

static uint32_t find_all_pages(CPURISCVState *env, RISCVAccess *access,
                           target_ulong addr, target_ulong size,
                           int mmu_idx, int ra)
{
    uint32_t k = 0;
    target_ulong sz;
    // printf("MAX Pages %ld\n", (uint64_t) n_pages *2);

    while (size > 0) {
        /* size that fits inside a page (taking into account offset) */
        sz = MIN(size, -(addr | TARGET_PAGE_MASK));

        /*
         * tlb_vaddr_to_host: tries to see in the TLB the hw address
         * (that corresponds to a host pointer).
         * It's a trapless lookup, so it might return NULL if it's not
         * found.
         */
        // printf("####Address %lu size %lu\n", (uint64_t) addr, (uint64_t) sz);
        // access->h_pages[k] = probe_write(env, addr, sz, mmu_idx, GETPC());
        access->h_pages[k] = tlb_vaddr_to_host(env, addr, MMU_DATA_STORE, mmu_idx);
        access->addr_pages[k] = addr;
        access->sz_pages[k++] = sz;

        size -= sz;
        addr += sz;
        // printf("New size is %lu\n", (uint64_t) size);

    }

    g_assert(size == 0);

    return k;
}

#endif

target_ulong helper_bbop(CPURISCVState *env, target_ulong src1,
                         target_ulong src2, target_ulong dest,
                         target_ulong size)
{
    return size;
}


#if !defined(CONFIG_USER_ONLY)
RISCVAccess rcc_src_access, rcc_dest_access;
int rcc_src_found_pages, rcc_dest_found_pages;
int rcc_i = 0;
bool rcc_faulted_all_src = false, rcc_faulted_all_dest = false;
static int rcc_mmu_idx;
static TCGMemOpIdx rcc_oi;
#endif

target_ulong helper_rcc(CPURISCVState *env, target_ulong src,
                        target_ulong dest, target_ulong size)
{

    target_ulong old_size = size;

#if defined(CONFIG_USER_ONLY)
    void *src_addr;
    void *dest_addr;

    src_addr = g2h(src);
    dest_addr = g2h(dest);
    memcpy(dest_addr, src_addr, size);
#else

    /* Only init once */
    if(rcc_src_access.h_pages == NULL) {
        rcc_mmu_idx = cpu_mmu_index(env, false);
        rcc_oi = make_memop_idx(MO_UB, rcc_mmu_idx);


        init_riscv_access(&rcc_src_access, size);
        init_riscv_access(&rcc_dest_access, size);

        rcc_src_found_pages = find_all_pages(env, &rcc_src_access, src, size, rcc_mmu_idx, GETPC());
        rcc_dest_found_pages = find_all_pages(env, &rcc_dest_access, dest, size, rcc_mmu_idx, GETPC());
    }

    /* Pre-fault all src pages once */
    if(rcc_faulted_all_src == false) {
        for(; rcc_i < rcc_src_found_pages; rcc_i++) {
            if(unlikely(rcc_src_access.h_pages[rcc_i] == NULL)) {
                /*
                 * Do a single access and test if we can then get access to the
                 * page. This is especially relevant to speed up TLB_NOTDIRTY.
                 */
                helper_ret_ldsb_mmu(env, rcc_src_access.addr_pages[rcc_i], rcc_oi, GETPC());

                rcc_src_access.h_pages[rcc_i] = tlb_vaddr_to_host(env,
                                                rcc_src_access.addr_pages[rcc_i],
                                                MMU_DATA_LOAD, rcc_mmu_idx);
            }
        }
        rcc_i = 0;
        rcc_faulted_all_src = true;
    }

    /* Pre-fault all dest pages once */
    if( rcc_faulted_all_dest == false) {
         for(; rcc_i < rcc_dest_found_pages; rcc_i++) {
            if(unlikely(rcc_dest_access.h_pages[rcc_i] == NULL)) {
                /*
                 * Do a single access and test if we can then get access to the
                 * page. This is especially relevant to speed up TLB_NOTDIRTY.
                 */
                helper_ret_stb_mmu(env, rcc_dest_access.addr_pages[rcc_i], 0, rcc_oi, GETPC());

                rcc_dest_access.h_pages[rcc_i] = tlb_vaddr_to_host(env,
                                                rcc_dest_access.addr_pages[rcc_i],
                                                MMU_DATA_STORE, rcc_mmu_idx);
            }
        }
        rcc_i = 0;
        rcc_faulted_all_dest = true;
    }

    int s = 0, d = 0;
    target_ulong ss, sd, chosen, off_ss = 0, off_sd = 0;

    while (size > 0) {

        ss = rcc_src_access.sz_pages[s];
        sd = rcc_dest_access.sz_pages[d];

        chosen = MIN(ss, sd);

        // printf("src: page %d size %lu offset %lu chosen %lu\n", s, (uint64_t) ss, (uint64_t) off_ss, (uint64_t) chosen);
        // printf("dest: page %d size %lu offset %lu chosen %lu\n", d, (uint64_t) sd, (uint64_t) off_sd, (uint64_t) chosen);

        /* do it manually if there is no page */
        if(likely(rcc_src_access.h_pages[s]) &&
           likely(rcc_dest_access.h_pages[d])) {

               memmove(rcc_dest_access.h_pages[d] + off_sd, rcc_src_access.h_pages[s] + off_ss, chosen);

        } else {
            for(int i=0; i < chosen; i++){
                uint8_t byte;

                byte = helper_ret_ldsb_mmu(env, rcc_src_access.addr_pages[s] + off_ss + i, rcc_oi, GETPC());

                helper_ret_stb_mmu(env, rcc_dest_access.addr_pages[d] + off_sd + i, byte, rcc_oi, GETPC());
            }
        }

        if(ss == sd) {
            s++;
            d++;
            off_sd = 0;
            off_ss = 0;
        } else if(chosen == ss){
            s++;
            rcc_dest_access.sz_pages[d] -= chosen;
            off_sd = chosen;
            off_ss = 0;
        } else {
            d++;
            rcc_src_access.sz_pages[s] -= chosen;
            off_ss = chosen;
            off_sd = 0;
        }

        size -= chosen;
    }

    del_riscv_access(&rcc_src_access);
    del_riscv_access(&rcc_dest_access);

    rcc_faulted_all_dest = false;
    rcc_faulted_all_src = false;
#endif
    return old_size;
}

#if !defined(CONFIG_USER_ONLY)

static RISCVAccess rci_access;
static int rci_found_pages, rci_i = 0;
static bool rci_faulted_all = false;
static int rci_mmu_idx;
static TCGMemOpIdx rci_oi;

#endif

target_ulong helper_rci(CPURISCVState *env, target_ulong dest,
                        target_ulong size)
{

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

    // hwaddr addr = riscv_cpu_get_phys_page_debug(env_cpu(env), dest);
    // printf("Virtual %lx Physical %lx\n", (uint64_t) dest, addr);

    addr = cpu_get_phys_page_debug(env_cpu(env), dest);
    // printf("Virtual %lx Physical %lx\n", (uint64_t) dest, addr);

    /* Only init once */
    if(rci_access.h_pages == NULL) {
        rci_mmu_idx = cpu_mmu_index(env, false);
        rci_oi = make_memop_idx(MO_UB, rci_mmu_idx);

        init_riscv_access(&rci_access, size);

        rci_found_pages = find_all_pages(env, &rci_access, dest, size, rci_mmu_idx, GETPC());
    }

    /* Pre-fault all pages once */
    if(rci_faulted_all == false) {
        for(; rci_i < rci_found_pages; rci_i++) {
            if(unlikely(rci_access.h_pages[rci_i] == NULL)) {
                /*
                 * Do a single access and test if we can then get access to the
                 * page. This is especially relevant to speed up TLB_NOTDIRTY.
                 */
                g_assert(rci_access.sz_pages[rci_i] > 0);

                helper_ret_stb_mmu(env, rci_access.addr_pages[rci_i], MEMSET_BYTE, rci_oi, GETPC());

                rci_access.h_pages[rci_i] = tlb_vaddr_to_host(env, rci_access.addr_pages[rci_i], MMU_DATA_STORE, rci_mmu_idx);
            }
        }
        rci_i = 0;
        rci_faulted_all = true;
    }


    for (int i=0; i < rci_found_pages; i++) {
        if (likely(rci_access.h_pages[i])) {
            /* There is a page in TLB, just memset it */
            memset(rci_access.h_pages[i], MEMSET_BYTE, rci_access.sz_pages[i]);
        } else {
            /* No luck, do it manually. Skip first byte bc already set */
            for (int j = 1; j < rci_access.sz_pages[i]; j++) {
                helper_ret_stb_mmu(env, rci_access.addr_pages[i] + j, MEMSET_BYTE, rci_oi, GETPC());
            }
        }
    }

    del_riscv_access(&rci_access);
    rci_faulted_all = false;
#endif
    return old_size;
}