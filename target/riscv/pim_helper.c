#include "qemu/osdep.h"
#include "qemu/atomic.h"
#include "cpu.h"
#include "qemu/host-utils.h"
#include "exec/exec-all.h"
#include "exec/helper-proto.h"
#include "exec/helper-head.h"
#include "exec/cpu_ldst.h"
#include "fpu/softfloat.h"
#include "internals.h"
#include <math.h>

#define MEMSET_BYTE 1 /* FIXME: */
#define ENABLE_DELAY 0

#define CACHE_LINE_SIZE 64

#define DRAM_DEBUG 0

#if DRAM_DEBUG
#define  debug_printf(fmt, ...)  do { printf(fmt, ## __VA_ARGS__); }while(0);
#else
#define debug_printf(fmt, ...)    /* Do nothing */
#endif

#define CPU_DELAY(size) size
#define PIM_DELAY(size) 0
#define PSM_DELAY(size) (size / CACHE_LINE_SIZE)
#define FPM_DELAY(size) 0
#define AMBIT_DELAY(size) 0

#if !defined(CONFIG_USER_ONLY)

struct row_data;

typedef struct partial_row {
    hwaddr start;
    uint64_t size;
    void *host;
    int page_parent; // page that owns it
    struct row_data *row_parent; // row that owns it
    QSIMPLEQ_ENTRY(partial_row) next_partial; // next partial piece (same or next page)
    QSIMPLEQ_ENTRY(partial_row) next_same_row; // next with same row parent
} partial_row;

typedef QSIMPLEQ_HEAD(, partial_row) partial_row_list;

typedef struct row_data {
    hwaddr addr;
    void *host;
    uint64_t usage;
    QSIMPLEQ_ENTRY(row_data) next_row;
    partial_row_list partial_list;
    int partial_list_size;
} row_data;

typedef QSIMPLEQ_HEAD(, row_data) row_data_list;

static char *zero_row;

static inline void slow_down_by(CPURISCVState *env, size_t delay)
{
#if ENABLE_DELAY

    // printf("Delay %zu\n", delay);

    if(delay == 0)
        return;

    qemu_mutex_lock(&env->op_mutex);
    timer_mod(env->op_timer, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) + delay);
#endif
}


static uint64_t get_msb(hwaddr n)
{
    if(n == 0)
        return 0;

    n |= n >> 1;
    n |= n >> 2;
    n |= n >> 4;
    n |= n >> 8;
    n |= n >> 16;
    n |= n >> 32;
    n = n + 1;
    return n;
    // return (n >> 1);
}

static void init_riscv_access(RISCVAccess *access, target_ulong src, target_ulong size)
{
    uint64_t n_pages;

    n_pages = (size / TARGET_PAGE_SIZE) + 1;
    access->n_pages = n_pages;
    access->vaddr = src;
    access->size = size;
    /* multiply by 2 because it can cross page boundary */
    if(n_pages > access->max_pages){
        access->max_pages = n_pages * 2;
        access->pages = realloc(access->pages, sizeof(RISCVPage) * n_pages * 2);
    }

    g_assert(access->pages);
}

static void del_riscv_access(RISCVAccess *access)
{
    access->n_pages = 0;
    access->vaddr = 0;
    access->size = 0;
}

// it's about the column so don't care about randomization
static hwaddr get_row_mask(hwaddr phys, dram_cpu_info *info)
{
    hwaddr sizes = 0;
    hwaddr tot = get_el_value(&info->bank, phys);
    sizes = info->bank.size;
    tot |= get_el_value(&info->row, phys) * sizes;
    sizes *= info->row.size;
    tot |= get_el_value(&info->subarr, phys) * sizes;
    sizes *= info->subarr.size;
    tot |= get_el_value(&info->rank, phys)* sizes;
    sizes *= info->rank.size;
    tot |= get_el_value(&info->channel, phys) * sizes;
    // sizes *= info->channel.size;

    return tot;
}

static hwaddr get_next_row(hwaddr phys, CPURISCVState *env)
{
    hwaddr nextel = env->lsb_nocol; // 1[0] format
    hwaddr nextel_mask = nextel -1;
    return (phys & (~nextel_mask)) + nextel;
}

/* Oneday: this might be the same as done in LKM, so that is faster
 * But no need to do it now, it's just a safety check */
static void init_rowlist(CPURISCVState *env, dram_cpu_info *info,
                        RISCVAccess *access, row_data_list *row_list,
                        partial_row_list *partial_list)
{
    GHashTable *row_table;
    hwaddr start, next, end, nocol;
    uint64_t size_used;
    partial_row *prow;
    row_data *row;

    row_table = g_hash_table_new(g_int64_hash, g_int64_equal);
    QSIMPLEQ_INIT(row_list);
    QSIMPLEQ_INIT(partial_list);

    debug_printf("INIT ROWLIST %d\n", (int) access->n_pages);

    for (int i=0; i < access->n_pages; i++) {

        if (unlikely(!access->pages[i].host_addr)) {
            printf("MISSED PAGE %d?\n", i);
            // missed[missed_i++] = i;
            continue;
        }

        start = access->pages[i].phys_addr;
        next = get_next_row(start, env);
        end = access->pages[i].phys_addr + access->pages[i].size;

        while(start < end) {
            size_used = MIN(next, end) - start;

            debug_printf("start %lx next %lx\n", start, next);
            prow = g_new0(partial_row, 1);
            prow->start = start;
            prow->size = size_used;
            prow->page_parent = i;
            prow->host = (void *) ((uint64_t) access->pages[i].host_addr + (start - access->pages[i].phys_addr));
            QSIMPLEQ_INSERT_TAIL(partial_list, prow, next_partial);

            nocol = get_row_mask(start, info);

            /* search hash table by row index, but save addr as smallest addr within row. */
            row = g_hash_table_lookup(row_table, &nocol);
            debug_printf("Looking for row %lx in hashmap\n", nocol);

            /* build a linked list of rows, index them using hashmap
               but just temporarly */
            if (!row) {
                debug_printf("Row not found in hashmap, add it\n");
                row = g_new0(row_data, 1);
                g_assert(row);
                row->addr = start;
                QSIMPLEQ_INSERT_TAIL(row_list, row, next_row);
                QSIMPLEQ_INIT(&row->partial_list);
                g_hash_table_insert(row_table, &nocol, row);
            }

            prow->row_parent = row;

            if(start <= row->addr){
                row->addr = start;
                row->host = prow->host;
            }

            row->usage += size_used;

            QSIMPLEQ_INSERT_TAIL(&row->partial_list, prow, next_same_row);
            row->partial_list_size++;

            debug_printf("Partial row start %lx size %lu page %d row %lx(%lx)\n", prow->start, prow->size, prow->page_parent, nocol, row->addr);

            debug_printf("Row %lx(%lx) usage %lu\n", nocol, row->addr, row->usage);

            start = next;
            next = get_next_row(start, env);
        }
    }

    g_hash_table_destroy(row_table);
    debug_printf("######################\n");
}

static void del_rowlist(row_data_list *list)
{
    row_data *row = QSIMPLEQ_FIRST(list);
    row_data *ro2;
    while (row != NULL) {
            ro2 = QSIMPLEQ_NEXT(row, next_row);
            free(row);
            row = ro2;
    }
    QSIMPLEQ_INIT(list);
}

static void del_partiallist(partial_row_list *list)
{
    partial_row *row = QSIMPLEQ_FIRST(list);
    partial_row *ro2;
    while (row != NULL) {
            ro2 = QSIMPLEQ_NEXT(row, next_partial);
            free(row);
            row = ro2;
    }
    QSIMPLEQ_INIT(list);
}

static hwaddr virt_to_phys(CPURISCVState *env, dram_cpu_info *info,
                           target_ulong addr)
{
    CPUState *cpu = env_cpu(env);
    target_ulong align;
    hwaddr offset, ret;

    align = QEMU_ALIGN_DOWN(addr, TARGET_PAGE_SIZE);
    ret = cpu_get_phys_page_debug(cpu, align);
    offset = addr - align;

    g_assert(ret >= info->offset);
    // remove offset given by DRAM in cpu address space
    ret -= info->offset;

    if(ret != (hwaddr) -1)
        ret += offset;
    
    return ret;
}

static void *virt_to_host(CPURISCVState *env, target_ulong addr,
                          int access_type, dram_cpu_info *info,
                          int mmu_idx, TCGMemOpIdx oi, uintptr_t ra)
{
    void *ret;

    ret = tlb_vaddr_to_host(env, addr, access_type, mmu_idx);
    debug_printf("Initial address found %p\n", ret);

    if(ret)
        return ret;

    // try to access a byte so that the address is loaded in TLB
    debug_printf("Attempting pf\n");

    if(access_type == MMU_DATA_STORE)
        helper_ret_stb_mmu(env, addr, 0, oi, ra);
    else
        helper_ret_ldsb_mmu(env, addr, oi, ra);

    ret = tlb_vaddr_to_host(env, addr, access_type, mmu_idx);

    debug_printf("New address found %p\n", ret);

    return ret;
}

static uint32_t find_all_pages(CPURISCVState *env, RISCVAccess *access,
                                int access_type, dram_cpu_info *info,
                                int mmu_idx, TCGMemOpIdx oi, uintptr_t ra)
{
    target_ulong sz;
    RISCVPage *page;

    if(env->found_size == 0) {
        env->found_size = access->size;
        debug_printf("Size is %lu\n", (uint64_t) env->found_size);
        env->found_addr = access->vaddr;
    }

    while (env->found_size > 0) {
        /* size that fits inside a page (taking into account offset) */
        sz = MIN(env->found_size, -(env->found_addr | TARGET_PAGE_MASK));

        /*
         * tlb_vaddr_to_host: tries to see in the TLB the hw address
         * (that corresponds to a host pointer).
         * It's a trapless lookup, so it might return NULL if it's not
         * found.
         */
        page = &access->pages[env->found_index];
        page->v_addr = env->found_addr;
        page->size = sz;

        page->host_addr = virt_to_host(env, env->found_addr, access_type, info,
                                       mmu_idx, oi, ra);

        // handle physical page now
        page->phys_addr = virt_to_phys(env, info, env->found_addr);

        debug_printf("Page %d\nHost %lx\nPhys %lx\nVirt %lx\nSize %lu\n-----\n", env->found_index, (uint64_t) page->host_addr, page->phys_addr, (uint64_t)page->v_addr, (uint64_t) page->size);

        env->found_index++;
        env->found_size -= sz;
        env->found_addr += sz;

        g_assert(env->found_index <= (access->n_pages * 2));
    }

    debug_printf("################\n");

    g_assert(env->found_size == 0);
    access->n_pages = env->found_index;
    env->found_index = 0;
    env->found_size = 0;
    env->found_addr = 0;

    return access->n_pages;
}

static uint64_t fpm_psm_delay(CPURISCVState *env, target_ulong size, hwaddr start, hwaddr dest, rcc_stats *stat)
{
    uint64_t delay = 0;
    dram_cpu_info *info =  &(RISCV_CPU(env_cpu(env))->dram_info);

    hwaddr chans = get_el_value(&info->channel, start);
    hwaddr ranks = get_el_value(&info->rank, start);
    hwaddr banks = get_el_value(&info->bank, start);
    hwaddr subs = get_el_value(&info->subarr, start);
    hwaddr rows = get_el_value(&info->row, start);

    hwaddr chand = get_el_value(&info->channel, dest);
    hwaddr rankd = get_el_value(&info->rank, dest);
    hwaddr bankd = get_el_value(&info->bank, dest);
    hwaddr subd = get_el_value(&info->subarr, dest);
    hwaddr rowd = get_el_value(&info->row, dest);

    bool same_rc = (chans == chand) && (ranks == rankd);
    bool same_bank = (banks == bankd) && same_rc;
    bool same_sub = (subs == subd) && same_bank;
    bool same_row = (rows == rowd) && same_sub;

    debug_printf("Subarr src is %lx\n", subs);
    debug_printf("Subarr dest is %lx\n", subd);

    debug_printf("Bank src is %lx\n", banks);
    debug_printf("Bank dest is %lx\n", bankd);

    debug_printf("Row src is %lx\n", rows);
    debug_printf("Row dest is %lx\n", rowd);

    if(same_row) { // same row, all in CPU
        debug_printf("Slowdown CPU, row %lx == row %lx\n", rows, rowd);
        return CPU_DELAY(size);
    }

    if(same_bank) { // same bank
        // if same subarray, FPM
        // if different, PSM * 2
        if(same_sub) {
            debug_printf("Slowdown FPM\n");
            delay = FPM_DELAY(size);
            atomic_add(&stat->in_fpm, size);
            // stat->in_fpm += size;
        } else {
            debug_printf("Slowdown PSM 2, sub %lx != sub %lx\n", subs, subd);
            delay = PSM_DELAY(size) * 2;
            atomic_add(&stat->in_psm, size);
            // stat->in_psm += size;
        }
    } else if(same_rc){ // diff bank, same channel/rank, PSM
        debug_printf("Slowdown PSM, bank %lx != bank %lx\n", banks, bankd);
        delay = PSM_DELAY(size);
        atomic_add(&stat->in_psm, size);
        // stat->in_psm += size;
    } else { // different bank, but different channel
        debug_printf("Slowdown CPU, bank %lx == bank %lx\n", banks, bankd);
        return CPU_DELAY(size);
    }

    // stat->general.in_pim += size;
    atomic_add(&stat->general.in_pim, size);

    return delay;
}

static int compare_three_el(hwaddr el1, hwaddr el2, hwaddr el3)
{
    int res = 0;
    if(el1 == el2)
        res++;

    if(el2 == el3)
        res++;

    if(el3 == el1)
        res++;

    g_assert(res != 2);

    return res == 1 ? 2 : res; // 0 means 0 equal, 1 means 2 equal, 2 IMPOSSIBLE, 3 means all three equal
}

static uint64_t tri_fpm_psm_delay(CPURISCVState *env, target_ulong size, hwaddr start1, hwaddr start2, hwaddr dest, rcc_stats *stat)
{
    uint64_t delay = 0, sz = 0;
    dram_cpu_info *info =  &(RISCV_CPU(env_cpu(env))->dram_info);

    hwaddr chans1 = get_el_value(&info->channel, start1);
    hwaddr chans2 = get_el_value(&info->channel, start2);
    hwaddr chand = get_el_value(&info->channel, dest);
    sz = info->channel.size;

    hwaddr rcs1 = chans1 | (get_el_value(&info->rank, start1) * sz);
    hwaddr rcs2 = chans2 | (get_el_value(&info->rank, start2) * sz);
    hwaddr rcd = chand | (get_el_value(&info->rank, dest) * sz);
    sz *= info->rank.size;

    hwaddr banks1 = rcs1 | (get_el_value(&info->bank, start1 * sz));
    hwaddr banks2 = rcs2 | (get_el_value(&info->bank, start2 * sz));
    hwaddr bankd = rcd | (get_el_value(&info->bank, dest * sz));
    sz *= info->bank.size;

    hwaddr subs1 = banks1 | (get_el_value(&info->subarr, start1) * sz);
    hwaddr subs2 = banks2 | (get_el_value(&info->subarr, start2) * sz);
    hwaddr subd = bankd | (get_el_value(&info->subarr, dest) * sz);


    int n_same_rc = compare_three_el(rcd, rcs1, rcs2);
    int n_same_bank = compare_three_el(bankd, banks1, banks2);
    int n_same_sub = compare_three_el(subd, subs1, subs2);

    debug_printf("Subarr src1 is %lx\n", subs1);
    debug_printf("Subarr src2 is %lx\n", subs2);
    debug_printf("Subarr dest is %lx\n", subd);

    debug_printf("Bank src1 is %lx\n", banks1);
    debug_printf("Bank src2 is %lx\n", banks2);
    debug_printf("Bank dest is %lx\n", bankd);

    // op are:
    // cp(scr1->T0) cp(src2->T1) ambit(T1 T0->T2) cp(T2->dest)

    if(n_same_sub == 3){
        // if 3 same bank same sub --> 3*FPM
        debug_printf("All three in same sub, FPM\n");
        // stat->in_fpm += size;
        atomic_add(&stat->in_fpm, size);
    } else if (n_same_sub == 2) {
        if (n_same_bank == 3) {
            // if 2 same bank same sub, 1 diff sub -> 2*PSM + 2*FPM
            debug_printf("2 same sub, 1 diff sub, same bank\n");
            atomic_add(&stat->in_fpm, size/2);
            atomic_add(&stat->in_psm, size/2);
            // stat->in_fpm += size/2;
            // stat->in_psm += size/2;
        } else if (n_same_rc == 3) {
            // if 2 same bank same sub, 1 diff bank same rc -> PSM + 2*FPM
            debug_printf("2 same sub, 1 diff bank same rc\n");
            // stat->in_fpm += size/3 * 2;
            // stat->in_psm += size/3;
            atomic_add(&stat->in_fpm, size/3*2);
            atomic_add(&stat->in_psm, size/3);
        } else {
            // if 2 same bank same sub, 1 diff bank diff rc -> CPU + 2*FPM
            debug_printf("2 same sub, 1 diff bank diff rc\n");
            size = size/3 *2;
            // stat->in_fpm += size;
            atomic_add(&stat->in_fpm, size);
            // 1/3 is CPU
        }
    } else if (n_same_bank == 3) {
        g_assert(n_same_sub == 0);
        // if 3 same bank diff sub (all diff) -> 3*PSM
        debug_printf("3 same bank, diff sub (all diff)\n");
        // stat->in_psm += size;
        atomic_add(&stat->in_psm, size);
    } else if (n_same_bank == 2) {
        g_assert(n_same_sub == 0);
        g_assert(n_same_rc != 0);

        if(n_same_rc == 3){
            // if 2 same bank diff sub, 1 diff bank same rc -> 2*PSM + FPM
            debug_printf("2 same bank diff sub, 1 diff bank same rc\n");
            // stat->in_fpm += size/3;
            // stat->in_psm += size/3 * 2;
            atomic_add(&stat->in_fpm, size/3);
            atomic_add(&stat->in_psm, size/3*2);
        } else {
            // if 2 same bank diff sub, 1 diff bank diff rc -> 2*PSM + CPU
            debug_printf("2 same bank diff sub, 1 diff bank diff rc\n");
            size = size/3 *2;
            // stat->in_psm += size;
            atomic_add(&stat->in_psm, size);
            // 1/3 is CPU
        }
    } else if (n_same_rc == 3) {
        g_assert(n_same_bank == 0);
        // if 3 diff bank same rc (all diff) -> 2*PSM + FPM
        debug_printf("3 diff bank same rc (all diff)\n");
        // stat->in_fpm += size/3;
        // stat->in_psm += size/3 * 2;
        atomic_add(&stat->in_fpm, size/3);
        atomic_add(&stat->in_psm, size/3*2);
    } else if (n_same_rc == 2) {
        g_assert(n_same_bank == 0);
        // if 2 diff bank same rc, 1 diff bank diff rc -> PSM + FPM + CPU
        size = size/3;
        // stat->in_fpm += size;
        // stat->in_psm += size;
        atomic_add(&stat->in_fpm, size);
        atomic_add(&stat->in_psm, size);
        // 1/3 is CPU
        size *= 2;
    } else {
        g_assert(n_same_rc == 0);
        // if 3 diff bank diff rc (all diff) -> 2*CPU + FPM
        debug_printf("3 diff bank diff rc (all diff)\n");
        size = size/3;
        // stat->in_fpm += size;
        atomic_add(&stat->in_fpm, size);
        // 2/3 is CPU
    }

    atomic_add(&stat->general.in_pim, size);
    return delay;
}

static hwaddr get_next_address(hwaddr addr, int level, dram_cpu_info *info)
{
    return ((addr & ~(info->col.mask)) |
                ((addr + info->part_row_start[level]) & info->col.mask));
}

static void rec_iteration(int level, char **row, hwaddr phys, void *host,dram_cpu_info *info, uint64_t *size, char *src_buff)
{
    uint64_t host_64, j;

    for(j=0; j < (1 << info->col.bits[level]); j++){

        if(*size == 0) {
            return;
        }

        if (level != 1) { // here is 1 because we add bit[0] every iteration

            rec_iteration(level-1, row, phys, host, info, size, src_buff);

            host_64 = (uint64_t) host;
            host = (void *) get_next_address(host_64, level-1, info);
            phys = get_next_address(phys, level-1, info);
            continue;
        }

        uint64_t off_phys = (phys & info->part_row_end);
        uint64_t sz = MIN(info->part_row_end - off_phys + 1, *size);
        if(src_buff){ // copy from src to row
            memcpy(host, *row, sz);
            // printf("copying from %lx till %lx sz %lu\n", phys,  phys + sz, sz);
        } else {
            memcpy(*row, host, sz);
        }
        *row += sz;
        *size -= sz;

        if(off_phys) {
            host -= off_phys;
            phys -= off_phys;
        }

        host_64 = (uint64_t) host;
        host = (void *) get_next_address(host_64, 0, info);

        host_64 = phys;
        phys = get_next_address(phys, 0, info);
    }
}

typedef void (*qemu_src_dest) (void *dest, void *src, uint64_t sz);

static void rec_rr_iteration(int level, void *host_dest, void *host_src,
                          dram_cpu_info *info, uint64_t *size,
                          qemu_src_dest fn)
{
    uint64_t host_64, j;

    for(j=0; j < (1 << info->col.bits[level]); j++){

        if(*size == 0) {
            return;
        }

        if (level != 1) {
            rec_rr_iteration(level-1, host_dest, host_src, info, size, fn);
            host_64 = (uint64_t) host_src;
            host_src = (void *) get_next_address(host_64, level-1, info);

            host_64 = (uint64_t) host_dest;
            host_dest = (void *) get_next_address(host_64, level-1, info);
            continue;
        }

        uint64_t sz = MIN(info->part_row_end + 1, *size);
        fn(host_dest, host_src, sz);
        *size -= sz;

        host_64 = (uint64_t) host_src;
        host_src = (void *) get_next_address(host_64, 0, info);

        host_64 = (uint64_t) host_dest;
        host_dest = (void *) get_next_address(host_64, 0, info);
    }

}

typedef void (*qemu_ambit) (void *dest, void *src1, void *src2, uint64_t sz);

static void rec_rrr_iteration(int level, hwaddr phys_dest, void *host_dest,
                          hwaddr phys_src1, void *host_src1,
                          hwaddr phys_src2, void *host_src2,
                          dram_cpu_info *info, uint64_t *size,
                          qemu_ambit fn)
{
    uint64_t host_64, j;

    for(j=0; j < (1 << info->col.bits[level]); j++){

        if(*size == 0) {
            return;
        }

        if (level != 1) {
            rec_rrr_iteration(level-1, phys_dest, host_dest, phys_src1, host_src1, phys_src2, host_src2, info, size, fn);

            host_64 = (uint64_t) host_src1;
            host_src1 = (void *) get_next_address(host_64, level-1, info);
            phys_src1 = get_next_address(phys_src1, level-1, info);

            host_64 = (uint64_t) host_src2;
            host_src2 = (void *) get_next_address(host_64, level-1, info);
            phys_src2 = get_next_address(phys_src2, level-1, info);

            host_64 = (uint64_t) host_dest;
            host_dest = (void *) get_next_address(host_64, level-1, info);
            phys_dest = get_next_address(phys_dest, level-1, info);
            continue;
        }

        uint64_t sz = MIN(info->part_row_end + 1, *size);
        fn(host_dest, host_src1, host_src2, sz);
        // printf("copying from %lx till %lx sz %lu\n", phys_src,  phys_src + sz, sz);
        *size -= sz;

        host_64 = (uint64_t) host_src1;
        host_src1 = (void *) get_next_address(host_64, 0, info);
        phys_src1 = get_next_address(phys_src1, 0, info);

        host_64 = (uint64_t) host_src2;
        host_src2 = (void *) get_next_address(host_64, 0, info);
        phys_src2 = get_next_address(phys_src2, 0, info);

        host_64 = (uint64_t) host_dest;
        host_dest = (void *) get_next_address(host_64, 0, info);
        phys_dest = get_next_address(phys_dest, 0, info);
    }

}

static char *from_row(char *src_buff, hwaddr phys, void *host, dram_cpu_info *info, uint64_t size)
{
    char *row;

    g_assert(size <= info->col.size);

    if(src_buff)
        row = src_buff;
    else
        row = g_malloc0(size);

    char *res = row;
    rec_iteration(DRAM_MAX_BIT_INTERLEAVING-1, &row, phys, host, info, &size, src_buff);

    g_assert(size == 0);
    return res;
}

ATTRIBUTE_UNUSED
static char *get_from_row(hwaddr phys, void *host, dram_cpu_info *info, uint64_t size)
{
    return from_row(NULL, phys, host, info, size);
}

static void set_to_row(char *src_buff, hwaddr phys, void *host, dram_cpu_info *info, uint64_t size)
{
    from_row(src_buff, phys, host, info, size);
}

static void r_cp(void *dest, void *src, uint64_t sz)
{
    memcpy(dest, src, sz);
}

typedef void (*row_src_dest) (hwaddr src_phys, void *src_host, hwaddr dest_phys, void *dest_host, dram_cpu_info *info, uint64_t size);

static void row_memcpy(hwaddr src_phys, void *src_host, hwaddr dest_phys, void *dest_host, dram_cpu_info *info, uint64_t size)
{
    rec_rr_iteration(DRAM_MAX_BIT_INTERLEAVING-1, dest_host, src_host, info, &size, r_cp);
}

static void r_not(void *dest, void *src, uint64_t sz)
{
    char *s = (char *)src;
    char *d = (char *)dest;

    for(int i=0; i < sz; i++)
        d[i] = ~(s[i]);
}

static void row_not(hwaddr src_phys, void *src_host, hwaddr dest_phys, void *dest_host, dram_cpu_info *info, uint64_t size)
{
    rec_rr_iteration(DRAM_MAX_BIT_INTERLEAVING-1, dest_host, src_host, info, &size, r_not);
}

typedef void (*row_ambit) (hwaddr src_phys1, void *src_host1, hwaddr src_phys2, void *src_host2, hwaddr dest_phys, void *dest_host, dram_cpu_info *info, uint64_t size);

static void r_and(void *dest, void *src1, void *src2, uint64_t sz)
{
    char *s1 = (char *)src1;
    char *s2 = (char *)src2;
    char *d = (char *)dest;

    for(int i=0; i < sz; i++)
        d[i] = s1[i] & s2[i];
}

static void row_and(hwaddr src_phys1, void *src_host1, hwaddr src_phys2, void *src_host2, hwaddr dest_phys, void *dest_host, dram_cpu_info *info, uint64_t size)
{
    rec_rrr_iteration(DRAM_MAX_BIT_INTERLEAVING-1, dest_phys, dest_host, src_phys1, src_host1, src_phys2, src_host2, info, &size, r_and);
}

static void r_or(void *dest, void *src1, void *src2, uint64_t sz)
{
    char *s1 = (char *)src1;
    char *s2 = (char *)src2;
    char *d = (char *)dest;

    for(int i=0; i < sz; i++){
        d[i] = s1[i] | s2[i];
    }
}

static void row_or(hwaddr src_phys1, void *src_host1, hwaddr src_phys2, void *src_host2, hwaddr dest_phys, void *dest_host, dram_cpu_info *info, uint64_t size)
{
    rec_rrr_iteration(DRAM_MAX_BIT_INTERLEAVING-1, dest_phys, dest_host, src_phys1, src_host1, src_phys2, src_host2, info, &size, r_or);
}

static uint64_t get_increment_row_address(uint64_t increment, dram_cpu_info *info)
{
    uint64_t mask, res = 0;

    for(int i=0; i < info->col.n_sections; i++){
        mask = (1 << info->col.bits[i]) -1;
        res |= (increment & mask) << info->col.offsets[i];
        increment = increment >> info->col.bits[i];
    }

    return res;
}

static void init_zero_row(uint64_t row_size)
{
    if(!zero_row){
        zero_row = g_malloc0(row_size);
    #if MEMSET_BYTE
        memset(zero_row, MEMSET_BYTE, row_size);
     #endif
        debug_printf("Byte in row is %u\n", zero_row[0]);
    }
}

static void init_3_riscv_access_once(CPURISCVState *env, target_ulong src1,
                                     target_ulong src2,
                                     target_ulong dest, target_ulong size)
{
    if(env->rc_src_access.n_pages == 0) {
        env->rc_mmu_idx = cpu_mmu_index(env, false);
        env->rc_oi = make_memop_idx(MO_UB, env->rc_mmu_idx);

        if(src1 != 0)
            init_riscv_access(&env->rc_src_access, src1, size);

        if(src2 != 0)
            init_riscv_access(&env->rc_src2_access, src2, size);

        if(dest != 0)
            init_riscv_access(&env->rc_dest_access, dest, size);
    }
}

static void init_2_riscv_access_once(CPURISCVState *env, target_ulong src,
                                     target_ulong dest, target_ulong size)
{
    init_3_riscv_access_once(env, src, 0, dest, size);
}

static void init_riscv_access_once(CPURISCVState *env, target_ulong dest,
                                   target_ulong size)
{
    init_3_riscv_access_once(env, 0, 0, dest, size);
}

static void pre_fault_dest(CPURISCVState *env, dram_cpu_info *info,
                               uintptr_t curr_pc, rci_stats *stat)
{
    /* Pre-fault all dest pages once */
    if(env->rc_faulted_all_dest == false) {
        atomic_inc(&stat->avg_pf.sum);
        find_all_pages(env, &env->rc_dest_access, MMU_DATA_STORE, info, env->rc_mmu_idx, env->rc_oi, curr_pc);
        env->rc_faulted_all_dest = true;
        atomic_dec(&stat->avg_pf.sum);
    }
}

static void pre_fault_dest_rcc(CPURISCVState *env, dram_cpu_info *info,
                               uintptr_t curr_pc, rcc_stats *stat)
{
    pre_fault_dest(env, info, curr_pc, &stat->general);
}

static void pre_fault_src_dest(CPURISCVState *env, dram_cpu_info *info,
                               uintptr_t curr_pc, rcc_stats *stat)
{
    pre_fault_dest_rcc(env, info, curr_pc, stat);

     /* Pre-fault all src pages once */
    if(env->rc_faulted_all_src == false) {
        atomic_inc(&stat->general.avg_pf.sum);
        find_all_pages(env, &env->rc_src_access, MMU_DATA_LOAD, info, env->rc_mmu_idx, env->rc_oi, curr_pc);
        env->rc_faulted_all_src = true;
        atomic_dec(&stat->general.avg_pf.sum);
    }
}

static void pre_fault_src_src2_dest(CPURISCVState *env, dram_cpu_info *info,
                                    uintptr_t curr_pc, rcc_stats *stat)
{
    pre_fault_src_dest(env, info, curr_pc, stat);

    if(env->rc_faulted_all_src2 == false) {
        atomic_inc(&stat->general.avg_pf.sum);
        find_all_pages(env, &env->rc_src2_access, MMU_DATA_LOAD, info, env->rc_mmu_idx, env->rc_oi, curr_pc);
        env->rc_faulted_all_src2 = true;
        atomic_dec(&stat->general.avg_pf.sum);
    }
}

static void gather_stats_rci(rci_stats *stat, uint64_t delay_op)
{
    atomic_add(&stat->avg_delay.sum, delay_op);
#if USE_STDEV_STDERR
    g_assert(stat->avg_delay.n < STAT_MAX_DEL_VAL);
    stat->del_val[stat->avg_delay.n] = delay_op;
#endif
    atomic_inc(&stat->avg_delay.n);
    atomic_inc(&stat->avg_pf.n);
}

static void gather_stats(rcc_stats *stat, uint64_t delay_op)
{
    gather_stats_rci(&stat->general, delay_op);
}

static void reset_riscv_access(CPURISCVState *env)
{
    del_riscv_access(&env->rc_dest_access);
    g_assert(env->rc_faulted_all_dest);
    env->rc_faulted_all_dest = false;
}

static void reset_2_riscv_access(CPURISCVState *env)
{
    reset_riscv_access(env);
    del_riscv_access(&env->rc_src_access);
    g_assert(env->rc_faulted_all_src);
    env->rc_faulted_all_src = false;
}

static void reset_3_riscv_access(CPURISCVState *env)
{
    reset_2_riscv_access(env);
    del_riscv_access(&env->rc_src2_access);
    g_assert(env->rc_faulted_all_src2);
    env->rc_faulted_all_src2 = false;
}

#endif


// ########################## RCC ###############################
#if !defined(CONFIG_USER_ONLY)
static rcc_stats rcc_stat;
static rcc_stats anot_stat;

static void perform_src_dest_op(partial_row_list *partial_src,
                                partial_row_list *partial_dest,
                                qemu_src_dest fn)
{
    partial_row *src_part, *dest_part;
    uint64_t mov_size = 0;
    uint64_t srcs, dests;

    src_part = QSIMPLEQ_FIRST(partial_src);
    dest_part = QSIMPLEQ_FIRST(partial_dest);

    while(src_part && dest_part) {
        srcs = src_part->size;
        dests = dest_part->size;

        debug_printf("PART Src row %lx size %lu host %p\n", src_part->start, src_part->size, src_part->host);
        debug_printf("PART Dest row %lx size %lu host %p\n", dest_part->start, dest_part->size, dest_part->host);

        mov_size = MIN(srcs, dests);
        debug_printf("Move %lu\n", mov_size);

        fn(dest_part->host, src_part->host, mov_size);

        if(srcs < dests) {
            src_part = QSIMPLEQ_NEXT(src_part, next_partial);
            dest_part->size -= mov_size;
            dest_part->start += mov_size;
            dest_part->host = (void *) ((uint64_t) dest_part->host + mov_size);
        } else if (srcs > dests) {
            dest_part = QSIMPLEQ_NEXT(dest_part, next_partial);
            src_part->size -= mov_size;
            src_part->start += mov_size;
            src_part->host = (void *) ((uint64_t) src_part->host + mov_size);
        } else {
            src_part = QSIMPLEQ_NEXT(src_part, next_partial);
            dest_part = QSIMPLEQ_NEXT(dest_part, next_partial);
        }
    }
}

static uint64_t perform_src_dest_delay(CPURISCVState *env,
                                  row_data_list *row_src,
                                  row_data_list *row_dest,
                                  dram_cpu_info *info,
                                  rcc_stats *stat)
{
    row_data *src_row, *dest_row;
    uint64_t mov_size = 0, increment = 0;
    uint64_t delay_op = 0, delay = 0;
    uint64_t row_size;

    row_size = info->col.size;
    src_row = QSIMPLEQ_FIRST(row_src);
    dest_row = QSIMPLEQ_FIRST(row_dest);

    while(src_row && dest_row) {

        debug_printf("Src row %lx size %lu\n", src_row->addr, src_row->usage);
        debug_printf("Dest row %lx size %lu\n", dest_row->addr,
                                                dest_row->usage);

        if(src_row->usage == dest_row->usage &&
           src_row->usage == row_size) {

            /* maybe now we can think about PIM */
            delay = fpm_psm_delay(env, row_size, src_row->addr, dest_row->addr, stat);

        } else {
            /* here we can have src and/or dest not full,
             or full but with different number of pieces inside */

            mov_size = MIN(src_row->usage, dest_row->usage);
            increment = get_increment_row_address(mov_size, info);

            debug_printf("Uneven rows, copy only a piece %lu\n", mov_size);
            delay = CPU_DELAY(mov_size);
        }

        slow_down_by(env, delay);
        delay_op += delay;

        if(src_row->usage < dest_row->usage){
            src_row = QSIMPLEQ_NEXT(src_row, next_row);

            debug_printf("Next src, dest increased by %lu %lx\n", mov_size,  increment);

            dest_row->usage -= mov_size;
            dest_row->addr += increment;
            dest_row->host = (void *) ((uint64_t) dest_row->host + increment);
        } else if(src_row->usage > dest_row->usage) {
            dest_row = QSIMPLEQ_NEXT(dest_row, next_row);

            debug_printf("Next dest, src increased by %lu %lx\n", mov_size,  increment);

            src_row->usage -= mov_size;
            src_row->addr += increment;
            src_row->host = (void *) ((uint64_t) src_row->host + increment);
        } else {
            src_row = QSIMPLEQ_NEXT(src_row, next_row);
            dest_row = QSIMPLEQ_NEXT(dest_row, next_row);

            debug_printf("Next src and dest\n");
        }
    }

    return delay_op;
}

static void helper_src_dest(CPURISCVState *env, target_ulong src,
                            target_ulong dest, target_ulong size,
                            uintptr_t curr_pc, rcc_stats *stat,
                            qemu_src_dest fn)
{
    if(size == 0)
        return;

    dram_cpu_info *info;
    row_data_list row_src, row_dest;
    partial_row_list partial_src, partial_dest;
    uint64_t delay_op = 0;

    /* Only init once */
    init_2_riscv_access_once(env, src, dest, size);
    info =  &(RISCV_CPU(env_cpu(env))->dram_info);

    /* Page fault if needed, but find all pages. Code until here
     * can re-executed becauses find_all_pages triggers a pf. */
    pre_fault_src_dest(env, info, curr_pc, stat);

    init_rowlist(env, info, &env->rc_src_access, &row_src, &partial_src);
    init_rowlist(env, info, &env->rc_dest_access, &row_dest, &partial_dest);

    atomic_add(&stat->general.tot_bytes, size);

    /* Core operation, perform the actual copy */
    perform_src_dest_op(&partial_src, &partial_dest, fn);
    delay_op = perform_src_dest_delay(env, &row_src, &row_dest, info, stat);

    /* Final stats bookeeping and cleanup */
    gather_stats(stat, delay_op);

    del_rowlist(&row_src);
    del_rowlist(&row_dest);
    del_partiallist(&partial_src);
    del_partiallist(&partial_dest);

    reset_2_riscv_access(env);
    debug_printf("#######################\n");
}

#endif

void helper_rcc(CPURISCVState *env, target_ulong src,
                target_ulong dest, target_ulong size)
{
#if !defined(CONFIG_USER_ONLY)
    helper_src_dest(env, src, dest, size, GETPC(), &rcc_stat, r_cp);
#endif
}

void helper_anot(CPURISCVState *env, target_ulong src,
                target_ulong dest, target_ulong size)
{
#if !defined(CONFIG_USER_ONLY)
    helper_src_dest(env, src, dest, size, GETPC(), &anot_stat, r_not);
#endif
}

// ########################## AAND/AOR ###############################
#if !defined(CONFIG_USER_ONLY)
static rcc_stats aand_stat;
static rcc_stats aor_stat;

static void perform_ambit_op(partial_row_list *partial_src1,
                             partial_row_list *partial_src2,
                             partial_row_list *partial_dest,
                             qemu_ambit fn)
{
    partial_row *src_part1, *src_part2, *dest_part;
    uint64_t mov_size = 0;
    uint64_t srcs1, srcs2, dests;

    src_part1 = QSIMPLEQ_FIRST(partial_src1);
    src_part2 = QSIMPLEQ_FIRST(partial_src2);
    dest_part = QSIMPLEQ_FIRST(partial_dest);

    while(src_part1 && src_part2 && dest_part) {
        srcs1 = src_part1->size;
        srcs2 = src_part2->size;
        dests = dest_part->size;

        debug_printf("PART Src1 row %lx size %lu host %p\n", src_part1->start, src_part1->size, src_part1->host);
        debug_printf("PART Src2 row %lx size %lu host %p\n", src_part2->start, src_part2->size, src_part2->host);
        debug_printf("PART Dest row %lx size %lu host %p\n", dest_part->start, dest_part->size, dest_part->host);

        mov_size = MIN(srcs1, dests);
        mov_size = MIN(srcs2, mov_size);
        debug_printf("Move %lu\n", mov_size);

        fn(dest_part->host, src_part1->host, src_part2->host, mov_size);

        if(srcs1 == mov_size) {
            src_part1 = QSIMPLEQ_NEXT(src_part1, next_partial);
        } else {
            src_part1->size -= mov_size;
            src_part1->start += mov_size;
            src_part1->host = (void *) ((uint64_t) src_part1->host + mov_size);
        }

        if(srcs2 == mov_size) {
            src_part2 = QSIMPLEQ_NEXT(src_part2, next_partial);
        } else {
            src_part2->size -= mov_size;
            src_part2->start += mov_size;
            src_part2->host = (void *) ((uint64_t) src_part2->host + mov_size);
        }

        if(dests == mov_size) {
            dest_part = QSIMPLEQ_NEXT(dest_part, next_partial);
        } else {
            dest_part->size -= mov_size;
            dest_part->start += mov_size;
            dest_part->host = (void *) ((uint64_t) dest_part->host + mov_size);
        }
    }
}

static uint64_t perform_ambit_delay(CPURISCVState *env,
                                  row_data_list *row_src1,
                                  row_data_list *row_src2,
                                  row_data_list *row_dest,
                                  rcc_stats *stat,
                                  dram_cpu_info *info)
{
    row_data *src_row1, *src_row2, *dest_row;
    uint64_t mov_size = 0, increment = 0;
    uint64_t delay_op = 0, delay = 0;
    uint64_t row_size;

    row_size = info->col.size;
    src_row1 = QSIMPLEQ_FIRST(row_src1);
    src_row2 = QSIMPLEQ_FIRST(row_src2);
    dest_row = QSIMPLEQ_FIRST(row_dest);

    while(src_row1 && src_row2 && dest_row) {

        debug_printf("Src row %lx size %lu\n", src_row1->addr, src_row1->usage);
        debug_printf("Src row %lx size %lu\n", src_row2->addr, src_row2->usage);
        debug_printf("Dest row %lx size %lu\n", dest_row->addr,
                                                dest_row->usage);

        if(src_row1->usage == dest_row->usage &&
           src_row2->usage == dest_row->usage &&
           dest_row->usage == row_size) {

            /* maybe now we can think about PIM */
            delay = tri_fpm_psm_delay(env, row_size, src_row1->addr, src_row2->addr, dest_row->addr, stat);

            mov_size = row_size;

        } else {
            /* here we can have src1/2 and/or dest not full,
             or full but with different number of pieces inside */

            mov_size = MIN(src_row1->usage, dest_row->usage);
            mov_size = MIN(src_row2->usage, mov_size);
            increment = get_increment_row_address(mov_size, info);

            debug_printf("Uneven rows, copy only a piece %lu\n", mov_size);
            delay = CPU_DELAY(mov_size);
        }

        slow_down_by(env, delay);
        delay_op += delay;

        if(src_row1->usage == mov_size){
            src_row1 = QSIMPLEQ_NEXT(src_row1, next_row);
        } else {
            src_row1->usage -= mov_size;
            src_row1->addr += increment;
            src_row1->host = (void *) ((uint64_t) src_row1->host + increment);
        }

        if(src_row2->usage == mov_size){
            src_row2 = QSIMPLEQ_NEXT(src_row2, next_row);
        } else {
            src_row2->usage -= mov_size;
            src_row2->addr += increment;
            src_row2->host = (void *) ((uint64_t) src_row2->host + increment);
        }

        if(dest_row->usage == mov_size){
            dest_row = QSIMPLEQ_NEXT(dest_row, next_row);
        } else {
            dest_row->usage -= mov_size;
            dest_row->addr += increment;
            dest_row->host = (void *) ((uint64_t) dest_row->host + increment);
        }

    }

    return delay_op;
}

static void helper_ambit(CPURISCVState *env, target_ulong src1,
                  target_ulong src2,
                  target_ulong dest, target_ulong size,
                  rcc_stats *stat, uintptr_t curr_pc, qemu_ambit fn)
{
    if(size == 0)
        return;

    dram_cpu_info *info;
    row_data_list row_src1, row_src2, row_dest;
    partial_row_list partial_src1, partial_src2, partial_dest;
    uint64_t delay_op = 0;

    /* Only init once */
    init_3_riscv_access_once(env, src1, src2, dest, size);
    info =  &(RISCV_CPU(env_cpu(env))->dram_info);

    // printf("init done\n");
    /* Page fault if needed, but find all pages. Code until here
     * can re-executed becauses find_all_pages triggers a pf. */
    pre_fault_src_src2_dest(env, info, curr_pc, stat);

    // printf("prefault done\n");

    init_rowlist(env, info, &env->rc_src_access, &row_src1, &partial_src1);
    init_rowlist(env, info, &env->rc_src2_access, &row_src2, &partial_src2);
    init_rowlist(env, info, &env->rc_dest_access, &row_dest, &partial_dest);

    atomic_add(&stat->general.tot_bytes, size);

    /* Core operation, perform the actual copy */
    perform_ambit_op(&partial_src1, &partial_src2, &partial_dest, fn);
    // printf("op done\n");

    delay_op = perform_ambit_delay(env, &row_src1, &row_src2, &row_dest, stat, info);
    // printf("delay done\n");

    /* Final stats bookeeping and cleanup */
    gather_stats(stat, delay_op);
    // printf("stat done\n");

    del_rowlist(&row_src1);
    del_rowlist(&row_src2);
    del_rowlist(&row_dest);
    del_partiallist(&partial_src1);
    del_partiallist(&partial_src2);
    del_partiallist(&partial_dest);
    // printf("del done\n");

    reset_3_riscv_access(env);
    // printf("reset done\n");

    debug_printf("#######################\n");
}
#endif

void helper_aand(CPURISCVState *env, target_ulong src1, target_ulong src2,
                  target_ulong dest, target_ulong size)
{
#if !defined(CONFIG_USER_ONLY)
    helper_ambit(env, src1, src2, dest, size, &aand_stat, GETPC(), r_and);
#endif
}

void helper_aor(CPURISCVState *env, target_ulong src1, target_ulong src2,
                  target_ulong dest, target_ulong size)
{
#if !defined(CONFIG_USER_ONLY)
    helper_ambit(env, src1, src2, dest, size, &aor_stat, GETPC(), r_or);
#endif
}

// ########################## RCI ###############################
#if !defined(CONFIG_USER_ONLY)
static rci_stats rci_stat;

static uint64_t perform_rci(CPURISCVState *env, row_data_list *rows,
                            dram_cpu_info *info)
{
    row_data *tmp;
    uint64_t row_size;
    uint64_t delay = 0, delay_op = 0;

    row_size = info->col.size;

    QSIMPLEQ_FOREACH(tmp, rows, next_row) {

        debug_printf("Row %lx %p %lu\n", tmp->addr, tmp->host, tmp->usage);

        if(tmp->usage == row_size){
            debug_printf("FAST PIM\n");
            delay = FPM_DELAY(row_size);
            atomic_add(&rci_stat.in_pim, row_size);

            set_to_row(zero_row, tmp->addr, tmp->host, info, row_size);
        } else {
            debug_printf("SLOW CPU\n");
            delay = CPU_DELAY(tmp->usage);

            partial_row *ttmp;
            /* Do partial row at time because a row can have holes in it
            * because pages might not be contiguous, so maybe some pieces
            * are missing */
            QSIMPLEQ_FOREACH(ttmp, &tmp->partial_list, next_same_row) {
                g_assert(ttmp->row_parent == tmp);
                // debug_printf("copying from %p till %p sz %lu\n", ttmp->host, (void *) ((uint64_t) ttmp->host + ttmp->size), ttmp->size);
                memset(ttmp->host, MEMSET_BYTE, ttmp->size);
                env->rc_src_access.pages[ttmp->page_parent].size -= ttmp->size;
            }
        }

        slow_down_by(env, delay);
        delay_op += delay;
    }

    return delay_op;
}

#if 0
// control commit on dec 10
static uint64_t perform_rci_missed(CPURISCVState *env, uintptr_t curr_pc)
{
    uint64_t delay_op = 0;

    /* rci_missed pages (the ones that have to be set manually) */
    for (int i=0; i < n_rci_missed; i++) {
        rci_stat.otherATOM += env->rc_src_access.pages[rci_missed[i]].size;
        slow_down_by(env, env->rc_src_access.pages[rci_missed[i]].size);
        delay_op += env->rc_src_access.pages[rci_missed[i]].size;

        /* No luck, do it manually */
        for (int j = 0; j < env->rc_src_access.pages[rci_missed[i]].size; j++) {
            helper_ret_stb_mmu(env, env->rc_src_access.pages[rci_missed[i]].v_addr + j, MEMSET_BYTE, env->rc_oi, curr_pc);
        }
    }

    return delay_op;
}
#endif

#endif

void helper_rci(CPURISCVState *env, target_ulong dest,
                        target_ulong size)
{
    if(size == 0)
        return;

#if !defined(CONFIG_USER_ONLY)
    /*
     * System: MMU, make sure to get the correct pages (host addresses)
     * via the TLB, and load missing pages if needed.
     */
    dram_cpu_info *info;
    row_data_list rows;
    partial_row_list partial_rows;
    uint64_t row_size;
    uint64_t delay_op;

    /* Only init once */
    init_riscv_access_once(env, dest, size);
    info =  &(RISCV_CPU(env_cpu(env))->dram_info);
    row_size = info->col.size;

    /* Page fault if needed, but find all pages. Code until here
     * can re-executed becauses find_all_pages triggers a pf. */
    pre_fault_dest(env, info, GETPC(), &rci_stat);

    /* pre-init the zero rowbuffer*/
    init_zero_row(row_size);

    /* Stats bookeeping */
    atomic_add(&rci_stat.tot_bytes, size);

    /* parse pages in row and partial rows */
    init_rowlist(env, info, &env->rc_dest_access, &rows, &partial_rows);

    /* Core operation, perform the actual memset */
    delay_op = perform_rci(env, &rows, info);

    /* Final stats, and cleanup */
    gather_stats_rci(&rci_stat, delay_op);

    del_rowlist(&rows);
    del_partiallist(&partial_rows);

    reset_riscv_access(env);
    debug_printf("#######################\n");
#endif
}

// ########################## RCIK ###############################
#if !defined(CONFIG_USER_ONLY)
static rci_stats rcik_stat;
#endif

/* row_dest is the guest virtual address representing the row
 * that we want to memset.
 */
void helper_rcik(CPURISCVState *env, target_ulong row_dest)
{
#if !defined(CONFIG_USER_ONLY)

    if (!(env->priv >= PRV_S)) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }

    dram_cpu_info *info;
    uint64_t row_size, delay;
    hwaddr offset_row;

    info =  &(RISCV_CPU(env_cpu(env))->dram_info);

    /* Only init once */
    init_riscv_access_once(env, row_dest, get_msb(info->col.mask));
    row_size = info->col.size;
    init_zero_row(row_size);

    /* Page fault if needed, but find all pages. Code until here
     * can re-executed becauses find_all_pages triggers a pf. */
    pre_fault_dest(env, info, GETPC(), &rcik_stat);
    // printf("Phys addr %lx, Vaddr %lx\n", env->rc_dest_access.pages[0].phys_addr, (uint64_t) env->rc_dest_access.pages[0].v_addr);

    // if the row is unaligned, error. Keep the instrucion as simple as poss
    offset_row = env->rc_dest_access.pages[0].phys_addr & info->col.mask;
    g_assert(offset_row == 0);

    /* Stats bookeeping */
    atomic_add(&rcik_stat.tot_bytes, row_size);

    set_to_row(zero_row, env->rc_dest_access.pages[0].phys_addr, env->rc_dest_access.pages[0].host_addr, info, row_size);

    // printf("Full size, full speed\n");
    delay = FPM_DELAY(row_size);
    atomic_add(&rcik_stat.in_pim, row_size);

    slow_down_by(env, delay);

    /* Final stats, and cleanup */
    gather_stats_rci(&rcik_stat, delay);

    reset_riscv_access(env);
#endif
}


// ########################## RCCK / ANOTK ###############################
#if !defined(CONFIG_USER_ONLY)
static rcc_stats rcck_stat;
static rcc_stats anotk_stat;

static void helper_src_destk(CPURISCVState *env, target_ulong src, target_ulong dest, uintptr_t curr_pc, row_src_dest row_fn, rcc_stats *stat)
{
    if (!(env->priv >= PRV_S)) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, curr_pc);
    }

    dram_cpu_info *info;
    uint64_t delay_op = 0;
    hwaddr row_size;

    info =  &(RISCV_CPU(env_cpu(env))->dram_info);
    row_size = info->col.size;

    /* Only init once */
    init_2_riscv_access_once(env, src, dest, get_msb(info->col.mask));

    /* Page fault if needed, but find all pages. Code until here
     * can re-executed becauses find_all_pages triggers a pf. */
    pre_fault_src_dest(env, info, curr_pc, stat);
    atomic_add(&stat->general.tot_bytes, row_size);

    hwaddr phys_src = env->rc_src_access.pages[0].phys_addr;
    hwaddr phys_dest = env->rc_dest_access.pages[0].phys_addr;
    hwaddr offset_src, offset_dest;

    offset_src = phys_src & info->col.mask;
    offset_dest = phys_dest & info->col.mask;
    g_assert(offset_src == 0);
    g_assert(offset_dest == 0);

    /* Core operation, perform the actual copy */
    row_fn(phys_src,
            env->rc_src_access.pages[0].host_addr,
            phys_dest,
            env->rc_dest_access.pages[0].host_addr, info, row_size);

    /* Calculate the actual delay */
    delay_op = fpm_psm_delay(env, row_size, phys_src, phys_dest, stat);

    /* Final stats bookeeping and cleanup */
    gather_stats(stat, delay_op);

    reset_2_riscv_access(env);
    debug_printf("#######################\n");
}
#endif

void helper_rcck(CPURISCVState *env, target_ulong src, target_ulong dest)
{
#if !defined(CONFIG_USER_ONLY)
    helper_src_destk(env, src, dest, GETPC(), row_memcpy, &rcck_stat);
#endif
}

void helper_anotk(CPURISCVState *env, target_ulong src, target_ulong dest)
{
#if !defined(CONFIG_USER_ONLY)
    // printf("helper_anot\n");
    helper_src_destk(env, src, dest, GETPC(), row_not, &anotk_stat);
#endif
}

// ########################## AMBIT ORK/ANDK ###############################
#if !defined(CONFIG_USER_ONLY)
static rcc_stats aandk_stat;
static rcc_stats aork_stat;

static void helper_ambitk(CPURISCVState *env, target_ulong src1, target_ulong src2, target_ulong dest, uintptr_t curr_pc, row_ambit row_fn, rcc_stats *stat)
{
    if (!(env->priv >= PRV_S)) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, curr_pc);
    }

    dram_cpu_info *info;
    uint64_t delay_op = 0;
    hwaddr row_size;

    info =  &(RISCV_CPU(env_cpu(env))->dram_info);
    row_size = info->col.size;

    /* Only init once */
    init_3_riscv_access_once(env, src1, src2, dest, get_msb(info->col.mask));

    pre_fault_src_src2_dest(env, info, curr_pc, stat);
    atomic_add(&stat->general.tot_bytes, row_size);

    hwaddr phys_src1 = env->rc_src_access.pages[0].phys_addr;
    hwaddr phys_src2 = env->rc_src2_access.pages[0].phys_addr;
    hwaddr phys_dest = env->rc_dest_access.pages[0].phys_addr;
    hwaddr offset_src1, offset_src2, offset_dest;

    offset_src1 = phys_src1 & info->col.mask;
    offset_src2 = phys_src2 & info->col.mask;
    offset_dest = phys_dest & info->col.mask;

    if (offset_src1 != 0 || offset_src2 != 0 || offset_dest != 0) {
        riscv_raise_exception(env, RISCV_EXCP_INST_ACCESS_FAULT, curr_pc);
    }

    /* Core operation, perform the actual copy */
    row_fn(phys_src1,
           env->rc_src_access.pages[0].host_addr,
           phys_src2,
           env->rc_src2_access.pages[0].host_addr,
           phys_dest,
           env->rc_dest_access.pages[0].host_addr, info, row_size);

    /* Calculate the actual delay */
    delay_op = tri_fpm_psm_delay(env, row_size, phys_src1, phys_src2, phys_dest, stat);

    /* Final stats bookeeping and cleanup */
    gather_stats(stat, delay_op);

    reset_3_riscv_access(env);
    debug_printf("#######################\n");
}

#endif

void helper_aandk(CPURISCVState *env, target_ulong src1, target_ulong src2, target_ulong dest)
{
#if !defined(CONFIG_USER_ONLY)
    helper_ambitk(env, src1, src2, dest, GETPC(), row_and, &aandk_stat);
#endif
}

void helper_aork(CPURISCVState *env, target_ulong src1, target_ulong src2, target_ulong dest)
{
#if !defined(CONFIG_USER_ONLY)
    helper_ambitk(env, src1, src2, dest, GETPC(), row_or, &aork_stat);
#endif
}

// ########################## STATS ###############################
#if !defined(CONFIG_USER_ONLY)
static uint64_t calc_perc(uint64_t tot, uint64_t part)
{
    if(tot == 0)
        return 0;
    return part * 100 / tot;
}

#if USE_STDEV_STDERR
static double calc_stddev(uint64_t sum, uint64_t *el, int size)
{
    double variance = 0;
    double avg;

    if(size == 0){
        printf("STTDEV N is 0\n");
        return 0;
    }


    avg = sum / size;

    for (int i=0; i < size; i++) {
        variance += (el[i] - avg) * (el[i] - avg);
    }
    variance /= size-1;
    return sqrt(variance);
}

static double calc_stderr(double stddev, int n)
{
    if(n == 0)
        return 0;

    return stddev / sqrt(n);
}
#endif /* USE_STDEV_STDERR */

static void helper_rci_stat(CPURISCVState *env, bool k)
{
    const char *name = "RCI";
    const char *name2 = "RCIK";
    double stddev = 0, stdderr = 0;
    rci_stats *stat = &rci_stat;

    if(k){
        name = name2;
        stat = &rcik_stat;
    }

#if USE_STDEV_STDERR
    stddev = calc_stddev(stat->avg_delay.sum, stat->del_val, stat->avg_delay.n);
    stdderr = calc_stderr(stddev, stat->avg_delay.n);
#endif

    printf("%s STATS:\n\tProcessed:\t%ld\n\tPIM:\t%ld\t%ld%%\n\tCPU:\t%ld\t%ld%%\n\tOther:\t%ld\t%ld%%\nAvg pf/call: %ld\nAvg delay/call: %d\nStddev delay/call: %f\nStderr delay/call: %f\n----------------\n",
        name,
        stat->tot_bytes,

        stat->in_pim,
        calc_perc(stat->tot_bytes, stat->in_pim),

        stat->tot_bytes - stat->in_pim - stat->other,
        100 - calc_perc(stat->tot_bytes, stat->in_pim) -
        calc_perc(stat->tot_bytes, stat->other),

        stat->other,
        calc_perc(stat->tot_bytes, stat->other),

        stat->avg_pf.n == 0 ? 0 : (stat->avg_pf.sum / stat->avg_pf.n),
        stat->avg_delay.n == 0 ? 0 : ((uint32_t) stat->avg_delay.sum / stat->avg_delay.n),

        stddev, stdderr
        );

    memset(stat, 0, sizeof(rci_stats));
}

static void helper_rcc_stat(CPURISCVState *env, rcc_stats *stat,
                            const char *name)
{
    double stddev = 0, stdderr = 0;

#if USE_STDEV_STDERR
    stddev = calc_stddev(stat->general.avg_delay.sum, stat->general.del_val, stat->general.avg_delay.n);
    stdderr = calc_stderr(stddev, stat->general.avg_delay.n);
#endif

    printf("%s STATS:\n\tProcessed:\t%ld\n\tPIM:\t%ld\t%ld%%\n\t\tFPM:\t%ld\t%ld%%\n\t\tPSM:\t%ld\t%ld%%\n\tCPU:\t%ld\t%ld%%\n\tOther:\t%ld\t%ld%%\nAvg pf/call: %ld\nAvg delay/call: %d\nStddev delay/call: %f\nStderr delay/call: %f\n----------------\n",
        name,
        stat->general.tot_bytes,

        stat->general.in_pim,
        calc_perc(stat->general.tot_bytes, stat->general.in_pim),

        stat->in_fpm,
        calc_perc(stat->general.in_pim, stat->in_fpm),

        stat->in_psm,
        stat->in_psm == 0 ? 0 : 100ul - calc_perc(stat->general.in_pim, stat->in_fpm),

        stat->general.tot_bytes - stat->general.in_pim - stat->general.other,
        100 - calc_perc(stat->general.tot_bytes, stat->general.in_pim) - calc_perc(stat->general.tot_bytes, stat->general.other),

        stat->general.other,
        calc_perc(stat->general.tot_bytes, stat->general.other),

        stat->general.avg_pf.n == 0 ? 0 : (stat->general.avg_pf.sum / stat->general.avg_pf.n),

        stat->general.avg_delay.n == 0 ? 0 : ((uint32_t) stat->general.avg_delay.sum / stat->general.avg_delay.n),

        stddev, stdderr);

    memset(stat, 0, sizeof(rcc_stats));
}
#endif

void helper_stat(CPURISCVState *env, target_ulong val, target_ulong name)
{
#if !defined(CONFIG_USER_ONLY)
    // printf("Name %lx Val %ld\n", (uint64_t) name, (uint64_t) val);
    void *name_host = tlb_vaddr_to_host(env, name, MMU_DATA_LOAD, cpu_mmu_index(env, false));
    printf("#################################\n");
    printf("%s\n", (char *) name_host);
    printf("#################################\n");
    fflush(stdout);

    switch (val)
    {
    case 0:
        helper_rci_stat(env, false);
        break;
    case 1:
        helper_rcc_stat(env, &rcc_stat, "RCC");
        break;
    case 2:
        helper_rcc_stat(env, &aand_stat, "AAND");
        break;
    case 3:
        helper_rcc_stat(env, &aor_stat, "AOR");
        break;
    case 4:
        helper_rcc_stat(env, &anot_stat, "ANOT");
        break;
    case 5:
        helper_rci_stat(env, true);
        break;
    case 6:
        helper_rcc_stat(env, &rcck_stat, "RCCK");
        break;
    case 7:
        helper_rcc_stat(env, &aandk_stat, "AANDK");
        break;
    case 8:
        helper_rcc_stat(env, &aork_stat, "AORK");
        break;
    case 9:
        helper_rcc_stat(env, &anotk_stat, "ANOTK");
        break;
    
    default:
        printf("helper_stat: Command %ld not recognized!\n",(uint64_t) val);
        break;
    }
#endif
}

#if !defined(CONFIG_USER_ONLY)

static void helper_stat_i(target_ulong val)
{
    atomic_add(&rcik_stat.tot_bytes, val);
    atomic_add(&rcik_stat.avg_delay.sum, CPU_DELAY(val));
    atomic_inc(&rcik_stat.avg_delay.n);
}

static void helper_stat_c(target_ulong val, rcc_stats *stat)
{
    atomic_add(&stat->general.tot_bytes, val);
    atomic_add(&stat->general.avg_delay.sum, CPU_DELAY(val));
    atomic_inc(&stat->general.avg_delay.n);
}
#endif

void helper_incr_cpu(target_ulong val, target_ulong op)
{
#if !defined(CONFIG_USER_ONLY)
    switch (op) {
        case 0:
            helper_stat_i(val);
            break;
        case 1:
            helper_stat_c(val, &rcck_stat);
            break;
        case 2:
            helper_stat_c(val, &aandk_stat);
            break;
        case 3:
            helper_stat_c(val, &aork_stat);
            break;
        case 4:
            helper_stat_c(val, &anotk_stat);
            break;
    }
#endif
}