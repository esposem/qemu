#include "qemu/osdep.h"
#include "cpu.h"
#include "qemu/host-utils.h"
#include "exec/exec-all.h"
#include "exec/helper-proto.h"
#include "exec/helper-head.h"
#include "exec/cpu_ldst.h"
#include "fpu/softfloat.h"
#include "internals.h"
#include <math.h>

#define MEMSET_BYTE 1
#define ENABLE_DELAY 0

#define CACHE_LINE_SIZE 64

#define DRAM_DEBUG 0
#define USE_STDEV_STDERR 0

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

/* An access can cover an arbitrary # of pages, so try to save them all */
typedef struct RISCVPage {
    void *host_addr;
    target_ulong v_addr;
    hwaddr phys_addr;
    target_ulong size;
} RISCVPage;

typedef struct RISCVAccess {
    RISCVPage *pages;    // pages touched by request
    target_ulong n_pages;
    target_ulong vaddr; // request vaddr
    target_ulong size; // request size
} RISCVAccess;

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

typedef struct avg_t {
    uint64_t sum;
    int n;
} avg_t;

#define STAT_MAX_DEL_VAL 10000
typedef struct rci_stats {
    avg_t avg_pf;
    avg_t avg_delay;
#if USE_STDEV_STDERR
    uint64_t del_val[STAT_MAX_DEL_VAL];
#endif
    uint64_t tot_bytes;
    uint64_t in_pim;
    uint64_t other;
} rci_stats;

typedef struct rcc_stats {
    rci_stats general;
    uint64_t in_fpm;
    uint64_t in_psm;
} rcc_stats;

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

static void init_riscv_access(RISCVAccess *access, target_ulong src, target_ulong size)
{
    uint64_t n_pages;

    n_pages = (size / TARGET_PAGE_SIZE) + 1;
    access->n_pages = n_pages;
    access->vaddr = src;
    access->size = size;
    /* multiply by 2 because it can cross page boundary */
    access->pages = malloc(sizeof(RISCVPage) * n_pages * 2);
    g_assert(access->pages);
}

static void del_riscv_access(RISCVAccess *access)
{
    access->n_pages = 0;
    access->vaddr = 0;
    free(access->pages);
    access->pages = NULL;
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
static int init_rowlist(CPURISCVState *env, dram_cpu_info *info,
                        RISCVAccess *access, row_data_list *row_list,
                        partial_row_list *partial_list, int *missed)
{
    int missed_i = 0;
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
            debug_printf("MISSED PAGE %d?\n", i);
            missed[missed_i++] = i;
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
    return missed_i;
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

static uint32_t found_index = 0;
static target_ulong found_size = 0;
static target_ulong found_addr = 0;

static uint32_t find_all_pages(CPURISCVState *env, RISCVAccess *access,
                                int access_type, dram_cpu_info *info,
                                int mmu_idx, TCGMemOpIdx oi, uintptr_t ra)
{
    target_ulong sz;
    RISCVPage *page;

    if(found_size == 0) {
        found_size = access->size;
        debug_printf("Size is %lu\n", (uint64_t) found_size);
        found_addr = access->vaddr;
    }

    while (found_size > 0) {
        /* size that fits inside a page (taking into account offset) */
        sz = MIN(found_size, -(found_addr | TARGET_PAGE_MASK));

        /*
         * tlb_vaddr_to_host: tries to see in the TLB the hw address
         * (that corresponds to a host pointer).
         * It's a trapless lookup, so it might return NULL if it's not
         * found.
         */
        page = &access->pages[found_index];
        page->v_addr = found_addr;
        page->size = sz;

        page->host_addr = virt_to_host(env, found_addr, access_type, info,
                                       mmu_idx, oi, ra);

        // handle physical page now
        page->phys_addr = virt_to_phys(env, info, found_addr);

        debug_printf("Page %d\nHost %lx\nPhys %lx\nVirt %lx\nSize %lu\n-----\n", found_index, (uint64_t) page->host_addr, page->phys_addr, (uint64_t)page->v_addr, (uint64_t) page->size);

        found_index++;
        found_size -= sz;
        found_addr += sz;

        g_assert(found_index <= (access->n_pages * 2));
    }

    debug_printf("################\n");

    g_assert(found_size == 0);
    access->n_pages = found_index;
    found_index = 0;
    found_size = 0;
    found_addr = 0;

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
            stat->in_fpm += size;
        } else {
            debug_printf("Slowdown PSM 2, sub %lx != sub %lx\n", subs, subd);
            delay = PSM_DELAY(size) * 2;
            stat->in_psm += size;
        }
    } else if(same_rc){ // diff bank, same channel/rank, PSM
        debug_printf("Slowdown PSM, bank %lx != bank %lx\n", banks, bankd);
        delay = PSM_DELAY(size);
        stat->in_psm += size;
    } else { // different bank, but different channel
        debug_printf("Slowdown CPU, bank %lx == bank %lx\n", banks, bankd);
        return CPU_DELAY(size);
    }

    stat->general.in_pim += size;

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
        stat->in_fpm += size;
    } else if (n_same_sub == 2) {
        if (n_same_bank == 3) {
            // if 2 same bank same sub, 1 diff sub -> 2*PSM + 2*FPM
            debug_printf("2 same sub, 1 diff sub, same bank\n");
            stat->in_fpm += size/2;
            stat->in_psm += size/2;
        } else if (n_same_rc == 3) {
            // if 2 same bank same sub, 1 diff bank same rc -> PSM + 2*FPM
            debug_printf("2 same sub, 1 diff bank same rc\n");
            stat->in_fpm += size/3 * 2;
            stat->in_psm += size/3;
        } else {
            // if 2 same bank same sub, 1 diff bank diff rc -> CPU + 2*FPM
            debug_printf("2 same sub, 1 diff bank diff rc\n");
            size = size/3 *2;
            stat->in_fpm += size;
            // 1/3 is CPU
        }
    } else if (n_same_bank == 3) {
        g_assert(n_same_sub == 0);
        // if 3 same bank diff sub (all diff) -> 3*PSM
        debug_printf("3 same bank, diff sub (all diff)\n");
        stat->in_psm += size;
    } else if (n_same_bank == 2) {
        g_assert(n_same_sub == 0);
        g_assert(n_same_rc != 0);

        if(n_same_rc == 3){
            // if 2 same bank diff sub, 1 diff bank same rc -> 2*PSM + FPM
            debug_printf("2 same bank diff sub, 1 diff bank same rc\n");
            stat->in_fpm += size/3;
            stat->in_psm += size/3 * 2;
        } else {
            // if 2 same bank diff sub, 1 diff bank diff rc -> 2*PSM + CPU
            debug_printf("2 same bank diff sub, 1 diff bank diff rc\n");
            size = size/3 *2;
            stat->in_psm += size;
            // 1/3 is CPU
        }
    } else if (n_same_rc == 3) {
        g_assert(n_same_bank == 0);
        // if 3 diff bank same rc (all diff) -> 2*PSM + FPM
        debug_printf("3 diff bank same rc (all diff)\n");
        stat->in_fpm += size/3;
        stat->in_psm += size/3 * 2;
    } else if (n_same_rc == 2) {
        g_assert(n_same_bank == 0);
        // if 2 diff bank same rc, 1 diff bank diff rc -> PSM + FPM + CPU
        size = size/3;
        stat->in_fpm += size;
        stat->in_psm += size;
        // 1/3 is CPU
        size *= 2;
    } else {
        g_assert(n_same_rc == 0);
        // if 3 diff bank diff rc (all diff) -> 2*CPU + FPM
        debug_printf("3 diff bank diff rc (all diff)\n");
        size = size/3;
        stat->in_fpm += size;
        // 2/3 is CPU
    }

    stat->general.in_pim += size;
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

static void rec_rr_iteration(int level, void *host_dest, void *host_src,
                          dram_cpu_info *info, uint64_t *size,
                          void (*fn)(void *dest, void *src, uint64_t sz))
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

static void rec_rrr_iteration(int level, hwaddr phys_dest, void *host_dest,
                          hwaddr phys_src1, void *host_src1,
                          hwaddr phys_src2, void *host_src2,
                          dram_cpu_info *info, uint64_t *size,
                          void (*fn)(void *dest, void *src1, void *src2, uint64_t sz))
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

    for(int i=0; i < sz; i++){
        // if(i == 0)
        //     printf("%x & %x = %x\n", s1[i], s2[i], (s1[i]) & (s2[i]));
        d[i] = s1[i] & s2[i];
    }
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
        // if(i == 0)
        //     printf("%x | %x = %x\n", s1[i], s2[i], (s1[i]) | (s2[i]));
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
#endif


// ########################## RCC ###############################
#if !defined(CONFIG_USER_ONLY)
static RISCVAccess rcc_src_access, rcc_dest_access;
static int rcc_mmu_idx;
static bool rcc_faulted_all_src = false, rcc_faulted_all_dest = false;
static TCGMemOpIdx rcc_oi;
static rcc_stats rcc_stat;
static int rcc_missed_src[100];
static int n_rcc_missed_src;
static int rcc_missed_dest[100];
static int n_rcc_missed_dest;

static void perform_rcc_op(partial_row_list *partial_src,
                               partial_row_list *partial_dest)
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

        memcpy(dest_part->host, src_part->host, mov_size);

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

static uint64_t perform_rcc_delay(CPURISCVState *env,
                                  row_data_list *row_src,
                                  row_data_list *row_dest,
                                  dram_cpu_info *info)
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
            delay = fpm_psm_delay(env, row_size, src_row->addr, dest_row->addr, &rcc_stat);

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
#endif

void helper_rcc(CPURISCVState *env, target_ulong src,
                        target_ulong dest, target_ulong size)
{
    if(size == 0)
        return;

#if defined(CONFIG_USER_ONLY)
    void *src_addr;
    void *dest_addr;

    src_addr = g2h(src);
    dest_addr = g2h(dest);
    memcpy(dest_addr, src_addr, size);
#else

    dram_cpu_info *info;
    row_data_list row_src, row_dest;
    partial_row_list partial_src, partial_dest;
    uint64_t delay_op = 0;

    /* Only init once */
    if(rcc_src_access.pages == NULL) {
        rcc_mmu_idx = cpu_mmu_index(env, false);
        rcc_oi = make_memop_idx(MO_UB, rcc_mmu_idx);

        init_riscv_access(&rcc_src_access, src, size);
        init_riscv_access(&rcc_dest_access, dest, size);
    }

    info =  &(RISCV_CPU(env_cpu(env))->dram_info);

    /* Page fault if needed, but find all pages. Code until here
     * can re-executed becauses find_all_pages triggers a pf. */
    
    /* Pre-fault all src pages once */
    if(rcc_faulted_all_src == false) {
        rcc_stat.general.avg_pf.sum++;
        find_all_pages(env, &rcc_src_access, MMU_DATA_LOAD, info, rcc_mmu_idx, rcc_oi, GETPC());
        rcc_faulted_all_src = true;
    }

    /* Pre-fault all dest pages once */
    if(rcc_faulted_all_dest == false) {
        rcc_stat.general.avg_pf.sum++;
        find_all_pages(env, &rcc_dest_access, MMU_DATA_STORE, info, rcc_mmu_idx, rcc_oi, GETPC());
        rcc_faulted_all_dest = true;
    }

    n_rcc_missed_src = init_rowlist(env, info, &rcc_src_access, &row_src, &partial_src, rcc_missed_src);
    n_rcc_missed_dest = init_rowlist(env, info, &rcc_dest_access, &row_dest, &partial_dest, rcc_missed_dest);

    rcc_stat.general.tot_bytes += size;

    /* Core operation, perform the actual copy */
    perform_rcc_op(&partial_src, &partial_dest);
    delay_op = perform_rcc_delay(env, &row_src, &row_dest, info);

    /* missed ones, should be done manually */
    g_assert(n_rcc_missed_src == 0 && n_rcc_missed_dest == 0);

    /* Final stats bookeeping and cleanup */
    rcc_stat.general.avg_delay.sum += delay_op;
#if USE_STDEV_STDERR
    g_assert(rcc_stat.general.avg_delay.n < STAT_MAX_DEL_VAL);
    rcc_stat.general.del_val[rcc_stat.general.avg_delay.n] = delay_op;
#endif
    rcc_stat.general.avg_delay.n++;
    rcc_stat.general.avg_pf.sum -= 2;
    rcc_stat.general.avg_pf.n++;

    del_rowlist(&row_src);
    del_rowlist(&row_dest);
    del_partiallist(&partial_src);
    del_partiallist(&partial_dest);
    del_riscv_access(&rcc_src_access);
    del_riscv_access(&rcc_dest_access);

    rcc_faulted_all_dest = false;
    rcc_faulted_all_src = false;
    debug_printf("#######################\n");

#endif
}


// ########################## RCI ###############################
#if !defined(CONFIG_USER_ONLY)
static RISCVAccess rci_access;
static int rci_mmu_idx;
static TCGMemOpIdx rci_oi;
static rci_stats rci_stat;
static int rci_missed[100];
static int n_rci_missed = 0;

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
            rci_stat.in_pim += row_size;

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
                rci_access.pages[ttmp->page_parent].size -= ttmp->size;
            }
        }

        slow_down_by(env, delay);
        delay_op += delay;
    }

    return delay_op;
}

static uint64_t perform_rci_missed(CPURISCVState *env)
{
    uint64_t delay_op = 0;

    /* rci_missed pages (the ones that have to be set manually) */
    for (int i=0; i < n_rci_missed; i++) {
        rci_stat.other += rci_access.pages[rci_missed[i]].size;
        slow_down_by(env, rci_access.pages[rci_missed[i]].size);
        delay_op += rci_access.pages[rci_missed[i]].size;

        /* No luck, do it manually */
        for (int j = 0; j < rci_access.pages[rci_missed[i]].size; j++) {
            helper_ret_stb_mmu(env, rci_access.pages[rci_missed[i]].v_addr + j, MEMSET_BYTE, rci_oi, GETPC());
        }
    }

    return delay_op;
}
#else 

static void perform_rci_user(target_ulong dest, target_ulong size)
{
    void *dest_addr;
    dest_addr = g2h(dest);
    memset(dest_addr, MEMSET_BYTE, size);
}
#endif

void helper_rci(CPURISCVState *env, target_ulong dest,
                        target_ulong size)
{
    if(size == 0)
        return;

#if defined(CONFIG_USER_ONLY)
    /* User: no MMU, just copy the data from one side to the other */
    perform_rci_user(dest, size);
#else
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
    if(rci_access.pages == NULL) {
        rci_mmu_idx = cpu_mmu_index(env, false);
        rci_oi = make_memop_idx(MO_UB, rci_mmu_idx);
        init_riscv_access(&rci_access, dest, size);
    }

    info =  &(RISCV_CPU(env_cpu(env))->dram_info);
    row_size = info->col.size;

    /* Page fault if needed, but find all pages. Code until here
     * can re-executed becauses find_all_pages triggers a pf. */
    rci_stat.avg_pf.sum++;
    find_all_pages(env, &rci_access, MMU_DATA_STORE, info, rci_mmu_idx, rci_oi, GETPC());

    /* pre-init the zero rowbuffer*/
    init_zero_row(row_size);

    /* Stats bookeeping */
    rci_stat.tot_bytes += size;

    /* parse pages in row and partial rows */
    n_rci_missed = init_rowlist(env, info, &rci_access, &rows, &partial_rows, rci_missed);

    /* Core operation, perform the actual memset */
    delay_op = perform_rci(env, &rows, info);
    delay_op += perform_rci_missed(env);

    /* Final stats, and cleanup */
    rci_stat.avg_delay.sum += delay_op;
#if USE_STDEV_STDERR
    g_assert(rci_stat.avg_delay.n < STAT_MAX_DEL_VAL);
    rci_stat.del_val[rci_stat.avg_delay.n] = delay_op;
#endif
    rci_stat.avg_delay.n++;
    rci_stat.avg_pf.sum--;
    rci_stat.avg_pf.n++;

    del_rowlist(&rows);
    del_partiallist(&partial_rows);
    del_riscv_access(&rci_access);

    debug_printf("#######################\n");
#endif
}


// ########################## RCIK ###############################
#if !defined(CONFIG_USER_ONLY)
static RISCVAccess rcik_access;
static int rcik_mmu_idx;
static TCGMemOpIdx rcik_oi;
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

    /* Only init once */
    if(rcik_access.pages == NULL) {
        // printf("RCIK request\n");
        rcik_mmu_idx = cpu_mmu_index(env, false);
        rcik_oi = make_memop_idx(MO_UB, rcik_mmu_idx);
        init_riscv_access(&rcik_access, row_dest, 1);
    }

    info =  &(RISCV_CPU(env_cpu(env))->dram_info);
    row_size = info->col.size;

    init_zero_row(row_size);

    /* Page fault if needed, but find all pages. Code until here
     * can re-executed becauses find_all_pages triggers a pf. */
    rcik_stat.avg_pf.sum++;
    rcik_access.size = 1;

    find_all_pages(env, &rcik_access, MMU_DATA_STORE, info, rcik_mmu_idx, rcik_oi, GETPC());
    // printf("Phys addr %lx, Vaddr %lx\n", rcik_access.pages[0].phys_addr, (uint64_t) rcik_access.pages[0].v_addr);
    g_assert(rcik_access.n_pages == 1);

    // if the row is unaligned, error. Keep the instrucion as simple as poss
    offset_row = rcik_access.pages[0].phys_addr & info->col.mask;
    g_assert(offset_row == 0);

    /* Stats bookeeping */
    rcik_stat.tot_bytes += row_size;

    set_to_row(zero_row, rcik_access.pages[0].phys_addr, rcik_access.pages[0].host_addr, info, row_size);

    // printf("Full size, full speed\n");
    delay = FPM_DELAY(row_size);
    rcik_stat.in_pim += row_size;
    slow_down_by(env, delay);

    /* Final stats, and cleanup */
    rcik_stat.avg_delay.sum += delay;
#if USE_STDEV_STDERR
    g_assert(rcik_stat.avg_delay.n < STAT_MAX_DEL_VAL);
    rcik_stat.del_val[rcik_stat.avg_delay.n] = delay;
#endif
    rcik_stat.avg_delay.n++;
    rcik_stat.avg_pf.sum--;
    rcik_stat.avg_pf.n++;

    del_riscv_access(&rcik_access);
#endif
}


// ########################## RCCK ###############################
#if !defined(CONFIG_USER_ONLY)
static RISCVAccess rcck_src_access, rcck_dest_access;
static int rcck_mmu_idx;
static rcc_stats rcck_stat;
static bool rcck_faulted_all_src = false, rcck_faulted_all_dest = false;
static TCGMemOpIdx rcck_oi;

static void helper_src_dest(CPURISCVState *env, target_ulong src, target_ulong dest, uintptr_t curr_pc, row_src_dest row_fn)
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
    if(rcck_src_access.pages == NULL) {
        // printf("###############\nRCCK request %lx %lx\n", (uint64_t) src, (uint64_t) dest);
        rcck_mmu_idx = cpu_mmu_index(env, false);
        rcck_oi = make_memop_idx(MO_UB, rcck_mmu_idx);

        init_riscv_access(&rcck_src_access, src, 1);
        init_riscv_access(&rcck_dest_access, dest, 1);

        rcck_src_access.size = 1;
        rcck_dest_access.size = 1;
    }

    /* Page fault if needed, but find all pages. Code until here
     * can re-executed becauses find_all_pages triggers a pf. */

    /* Pre-fault all src pages once */
    if(rcck_faulted_all_src == false) {
        rcck_stat.general.avg_pf.sum++;
        find_all_pages(env, &rcck_src_access, MMU_DATA_LOAD, info, rcck_mmu_idx, rcck_oi, curr_pc);
        rcck_faulted_all_src = true;
    }

    /* Pre-fault all dest pages once */
    if(rcck_faulted_all_dest == false) {
        rcck_stat.general.avg_pf.sum++;
        find_all_pages(env, &rcck_dest_access, MMU_DATA_STORE, info, rcck_mmu_idx, rcck_oi, curr_pc);
        rcck_faulted_all_dest = true;
    }

    rcck_stat.general.tot_bytes += row_size;

    hwaddr phys_src = rcck_src_access.pages[0].phys_addr;
    hwaddr phys_dest = rcck_dest_access.pages[0].phys_addr;
    hwaddr offset_src, offset_dest;

    offset_src = phys_src & info->col.mask;
    offset_dest = phys_dest & info->col.mask;
    g_assert(offset_src == 0);
    g_assert(offset_dest == 0);

    /* Core operation, perform the actual copy */
    row_fn(phys_src,
            rcck_src_access.pages[0].host_addr,
            phys_dest,
            rcck_dest_access.pages[0].host_addr, info, row_size);

    /* Calculate the actual delay */
    delay_op = fpm_psm_delay(env, row_size, phys_src, phys_dest, &rcck_stat);

    /* Final stats bookeeping and cleanup */
    rcck_stat.general.avg_delay.sum += delay_op;
#if USE_STDEV_STDERR
    g_assert(rcck_stat.general.avg_delay.n < STAT_MAX_DEL_VAL);
    rcck_stat.general.del_val[rcck_stat.general.avg_delay.n] = delay_op;
#endif
    rcck_stat.general.avg_delay.n++;
    rcck_stat.general.avg_pf.sum -= 2;
    rcck_stat.general.avg_pf.n++;

    del_riscv_access(&rcck_src_access);
    del_riscv_access(&rcck_dest_access);

    rcck_faulted_all_dest = false;
    rcck_faulted_all_src = false;
    debug_printf("#######################\n");
}
#endif

void helper_rcck(CPURISCVState *env, target_ulong src, target_ulong dest)
{
#if !defined(CONFIG_USER_ONLY)
    helper_src_dest(env, src, dest, GETPC(), row_memcpy);
#endif
}

void helper_anot(CPURISCVState *env, target_ulong src, target_ulong dest)
{
#if !defined(CONFIG_USER_ONLY)
    // printf("helper_anot\n");
    helper_src_dest(env, src, dest, GETPC(), row_not);
#endif
}

// ########################## AMBIT OR/AND ###############################
#if !defined(CONFIG_USER_ONLY)
static RISCVAccess ambit_src1_access, ambit_src2_access, ambit_dest_access;
static int ambit_mmu_idx;
static rcc_stats ambit_stat;
static bool ambit_faulted_all_src1 = false,  ambit_faulted_all_src2 = false, ambit_faulted_all_dest = false;
static TCGMemOpIdx ambit_oi;

static void helper_ambit(CPURISCVState *env, target_ulong src1, target_ulong src2, target_ulong dest, uintptr_t curr_pc, row_ambit row_fn)
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
    if(ambit_src1_access.pages == NULL) {
        ambit_mmu_idx = cpu_mmu_index(env, false);
        ambit_oi = make_memop_idx(MO_UB, ambit_mmu_idx);

        init_riscv_access(&ambit_src1_access, src1, 1);
        init_riscv_access(&ambit_src2_access, src2, 1);
        init_riscv_access(&ambit_dest_access, dest, 1);

        ambit_src1_access.size = 1;
        ambit_src2_access.size = 1;
        ambit_dest_access.size = 1;
    }

    /* Page fault if needed, but find all pages. Code until here
     * can re-executed becauses find_all_pages triggers a pf. */

    /* Pre-fault all src pages once */
    if(ambit_faulted_all_src1 == false) {
        ambit_stat.general.avg_pf.sum++;
        find_all_pages(env, &ambit_src1_access, MMU_DATA_LOAD, info, ambit_mmu_idx, ambit_oi, curr_pc);
        ambit_faulted_all_src1 = true;
    }

    if(ambit_faulted_all_src2 == false) {
        ambit_stat.general.avg_pf.sum++;
        find_all_pages(env, &ambit_src2_access, MMU_DATA_LOAD, info, ambit_mmu_idx, ambit_oi, curr_pc);
        ambit_faulted_all_src2 = true;
    }

    /* Pre-fault all dest pages once */
    if(ambit_faulted_all_dest == false) {
        ambit_stat.general.avg_pf.sum++;
        find_all_pages(env, &ambit_dest_access, MMU_DATA_STORE, info, ambit_mmu_idx, ambit_oi, curr_pc);
        ambit_faulted_all_dest = true;
    }

    ambit_stat.general.tot_bytes += row_size;

    hwaddr phys_src1 = ambit_src1_access.pages[0].phys_addr;
    hwaddr phys_src2 = ambit_src2_access.pages[0].phys_addr;
    hwaddr phys_dest = ambit_dest_access.pages[0].phys_addr;
    hwaddr offset_src1, offset_src2, offset_dest;

    offset_src1 = phys_src1 & info->col.mask;
    offset_src2 = phys_src2 & info->col.mask;
    offset_dest = phys_dest & info->col.mask;

    if (offset_src1 != 0 || offset_src2 != 0 || offset_dest != 0) {
        riscv_raise_exception(env, RISCV_EXCP_INST_ACCESS_FAULT, curr_pc);
    }

    /* Core operation, perform the actual copy */
    row_fn(phys_src1,
           ambit_src1_access.pages[0].host_addr,
           phys_src2,
           ambit_src2_access.pages[0].host_addr,
           phys_dest,
           ambit_dest_access.pages[0].host_addr, info, row_size);

    /* Calculate the actual delay */
    delay_op = tri_fpm_psm_delay(env, row_size, phys_src1, phys_src2, phys_dest, &ambit_stat);

    /* Final stats bookeeping and cleanup */
    ambit_stat.general.avg_delay.sum += delay_op;
#if USE_STDEV_STDERR
    g_assert(ambit_stat.general.avg_delay.n < STAT_MAX_DEL_VAL);
    ambit_stat.general.del_val[ambit_stat.general.avg_delay.n] = delay_op;
#endif
    ambit_stat.general.avg_delay.n++;
    ambit_stat.general.avg_pf.sum -= 3;
    ambit_stat.general.avg_pf.n++;

    del_riscv_access(&ambit_src1_access);
    del_riscv_access(&ambit_src2_access);
    del_riscv_access(&ambit_dest_access);

    ambit_faulted_all_dest = false;
    ambit_faulted_all_src1 = false;
    ambit_faulted_all_src2 = false;
    debug_printf("#######################\n");

}

#endif

void helper_aand(CPURISCVState *env, target_ulong src1, target_ulong src2, target_ulong dest)
{
#if !defined(CONFIG_USER_ONLY)
    helper_ambit(env, src1, src2, dest, GETPC(), row_and);
#endif
}

void helper_aor(CPURISCVState *env, target_ulong src1, target_ulong src2, target_ulong dest)
{
#if !defined(CONFIG_USER_ONLY)
    helper_ambit(env, src1, src2, dest, GETPC(), row_or);
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

#endif

static void helper_rci_stat(CPURISCVState *env, bool k)
{
#if !defined(CONFIG_USER_ONLY)
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
#endif
}

static void helper_rcc_stat(CPURISCVState *env, bool k)
{
#if !defined(CONFIG_USER_ONLY)
    const char *name = "RCC";
    const char *name2 = "RCCK";
    double stddev = 0, stdderr = 0;
    rcc_stats *stat = &rcc_stat;

    if(k){
        name = name2;
        stat = &rcck_stat;
    }

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
#endif
}

void helper_stat(CPURISCVState *env, target_ulong val, target_ulong name)
{

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
        helper_rcc_stat(env, false);
        break;
    case 2:
        helper_rci_stat(env, true);
        break;
    case 3:
        helper_rcc_stat(env, true);
        break;
    
    default:
        printf("helper_stat: Command %ld not recognized!\n",(uint64_t) val);
        break;
    }
}

#if !defined(CONFIG_USER_ONLY)

static void helper_stat_i(target_ulong val)
{
    rcik_stat.tot_bytes += val;
    rcik_stat.avg_delay.sum += CPU_DELAY(val);
    rcik_stat.avg_delay.n++;
}

static void helper_stat_c(target_ulong val)
{
    rcck_stat.general.tot_bytes += val;
    rcck_stat.general.avg_delay.sum += CPU_DELAY(val);
    rcck_stat.general.avg_delay.n++;
}

static void helper_stat_a(target_ulong val)
{
    ambit_stat.general.tot_bytes += val;
    ambit_stat.general.avg_delay.sum += CPU_DELAY(val);
    ambit_stat.general.avg_delay.n++;
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
            helper_stat_c(val);
            break;
        case 2:
            helper_stat_a(val);
            break;
    }
#endif
}