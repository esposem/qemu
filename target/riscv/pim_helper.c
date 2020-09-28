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
#define ENABLE_DELAY 1

#define CACHE_LINE_SIZE 64

#define DRAM_DEBUG 0

#if DRAM_DEBUG
#define  debug_printf(fmt, ...)  do { printf(fmt, ## __VA_ARGS__); }while(0);
#else
#define debug_printf(fmt, ...)    /* Do nothing */
#endif

#define CPU_DELAY(size) size
#define PSM_DELAY(size) (size / CACHE_LINE_SIZE)
#define FPM_DELAY(size) 0

#if !defined(CONFIG_USER_ONLY)

typedef struct DRAM_info {
    uint32_t channel;
    uint32_t rank;
    uint32_t row;
    uint32_t subarr;
    uint32_t bank;
    uint32_t col;
    hwaddr addr;
} DRAM_info;

static void dram_el_to_dram_info(dram_element_info *el, uint32_t *info, hwaddr addr)
{
    addr &= el->mask;
    *info = 0;
    hwaddr temp;
    for(int i=0; i < el->n_sections; i++) {
        if(el->offsets[i] > 0){
            temp = addr >> el->offsets[i];
        } else {
            temp = addr << el->offsets[i];
        }

        *info |= temp & ((1 << el->bits[i]) - 1);
    }
}

ATTRIBUTE_UNUSED
static int phys_to_dram_info(CPURISCVState *env, DRAM_info *info, hwaddr addr)
{
    if(addr == -1)
        return -1;

    info->addr = addr;

    dram_cpu_info *dinfo = &(RISCV_CPU(env_cpu(env))->dram_info);

    dram_el_to_dram_info(&dinfo->col, &info->col, addr);
    dram_el_to_dram_info(&dinfo->bank, &info->bank, addr);
    dram_el_to_dram_info(&dinfo->subarr, &info->subarr, addr);
    dram_el_to_dram_info(&dinfo->row, &info->row, addr);
    dram_el_to_dram_info(&dinfo->rank, &info->rank, addr);
    dram_el_to_dram_info(&dinfo->channel, &info->channel, addr);

    return 0;
}

ATTRIBUTE_UNUSED
static void print_dram_info(DRAM_info *info){
    printf("channel: %u\nrank: %u\nsubarr: %u\nrow: %u\nbank: %u\ncol: %u\n", info->channel, info->rank, info->subarr, info->row, info->bank, info->col);
}

ATTRIBUTE_UNUSED
static inline void slow_down_by(CPURISCVState *env, size_t delay)
{
#if ENABLE_DELAY
    if(delay == 0)
        return;

    qemu_mutex_lock(&env->op_mutex);
    timer_mod(env->op_timer, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) + delay);
    qemu_cond_wait(&env->op_cond, &env->op_mutex);
    qemu_mutex_unlock(&env->op_mutex);
#endif
}

/* An access can cover an arbitrary # of pages, so try to save them all */
typedef struct RISCVPage {
    void *host_addr;
    target_ulong v_addr;
    hwaddr phys_addr;
    target_ulong size;
} RISCVPage;

typedef struct RISCVAccess {
    RISCVPage *pages;
    target_ulong n_pages;
} RISCVAccess;

typedef struct row_pages {
    hwaddr addr_nocol;
    int n_pages;
    bool not_pim;
} row_pages;

typedef struct row_info {
    row_pages *rows;
    row_pages **rows_ref;
} row_info;

static void init_riscv_access(RISCVAccess *access, target_ulong size)
{
    uint64_t n_pages;

    n_pages = (size / TARGET_PAGE_SIZE) + 1;
    access->n_pages = n_pages;
    /* multiply by 2 because it can cross page boundary */
    access->pages = malloc(sizeof(RISCVPage) * n_pages * 2);
    g_assert(access->pages);
}

static void del_riscv_access(RISCVAccess *access)
{
    access->n_pages = 0;
    free(access->pages);
    access->pages = NULL;
}

static hwaddr get_nocol_mask(hwaddr phys, dram_cpu_info *info)
{
    return phys & (info->row.mask | info->bank.mask | info->channel.mask
                    | info->rank.mask | info->subarr.mask);
}

static void init_rowinfo(CPURISCVState *env, dram_cpu_info *info, row_info *row, RISCVAccess *access)
{
    // worst case: a page in each row
    int max_rows_here = access->n_pages;
    row->rows = malloc(sizeof(row_pages) * max_rows_here);
    row->rows_ref = malloc(sizeof(row_pages *) * max_rows_here);
    g_assert(row->rows);
    g_assert(row->rows_ref);

    memset(row->rows, 0, sizeof(row_pages) * max_rows_here);
    memset(row->rows_ref, 0, sizeof(row_pages *) * max_rows_here);

    int n_rows = 0;

    for (int i=0; i < access->n_pages; i++) {

        if (unlikely(!access->pages[i].host_addr)) {
            continue;
        }

#if DRAM_DEBUG
        uint32_t rrow;
        dram_cpu_info *info =  &(RISCV_CPU(env_cpu(env))->dram_info);
        dram_el_to_dram_info(&info->row, &rrow, access->pages[i].phys_addr);
        debug_printf("Row of page %d is %u\n", i, rrow);
#endif

        // & with all other fields because there can be
        // additional bits after MSB of the dram encoding
        hwaddr nocol = get_nocol_mask(access->pages[i].phys_addr, info);

        debug_printf("Nocol is %lx\n", nocol);

        bool found = false;
        int j=0;
        for(; j < n_rows; j++){
            if(row->rows[j].addr_nocol == nocol){
                found = true;
                break;
            }
        }

        if(!found) {
            j = n_rows;
            row->rows[j].addr_nocol = nocol;
            n_rows++;
        }

        // if size is not the full page, pim cannot be done
        // on a smaller-than-row size
        if((access->pages[i].size != TARGET_PAGE_SIZE)) {
            debug_printf("Page %d has not full page size (%lu)\n", i, (uint64_t)access->pages[i].size);
            row->rows[j].not_pim = true;
        }

        row->rows[j].n_pages++;
        g_assert(row->rows[j].n_pages <= env->pages_in_row);
        debug_printf("Row %u(%d) has %d pages\n----------------\n", rrow, j,row-> rows[j].n_pages);
        row->rows_ref[i] = &row->rows[j];
    }
}

static void del_rowinfo(row_info *row)
{
    free(row->rows);
    free(row->rows_ref);
}

static uint32_t find_all_pages(CPURISCVState *env, RISCVAccess *access,
                           target_ulong addr, target_ulong size,
                           int mmu_idx, int ra)
{
    uint32_t k = 0;
    target_ulong sz;
    // debug_printf("MAX Pages %ld\n", (uint64_t) n_pages *2);

    while (size > 0) {
        /* size that fits inside a page (taking into account offset) */
        sz = MIN(size, -(addr | TARGET_PAGE_MASK));

        /*
         * tlb_vaddr_to_host: tries to see in the TLB the hw address
         * (that corresponds to a host pointer).
         * It's a trapless lookup, so it might return NULL if it's not
         * found.
         */
        // debug_printf("####Address %lu size %lu\n", (uint64_t) addr, (uint64_t) sz);
        // access->pages[k].host_addr = probe_write(env, addr, sz, mmu_idx, GETPC());
        access->pages[k].host_addr = tlb_vaddr_to_host(env, addr, MMU_DATA_STORE, mmu_idx);
        access->pages[k].v_addr = addr;
        access->pages[k].size = sz;

        k++;
        size -= sz;
        addr += sz;
        // debug_printf("New size is %lu\n", (uint64_t) size);
        g_assert(k <= (access->n_pages * 2));
    }

    g_assert(size == 0);

    access->n_pages = k;

    return k;
}

static void find_all_phys_pages(CPURISCVState *env, RISCVAccess *access)
{
    CPUState *cpu = env_cpu(env);
    RISCVPage *page;
    hwaddr offset;
    target_ulong align;

    for(int i=0; i < access->n_pages; i++) {
        page = &access->pages[i];
        align = QEMU_ALIGN_DOWN(page->v_addr, TARGET_PAGE_SIZE);
        page->phys_addr = cpu_get_phys_page_debug(cpu, align);
        offset = page->v_addr - align;
        if(page->phys_addr != -1)
            page->phys_addr += offset;
    }
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
int rcc_i = 0;
bool rcc_faulted_all_src = false, rcc_faulted_all_dest = false;
static int rcc_mmu_idx;
static TCGMemOpIdx rcc_oi;

static hwaddr get_bank_mask(hwaddr phys, dram_cpu_info *info)
{
    return phys & (info->bank.mask | info->channel.mask | info->rank.mask);
}

static hwaddr get_subarray_mask(hwaddr phys, dram_cpu_info *info)
{
    return phys & (info->bank.mask | info->channel.mask | info->subarr.mask | info->rank.mask);
}

static void fpm_psm_delay(CPURISCVState *env, target_ulong size, hwaddr start, hwaddr end)
{
    dram_cpu_info *info =  &(RISCV_CPU(env_cpu(env))->dram_info);

    hwaddr subs = get_subarray_mask(start, info);
    hwaddr subd = get_subarray_mask(end, info);

    hwaddr banks = get_bank_mask(start, info);
    hwaddr bankd = get_bank_mask(end, info);

    debug_printf("Subarr src is %lx\n", subs);
    debug_printf("Subarr dest is %lx\n", subd);

    debug_printf("Bank src is %lx\n", banks);
    debug_printf("Bank dest is %lx\n", bankd);

    if(banks == bankd) { // same bank
        // if same subarray, FPM
        // if different, PSM * 2
        if(subs != subd) {
            debug_printf("Slowdown PSM 2\n");
            slow_down_by(env, PSM_DELAY(size) * 2);
        } else {
            slow_down_by(env, FPM_DELAY(size));
        }
    } else { // different bank, PSM
        debug_printf("Slowdown PSM\n");
        slow_down_by(env, PSM_DELAY(size));
    }
}

static int check_partial_row(CPURISCVState *env, row_pages *row)
{
    return !row || row->not_pim || row->n_pages < env->pages_in_row;
}

static void print_debug_partial_row(row_pages *row, int i)
{
    if(!row) {
        debug_printf("Page %d NULL (no row found), slowing\n", i);
    } else if (row->not_pim) {
        debug_printf("Page %d notpim (size is incomplete), slowing\n", i);
    } else {
        debug_printf("Page %d (row misses pages), slowing\n", i);
    }
}

static uint64_t calc_init_slow(uint64_t row_size, void *addr)
{
    return (row_size - (((uint64_t) addr) % row_size))  % row_size;
}

static uint64_t calc_full_fast(uint64_t row_size, target_ulong pgsize, uint64_t init)
{
    return (((uint64_t) pgsize) - init) / row_size;
}

static uint64_t calc_remaining_slow(uint64_t row_size, target_ulong pgsize, uint64_t init)
{
    return (((uint64_t) pgsize) - init) % row_size;
}

static void rcc_increment_indexes(target_ulong ss, target_ulong sd,
                                  int *s, int *d, target_ulong *off_ss,
                                  target_ulong *off_sd, RISCVPage *pages,
                                  RISCVPage *paged, target_ulong chosen)
{
    if(ss == sd) {
        (*s)++;
        (*d)++;
        (*off_sd) = 0;
        (*off_ss) = 0;
    } else if(chosen == ss){
        (*s)++;
        paged->size -= chosen;
        (*off_sd) = chosen;
        (*off_ss) = 0;
    } else {
        (*d)++;
        pages->size -= chosen;
        (*off_ss) = chosen;
        (*off_sd) = 0;
    }
}

static void rcc_row_gt_page(CPURISCVState *env, int s, int d, hwaddr pphys,
                            hwaddr pphyd, void *hosts, void *hostd,
                            row_info *row_src, row_info *row_dest,
                            target_ulong chosen)
{
    // row is bigger than page
    // same process as rci, but on both src and dest.
    // then, check that 1) row containing pages src dest
    // are in same subarray, and 2) check row is full.
    debug_printf("row > page\n");

    row_pages *row_s = row_src->rows_ref[s];
    row_pages *row_d = row_dest->rows_ref[d];

    // either not row or row not full
    if(check_partial_row(env, row_s) ||
        check_partial_row(env, row_d)) {

        print_debug_partial_row(row_s, s);
        print_debug_partial_row(row_d, d);

        slow_down_by(env, CPU_DELAY(chosen));
    } else {
        fpm_psm_delay(env, chosen, pphys, pphyd);
    }

    memmove(hostd, hosts, chosen);
}

static void rcc_row_lt_page(CPURISCVState *env, hwaddr pphys,
                            hwaddr pphyd, void *hosts, void *hostd,
                            target_ulong chosen, uint64_t row_size)
{
    // remove initial and end bytes that are slow
    // but only if in the same subarray
    debug_printf("row < page\n");

    // initial offset. If different for both
    // src and dest, there is no way we can do things faster
    uint64_t init_slow_s = calc_init_slow(row_size, hosts);
    uint64_t init_slow_d = calc_init_slow(row_size, hostd);

    // delete initial offset from size, but check now
    // that each full row is in the same subarray
    // middle is fast, done by PIM
    uint64_t full_rows = calc_full_fast(row_size, chosen, init_slow_s);

    // final offset
    // end is slow, done by CPU
    uint64_t remaining_slow =calc_remaining_slow(row_size, chosen, init_slow_s);

    if(init_slow_s != init_slow_d){
        // slow everything
        debug_printf("All are slow, init offset don't match\n");
        slow_down_by(env, CPU_DELAY(chosen));
        memmove(hostd, hosts, chosen);
        return;

    } else if(init_slow_s != 0) {
        // init is slow, done by CPU
        debug_printf("Initial %ld/%ld are slow\n", init_slow_s, (uint64_t) chosen);
        slow_down_by(env, CPU_DELAY(init_slow_s));
        memmove(hostd, hosts, init_slow_s);
    }

    debug_printf("Middle %ld/%ld are fast\n", full_rows *row_size, (uint64_t) chosen);

    pphys += init_slow_s;
    pphyd += init_slow_d;

    hosts += init_slow_s;
    hostd += init_slow_d;

    for(int i=0; i < full_rows; i++){
        fpm_psm_delay(env, chosen, pphys, pphyd);
        pphys += row_size;
        pphyd += row_size;

        memmove(hostd, hosts, row_size);

        hosts += row_size;
        hostd += row_size;
    }

    // final offset
    if(remaining_slow) {
        debug_printf("Final %ld/%ld are slow\n", remaining_slow, (uint64_t) chosen);
        slow_down_by(env, CPU_DELAY(remaining_slow));
        memmove(hostd, hosts, remaining_slow);
    }
}

static void rcc_row_eq_page(CPURISCVState *env, hwaddr pphys,
                            hwaddr pphyd, void *hosts, void *hostd,
                            target_ulong chosen, uint64_t row_size)
{
    debug_printf("row == page\n");

    // not row-aligned
    if(chosen != row_size){
        debug_printf("Chosen %lu bytes, less than %d, slowdown\n", (uint64_t) chosen, TARGET_PAGE_SIZE);
        slow_down_by(env, CPU_DELAY(chosen));
    } else {
        fpm_psm_delay(env, chosen, pphys, pphyd);
    }

    memmove(hostd, hosts, chosen);
}

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
    if(rcc_src_access.pages == NULL) {
        rcc_mmu_idx = cpu_mmu_index(env, false);
        rcc_oi = make_memop_idx(MO_UB, rcc_mmu_idx);


        init_riscv_access(&rcc_src_access, size);
        init_riscv_access(&rcc_dest_access, size);

        find_all_pages(env, &rcc_src_access, src, size, rcc_mmu_idx, GETPC());
        find_all_pages(env, &rcc_dest_access, dest, size, rcc_mmu_idx, GETPC());
    }

    /* Pre-fault all src pages once */
    if(rcc_faulted_all_src == false) {
        for(; rcc_i < rcc_src_access.n_pages; rcc_i++) {
            if(unlikely(rcc_src_access.pages[rcc_i].host_addr == NULL)) {
                /*
                 * Do a single access and test if we can then get access to the
                 * page. This is especially relevant to speed up TLB_NOTDIRTY.
                 */
                helper_ret_ldsb_mmu(env, rcc_src_access.pages[rcc_i].v_addr, rcc_oi, GETPC());

                rcc_src_access.pages[rcc_i].host_addr = tlb_vaddr_to_host(env,
                                                rcc_src_access.pages[rcc_i].v_addr,
                                                MMU_DATA_LOAD, rcc_mmu_idx);
            }
        }
        rcc_i = 0;
        rcc_faulted_all_src = true;
    }

    /* Pre-fault all dest pages once */
    if( rcc_faulted_all_dest == false) {
         for(; rcc_i < rcc_dest_access.n_pages; rcc_i++) {
            if(unlikely(rcc_dest_access.pages[rcc_i].host_addr == NULL)) {
                /*
                 * Do a single access and test if we can then get access to the
                 * page. This is especially relevant to speed up TLB_NOTDIRTY.
                 */
                helper_ret_stb_mmu(env, rcc_dest_access.pages[rcc_i].v_addr, 0, rcc_oi, GETPC());

                rcc_dest_access.pages[rcc_i].host_addr = tlb_vaddr_to_host(env,
                                                rcc_dest_access.pages[rcc_i].v_addr,
                                                MMU_DATA_STORE, rcc_mmu_idx);
            }
        }
        rcc_i = 0;
        rcc_faulted_all_dest = true;
    }

    find_all_phys_pages(env, &rcc_src_access);
    find_all_phys_pages(env, &rcc_dest_access);

    int s = 0, d = 0;
    target_ulong ss, sd, chosen, off_ss = 0, off_sd = 0;
    RISCVPage *pages, *paged;

    dram_cpu_info *info =  &(RISCV_CPU(env_cpu(env))->dram_info);
    uint64_t row_size = info->col.size;
    row_info row_src, row_dest;
    // debug_printf("Row size is %ld, page size is %d\n", row_size, TARGET_PAGE_SIZE);

    // if it's same subarray (rcc) or size == row size or two+pages == rowsize
    // or part_of_page == rowsize, do faster
    if (row_size > TARGET_PAGE_SIZE) {
        init_rowinfo(env, info, &row_src, &rcc_src_access);
        debug_printf("######################\n");
        init_rowinfo(env, info, &row_dest, &rcc_dest_access);
        debug_printf("######################\n");
    }

    while (size > 0) {

        pages = &(rcc_src_access.pages[s]);
        paged = &(rcc_dest_access.pages[d]);
        ss = pages->size;
        sd = paged->size;
        hwaddr pphys = pages->phys_addr + off_ss;
        hwaddr pphyd = paged->phys_addr + off_sd;
        void * hosts = pages->host_addr + off_ss;
        void * hostd = paged->host_addr + off_sd;

        chosen = MIN(ss, sd);

        debug_printf("src: page %d (%lx) size %lu offset %lu chosen %lu\n", s, pages->phys_addr,(uint64_t) ss, (uint64_t) off_ss, (uint64_t) chosen);
        debug_printf("dest: page %d (%lx) size %lu offset %lu chosen %lu\n", d, paged->phys_addr,(uint64_t) sd, (uint64_t) off_sd, (uint64_t) chosen);

        // a row can contain more NON contiguous pages of the same
        // array. If they all are there, and are in same subarray,
        // copy them
        if(likely(pages->host_addr) &&
           likely(paged->host_addr)) {

                debug_printf("Row size %ld, page size %d\n", row_size, TARGET_PAGE_SIZE);

                if(row_size > TARGET_PAGE_SIZE){
                    rcc_row_gt_page(env, s, d, pphys, pphyd, hosts, hostd, &row_src, &row_dest, chosen);
                } else if (row_size < TARGET_PAGE_SIZE) {
                   rcc_row_lt_page(env, pphys, pphyd, hosts, hostd, chosen, row_size);
                } else {
                   rcc_row_eq_page(env, pphys, pphyd, hosts, hostd, chosen, row_size);
               }

        } else {
            /* do it manually if there is no page */
            for(int i=0; i < chosen; i++){
                uint8_t byte;

                byte = helper_ret_ldsb_mmu(env, pages->v_addr + off_ss + i, rcc_oi, GETPC());

                helper_ret_stb_mmu(env, paged->v_addr + off_sd + i, byte, rcc_oi, GETPC());
            }
        }

        rcc_increment_indexes(ss, sd, &s, &d, &off_ss, &off_sd, pages, paged, chosen);

        size -= chosen;
    }

    if (row_size > TARGET_PAGE_SIZE) {
        del_rowinfo(&row_src);
        del_rowinfo(&row_dest);
    }

    printf("#######################\n");

    del_riscv_access(&rcc_src_access);
    del_riscv_access(&rcc_dest_access);

    rcc_faulted_all_dest = false;
    rcc_faulted_all_src = false;
#endif
    return old_size;
}




#if !defined(CONFIG_USER_ONLY)

static RISCVAccess rci_access;
static int rci_mmu_idx, rci_i = 0;
static bool rci_faulted_all = false;
static TCGMemOpIdx rci_oi;

static void rci_row_eq_page(CPURISCVState *env, int i, uint64_t row_size)
{

    debug_printf("Row size == TARGET logic\n");
    debug_printf("Access in page %d size %ld\n", i, (uint64_t) rci_access.pages[i].size);

    // less than a row, do it in CPU
    if(rci_access.pages[i].size != row_size){
        debug_printf("Access is not aligned to row/target size, slow\n");
        slow_down_by(env, CPU_DELAY(rci_access.pages[i].size));
    }

    debug_printf("Memset page %d\n", i);
    memset(rci_access.pages[i].host_addr, MEMSET_BYTE, rci_access.pages[i].size);
}

static void rci_row_lt_page(CPURISCVState *env, int i, uint64_t row_size)
{
    debug_printf("Row size < TARGET logic\n");

    // initial offset
    uint64_t init_slow = calc_init_slow(row_size, rci_access.pages->host_addr);

    // delete initial offset from size
    uint64_t full_rows = calc_full_fast(row_size, rci_access.pages[i].size, init_slow);

    // final offset
    uint64_t remaining_slow =calc_remaining_slow(row_size, rci_access.pages[i].size, init_slow);

    // init is slow, done by CPU
    if(init_slow){
        debug_printf("Initial %ld/%d are slow\n", init_slow, TARGET_PAGE_SIZE);
        slow_down_by(env, CPU_DELAY(init_slow));
        memset(rci_access.pages[i].host_addr, MEMSET_BYTE, init_slow);
    }

    // middle is fast, done by PIM
    if(full_rows) {
        debug_printf("Middle %ld/%d are fast\n", full_rows *row_size, TARGET_PAGE_SIZE);
        // done all at once
        memset(rci_access.pages[i].host_addr + init_slow, MEMSET_BYTE, full_rows *row_size);
    }

    // end is slow, done by CPU
    if(remaining_slow){
        debug_printf("Final %ld/%d are slow\n", remaining_slow, TARGET_PAGE_SIZE);
        slow_down_by(env, CPU_DELAY(remaining_slow));
        memset(rci_access.pages[i].host_addr + init_slow + (full_rows * row_size), MEMSET_BYTE, remaining_slow);
    }
}


static void rci_row_gt_page(CPURISCVState *env, int i, row_info *row)
{
    debug_printf("Row size > TARGET logic\n");

#if DRAM_DEBUG
    uint32_t rrow;
    dram_cpu_info *info =  &(RISCV_CPU(env_cpu(env))->dram_info);
    dram_el_to_dram_info(&info->row, &rrow, rci_access.pages[i].phys_addr);
    debug_printf("Row of page %d is %u\n", i, rrow);
#endif

    row_pages *page = row->rows_ref[i];

    // it's part of a full row, memset it one page at time
    // physical address (thus host virtual) could NOT be
    // contiguous for pages
    // in the same row, due to row interleaving.
    // so do it one at the time anyways
    if(check_partial_row(env, page)) {
        print_debug_partial_row(page, i);
        slow_down_by(env, CPU_DELAY(rci_access.pages[i].size));
    }

    memset(rci_access.pages[i].host_addr, MEMSET_BYTE, rci_access.pages[i].size);
}

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

    /* Only init once */
    if(rci_access.pages == NULL) {
        rci_mmu_idx = cpu_mmu_index(env, false);
        rci_oi = make_memop_idx(MO_UB, rci_mmu_idx);

        init_riscv_access(&rci_access, size);

        find_all_pages(env, &rci_access, dest, size, rci_mmu_idx, GETPC());
    }

    /* Pre-fault all pages once */
    if(rci_faulted_all == false) {
        for(; rci_i < rci_access.n_pages; rci_i++) {
            if(unlikely(rci_access.pages[rci_i].host_addr == NULL)) {
                /*
                 * Do a single access and test if we can then get access to the
                 * page. This is especially relevant to speed up TLB_NOTDIRTY.
                 */
                g_assert(rci_access.pages[rci_i].size > 0);

                helper_ret_stb_mmu(env, rci_access.pages[rci_i].v_addr, MEMSET_BYTE, rci_oi, GETPC());

                rci_access.pages[rci_i].host_addr = tlb_vaddr_to_host(env, rci_access.pages[rci_i].v_addr, MMU_DATA_STORE, rci_mmu_idx);
            }
        }
        rci_i = 0;
        rci_faulted_all = true;
    }

    find_all_phys_pages(env, &rci_access);

    dram_cpu_info *info =  &(RISCV_CPU(env_cpu(env))->dram_info);
    uint64_t row_size = info->col.size;
    // printf("Row size is %ld, page size is %d\n", row_size, TARGET_PAGE_SIZE);

    row_info row;

    if (row_size > TARGET_PAGE_SIZE) {
        debug_printf("Row size > TARGET logic\n");
        init_rowinfo(env,  info, &row, &rci_access);
    }

    for (int i=0; i < rci_access.n_pages; i++) {

        /* There is a page in TLB, just memset it */
        if (likely(rci_access.pages[i].host_addr)) {

            // we need more contiguous pages than this to get
            // a full row.
            if(row_size > TARGET_PAGE_SIZE){
                rci_row_gt_page(env, i, &row);
            }else if(row_size < TARGET_PAGE_SIZE) {
                rci_row_lt_page(env, i, row_size);
            } else { /* row is equal to a page size: */
                rci_row_eq_page(env, i, row_size);
            }

        } else {
            /* No luck, do it manually. Skip first byte bc already set */
            for (int j = 1; j < rci_access.pages[i].size; j++) {
                helper_ret_stb_mmu(env, rci_access.pages[i].v_addr + j, MEMSET_BYTE, rci_oi, GETPC());
            }
        }
    }

    if(row_size > TARGET_PAGE_SIZE){
        del_rowinfo(&row);
    }

    printf("#######################\n");

    del_riscv_access(&rci_access);
    rci_faulted_all = false;
#endif
    return old_size;
}