/*
 * QEMU educational PCI device
 *
 * Copyright (c) 2012-2015 Jiri Slaby
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "hw/pci/pci.h"
#include "hw/hw.h"
#include "hw/pci/msi.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h" /* iothread mutex */
#include "qemu/module.h"
#include "qapi/visitor.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/qdev-clock.h"


#define TYPE_PCI_PIM_DEVICE "pim"
#define PIM(obj)        OBJECT_CHECK(PIMState, obj, TYPE_PCI_PIM_DEVICE)
#define EDU_DEVICE_ID 0x11e8

#define SUMM_IRQ        0x00000001
#define DMA_IRQ         0x00000100
#define LOCAL_DMA_IRQ   0x00000101

#define DMA_START       0x40000
#define DMA_SIZE        16384 /* IMPORTANT: update also pim.h and libpim.c*/

#define LOCAL_MEM_SIZE     (1*GiB)
#define LOCAL_DMA_START    0x50000
#define LOCAL_DMA_SIZE     16384 /* IMPORTANT: update also pim.h and libpim.c*/

#define COPY_CMD 0x1


struct dma_state {
    dma_addr_t src;
    dma_addr_t dst;
    dma_addr_t cnt;
    dma_addr_t cmd;
} dma;

typedef struct {
    PCIDevice pdev;
    MemoryRegion mmio;
    MemoryRegion ram;

    QemuThread thread;
    QemuMutex thr_mutex;
    QemuCond thr_cond;
    bool stopping;

    QemuThread dma_thread;
    QemuMutex dma_mutex;
    QemuCond dma_cond;
    bool dma_stopping;

    uint32_t addr4;
    uint32_t summ;
#define EDU_STATUS_COMPUTING    0x01
#define EDU_STATUS_IRQSUMM      0x80
    uint32_t status;
    struct dma_state dma;
    struct dma_state local_dma;
    uint32_t irq_status;

#define EDU_DMA_RUN             0x1
#define EDU_DMA_DIR(cmd)        (cmd & 0x1)
#define EDU_DMA_FROM_PCI        0
#define EDU_DMA_TO_PCI          1

    QEMUTimer *dma_timer;
    char *dma_buf;

    QEMUTimer local_timer;
    char *local_dma_buf;

    uint64_t dma_mask;
} PIMState;

static MemoryRegion dev_memory;
static AddressSpace dev_addr_space;
static bool dma_running, local_dma_running;

static bool edu_msi_enabled(PIMState *edu)
{
    return msi_enabled(&edu->pdev);
}

static void edu_raise_irq(PIMState *edu, uint32_t val)
{
    edu->irq_status |= val;
    if (edu->irq_status) {
        if (edu_msi_enabled(edu)) {
            msi_notify(&edu->pdev, 0);
        } else {
            pci_set_irq(&edu->pdev, 1);
        }
    }
}

static void edu_lower_irq(PIMState *edu, uint32_t val)
{
    edu->irq_status &= ~val;

    if (!edu->irq_status && !edu_msi_enabled(edu)) {
        pci_set_irq(&edu->pdev, 0);
    }
}

static bool within(uint64_t addr, uint64_t start, uint64_t end)
{
    return start <= addr && addr <= end;
}

static void edu_check_range(uint64_t addr, uint64_t size1, uint64_t start,
                uint64_t size2)
{
    uint64_t end1 = addr + size1;
    uint64_t end2 = start + size2;

    if (within(addr, start, end2) &&
            end1 > addr && within(end1, start, end2)) {
        return;
    }

    hw_error("EDU: DMA range 0x%016"PRIx64"-0x%016"PRIx64
             " out of bounds (0x%016"PRIx64"-0x%016"PRIx64")!",
            addr, end1 - 1, start, end2 - 1);
}

// static dma_addr_t edu_clamp_addr(const PIMState *edu, dma_addr_t addr)
// {
    // dma_addr_t res = addr & edu->dma_mask;
    // dma_addr_t res = addr;

    // if (addr != res) {
    //     printf("EDU: clamping DMA %#.16"PRIx64" to %#.16"PRIx64"!\n", addr, res);
    // }

    // return res;
// }

static void host_memory_copy(bool is_dma, struct dma_state *state, PIMState *edu)
{
    char *host_src, *host_dest;
    MemoryRegionSection section_src, section_dest;
    MemoryRegion *mr = &edu->ram;

    if (is_dma){
        // printf("Is a dma host-host\n");
        mr = get_system_memory();
    }

    section_src = memory_region_find(mr, state->src, state->cnt);
    section_dest = memory_region_find(mr, state->dst, state->cnt);

    if (section_src.size == 0 || section_dest.size == 0) {
        printf("host-host copy: no section found in src or dest\n");
    }


    host_src = (char *) memory_region_get_ram_ptr(section_src.mr);
    // printf("Host src is %.5s\n", host_src);
    host_src += section_src.offset_within_region;
    // printf("Host src off is %.5s\n", host_src);

    host_dest = (char *) memory_region_get_ram_ptr(section_dest.mr);
    // printf("Host dest is %.5s\n", host_dest);
    host_dest += section_dest.offset_within_region;
    // printf("Host dest off is %.5s\n", host_dest);

    // printf("Src hw addr is %lx\n", state->src);
    // printf("Dest hw addr is %lx\n", state->dst);

    // printf("Size of transfer %ld\n", state->cnt);
    // qemu_mutex_lock_iothread();
    memmove(host_dest, host_src, state->cnt);
    // printf("NOW Host dest off is %.5s\n", host_dest);

    // qemu_mutex_unlock_iothread();
}

static void pim_timer(void *opaque, uint64_t buff_start, uint64_t buff_size)
{
    PIMState *edu = opaque;
    bool use_dma = buff_start == DMA_START;
    int err;
    struct dma_state *state = &edu->dma;
    char *buff_used = edu->dma_buf;

    if(!use_dma) {
        state = &edu->local_dma;
        buff_used = edu->local_dma_buf;
    }

    if (EDU_DMA_DIR(state->cmd) == EDU_DMA_FROM_PCI) {
        // printf("MEM -> DEV\n");

        uint64_t dst = state->dst;
        // printf("Destionation is %lx [%lx, %lx]\n", dst, buff_start, buff_size);
        if(within(dst, buff_start, buff_start+buff_size)) { // dest is buffer-included
            // printf("Dest is buffer-included\n");
            edu_check_range(dst, state->cnt, buff_start, buff_size);
            dst -= buff_start;

            if (use_dma) {
                // printf("Use DMA\n");
                err = pci_dma_read(&edu->pdev, state->src,
                buff_used + dst, state->cnt);

                if(err != 0){
                    perror("pci_dma_read");
                    printf("ERROR %d\n", err);
                }
            } else {
                MemTxResult res;
                // printf("#### Use Local dest %lx src %lx cnt %lu\n", dst, state->src, state->cnt);
                res = address_space_read(&dev_addr_space, state->src, MEMTXATTRS_UNSPECIFIED, edu->local_dma_buf, state->cnt);

                if(res != MEMTX_OK){
                    printf("PIM Local memory read error!\n");
                }
            }

        } else {
            // printf("Use host\n");
            host_memory_copy(use_dma, state, edu);
        }

        // printf("Read src:%lx dest:%lx (base %lx), size:%lu\n", state->src,(dma_addr_t) buff_used + dst, (dma_addr_t)buff_used, state->cnt);

    } else {
        // printf("DEV -> MEM\n");

        uint64_t src = state->src;
        bool buffer_included = within(src, buff_start, buff_start+buff_size);

        // printf("Src is %lx [%lx, %lx]\n", src, buff_start, buff_size);
        // printf("Dest is %lx\n", state->dst);

        if(buffer_included) { // src is buffer_included
            src -= buff_start;
        }

        uint64_t tot_size = state->cnt;

        while(tot_size > 0) {

            state->cnt = MIN(buff_size, tot_size);

            if(buffer_included) {
                if(use_dma){
                // printf("Use DMA\n");
                    err = pci_dma_write(&edu->pdev, state->dst,
                        buff_used + src, state->cnt);

                    if(err != 0){
                        perror("pci_dma_write");
                        printf("ERROR %d\n", err);
                    }
                } else {
                    MemTxResult res;
                    // printf("Use Local\n");
                    res = address_space_write(&dev_addr_space, state->dst, MEMTXATTRS_UNSPECIFIED, edu->local_dma_buf, state->cnt);

                    if(res != MEMTX_OK){
                        printf("PIM Local memory read error!\n");
                    }
                }
            } else {
                // printf("Use host\n");
                host_memory_copy(use_dma, state, edu);
            }

            state->dst += buff_size;
            tot_size -= state->cnt;
        }

    }

    // if(use_dma)
    //     edu_raise_irq(edu, DMA_IRQ);
    // else
        // edu_raise_irq(edu, LOCAL_DMA_IRQ);
}


static void pim_dma_timer(void *opaque)
{
    // printf("virt %ld host %ld  virtrt %ld rt %ld\n", qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), qemu_clock_get_ms(QEMU_CLOCK_HOST), qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL_RT), qemu_clock_get_ms(QEMU_CLOCK_REALTIME));
    pim_timer(opaque, DMA_START, DMA_SIZE);
    atomic_set(&dma_running, false);
}

static void pim_test(void *opaque)
{
    PIMState *edu = opaque;
     atomic_set(&dma_running, true);
    qemu_cond_signal(&edu->dma_cond);
    qemu_mutex_unlock(&edu->dma_mutex);
}

static void pim_local_timer(void *opaque)
{
    pim_timer(opaque, LOCAL_DMA_START, LOCAL_DMA_SIZE);
    atomic_set(&local_dma_running, false);
}

static void dma_rw(PIMState *edu, bool write, dma_addr_t *val, dma_addr_t *dma,
                bool timer)
{
    if (write && atomic_read(&dma_running)) {
        printf("DMA_RW RETURN\n");
        return;
    }

    if (write) {
        *dma = *val;
    } else {
        *val = *dma;
    }

    if (timer) {
        atomic_set(&dma_running, true);
        // pim_dma_timer(edu);
        // timer_mod(edu->dma_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
        //                NANOSECONDS_PER_SECOND / 10);


        qemu_cond_signal(&edu->dma_cond);
        qemu_mutex_unlock(&edu->dma_mutex);

        // timer_mod(&edu->dma_timer, 10000000);
        // printf("virt %ld host %ld  virtrt %ld rt %ld\n", qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), qemu_clock_get_ms(QEMU_CLOCK_HOST), qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL_RT), qemu_clock_get_ms(QEMU_CLOCK_REALTIME));
    }
}

static void local_dma_rw(PIMState *edu, bool write, dma_addr_t *val,
                dma_addr_t *dma, bool timer)
{
    if (write && atomic_read(&local_dma_running)) {
        printf("LOCAL_DMA_RW RETURN\n");
        return;
    }

    if (write) {
        *dma = *val;
    } else {
        *val = *dma;
    }

    if (timer) {
        atomic_set(&local_dma_running, true);
        timer_mod(&edu->local_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL));
        // pim_local_timer(edu);
    }
}

static uint64_t edu_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    PIMState *edu = opaque;
    uint64_t val = ~0ULL;

    switch (addr) {
        case 0x00:
            val = 0x010000edu;
            // printf("Read 0x0, returning 0x010000edu\n");
            break;
        case 0x04:
            val = edu->addr4;
            // printf("Read 0x04 (negation), return %lx\n", val);
            break;
        case 0x08:
            qemu_mutex_lock(&edu->thr_mutex);
            val = edu->summ;
            // printf("Read 0x08 (summ), return %lx\n", val);
            qemu_mutex_unlock(&edu->thr_mutex);
            break;
        case 0x10:
            val = atomic_read(&edu->status);
            // printf("Read 0x20, return %lx. If 1, summation, if 0, IR finished summation\n", val);
            break;
        case 0x18:
            val = edu->irq_status;
            break;
    }

    if(addr >= 0x110 && addr <= (0x110 + DMA_SIZE - sizeof(uint32_t))){
            val = 0;
            memcpy(&val, &edu->dma_buf[addr - 0x110], size);
            // val = *((uint64_t *) &edu->dma_buf[addr - 0x110]);
            // printf("Read 0x%lx, (read buff) return buffer %lx\n", addr - 0x110, val);
    }

    if(addr >= 0x4110 && addr <= (0x4110 + LOCAL_DMA_SIZE - sizeof(uint32_t))){       val = 0;
            memcpy(&val, &edu->local_dma_buf[addr - 0x4110], size);
            // val = *((uint64_t *) &edu->local_dma_buf[addr - 0x4110]);
            // printf("Read 0x%lx, (read buff) return buffer %lx\n", addr - 0x4110, val);
    }

    return val;
}

static void edu_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                unsigned size)
{
    PIMState *edu = opaque;
    // printf("MMIO write %lx, size %u\n", addr, size);

    switch (addr) {
    case 0x04:
        edu->addr4 = ~val;
        // printf("Write 0x04 (negation), set %x\n", edu->addr4);
        break;
    case 0x08:
        if (atomic_read(&edu->status) & EDU_STATUS_COMPUTING) {
            printf("status is still running\n");
            break;
        }
        /* EDU_STATUS_COMPUTING cannot go 0->1 concurrently, because it is only
         * set in this function and it is under the iothread mutex.
         */
        qemu_mutex_lock(&edu->thr_mutex);
        edu->summ = val;
        atomic_or(&edu->status, EDU_STATUS_COMPUTING);
        qemu_cond_signal(&edu->thr_cond);
        qemu_mutex_unlock(&edu->thr_mutex);
        // printf("Write 0x08 (summ), set %lx, status OR with COMPUTING\n", val);
        break;
    case 0x10:
        if (val & EDU_STATUS_IRQSUMM) {
            atomic_or(&edu->status, EDU_STATUS_IRQSUMM);
        } else {
            atomic_and(&edu->status, ~EDU_STATUS_IRQSUMM);
        }
        // printf("Write 0x10, set status %x\n", edu->status);
        break;
    case 0x20:
        edu_raise_irq(edu, val);
        // printf("Write 0x60, edu raise irq\n");
        break;
    case 0x24:
        edu_lower_irq(edu, val);
        // printf("Write 0x64, edu lower irq\n");
        break;
    case 0x30:
        // printf("Write 0x80, (src DMA) set %lx size %u\n", val, size);
        dma_rw(edu, true, &val, &edu->dma.src, false);
        break;
    case 0x38:
        dma_rw(edu, true, &val, &edu->dma.dst, false);
        // printf("Write 0x88, (dest DMA) set %lx\n", val);
        break;
    case 0x40:
        dma_rw(edu, true, &val, &edu->dma.cnt, false);
        // printf("Write 0x90, (size DMA) set %lx\n", val);
        break;
    case 0x48:
        switch (val >> 1)
        {
        case COPY_CMD:
            // printf("Copy command issued, val %lx\n", val);
            dma_rw(edu, true, &val, &edu->dma.cmd, true);
            break;

        default:
            printf("Command %lu not recognized\n", val);
            break;
        }
        // printf("Write 0x98, (cmd DMA) set %lx. pos 1, start, pos 2 dir, pos 4 IR enable\n", val);
        break;
    case 0x6b:
        memset(edu->dma_buf, val, DMA_SIZE);
        // printf("Buffer initialized to %lu\n", val);
        break;
    case 0x70:
        // printf("Write 0x70, (src DMA) set %lx size %u\n", val, size);
        local_dma_rw(edu, true, &val, &edu->local_dma.src, false);
        break;
    case 0x78:
        // printf("Write 0x78, (dest DMA) set %lx size %u\n", val, size);
        local_dma_rw(edu, true, &val, &edu->local_dma.dst, false);
        break;
    case 0x80:
        // printf("Write 0x80, (size DMA) set %lx size %u\n", val, size);
        local_dma_rw(edu, true, &val, &edu->local_dma.cnt, false);
        break;
    case 0x88:
        switch (val >> 1)
        {
        case COPY_CMD:
            local_dma_rw(edu, true, &val, &edu->local_dma.cmd, true);
            break;

        default:
            printf("Command not recognized\n");
            break;
        }

        break;
    case 0x10b:
        memset(edu->local_dma_buf, val, LOCAL_DMA_SIZE);
        // printf("Buffer initialized to %lu\n", val);
        break;
    }

    if(addr >= 0x110 && addr <= (0x110 + DMA_SIZE - sizeof(uint32_t))){
            memcpy(&edu->dma_buf[addr - 0x110], &val, size);
            // *((uint64_t *) &edu->dma_buf[addr - 0x110]) = val;
            // printf("Write 0x%lx, (write buff) val %lx\n", addr, val);
    }

    if(addr >= 0x4110 && addr <= (0x4110 + LOCAL_DMA_SIZE - sizeof(uint32_t))){
            memcpy(&edu->local_dma_buf[addr - 0x4110], &val, size);
            // *((uint64_t *) &edu->local_dma_buf[addr - 0x4110]) = val;
            // printf("Write 0x%lx, (write buff) val %lx\n", addr, val);
    }

}

static const MemoryRegionOps edu_mmio_ops = {
    .read = edu_mmio_read,
    .write = edu_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },

};

/*
 * We purposely use a thread, so that users are forced to wait for the status
 * register.
 */
static void *edu_summ_thread(void *opaque)
{
    PIMState *edu = opaque;

    while (1) {
        uint32_t val, ret = 1;

        qemu_mutex_lock(&edu->thr_mutex);
        // printf("\tQEMU thread locked...\n");
        while ((atomic_read(&edu->status) & EDU_STATUS_COMPUTING) == 0 &&
                        !edu->stopping) {
            qemu_cond_wait(&edu->thr_cond, &edu->thr_mutex);
        }
        // printf("\tQEMU thread unlocked...\n");


        if (edu->stopping) {
            qemu_mutex_unlock(&edu->thr_mutex);
            break;
        }

        val = edu->summ;
        qemu_mutex_unlock(&edu->thr_mutex);

        while (val > 0) {
            ret += val--;
        }

        qemu_mutex_lock(&edu->thr_mutex);
        edu->summ = ret;
        // printf("\nSumm set %d\n", ret);
        qemu_mutex_unlock(&edu->thr_mutex);
        atomic_and(&edu->status, ~EDU_STATUS_COMPUTING);

        if (atomic_read(&edu->status) & EDU_STATUS_IRQSUMM) {
            atomic_and(&edu->status, ~EDU_STATUS_IRQSUMM);
            qemu_mutex_lock_iothread();
            edu_raise_irq(edu, SUMM_IRQ);
            // printf("\tIRQ raised\n");
            qemu_mutex_unlock_iothread();
        }
    }

    return NULL;
}

static void *edu_dma_thread(void *opaque)
{
    PIMState *edu = opaque;

    while (1) {

        qemu_mutex_lock(&edu->dma_mutex);
        // printf("\tQEMU thread locked...\n");
        while (!atomic_read(&dma_running) && !edu->dma_stopping) {
            qemu_cond_wait(&edu->dma_cond, &edu->dma_mutex);
        }
        // printf("\tQEMU thread unlocked...\n");


        if (edu->dma_stopping) {
            qemu_mutex_unlock(&edu->dma_mutex);
            break;
        }

        pim_dma_timer(opaque);

        qemu_mutex_unlock(&edu->dma_mutex);

        qemu_mutex_lock_iothread();
        edu_raise_irq(edu, DMA_IRQ);
        // printf("\tIRQ raised\n");
        qemu_mutex_unlock_iothread();
    }

    return NULL;
}

static void pci_edu_realize(PCIDevice *pdev, Error **errp)
{
    PIMState *edu = PIM(pdev);
    uint8_t *pci_conf = pdev->config;

    pci_config_set_interrupt_pin(pci_conf, 1);

    if (msi_init(pdev, 0, 1, true, false, errp)) {
        return;
    }

    // timer_init_ms(&edu->dma_timer, QEMU_CLOCK_VIRTUAL, pim_test, edu);
    edu->dma_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pim_test, edu);
    timer_init_ms(&edu->local_timer, QEMU_CLOCK_VIRTUAL, pim_local_timer, edu);

    qemu_mutex_init(&edu->thr_mutex);
    qemu_cond_init(&edu->thr_cond);
    qemu_thread_create(&edu->thread, "edu", edu_summ_thread,
                       edu, QEMU_THREAD_JOINABLE);


    qemu_mutex_init(&edu->dma_mutex);
    qemu_cond_init(&edu->dma_cond);
    qemu_thread_create(&edu->dma_thread, "edu_dma", edu_dma_thread,
                       edu, QEMU_THREAD_JOINABLE);

    memory_region_init(&dev_memory, OBJECT(edu), "pim_memory", LOCAL_MEM_SIZE);
    memory_region_set_enabled(&dev_memory, true);
    address_space_init(&dev_addr_space, &dev_memory, "pim_memory_as");

    memory_region_init_io(&edu->mmio, OBJECT(edu), &edu_mmio_ops, edu,
                    "edu-mmio", 1 * MiB);

    Error *err = NULL;
    memory_region_init_ram(&edu->ram, OBJECT(edu), "edu-ram", LOCAL_MEM_SIZE, &err);

    if(err) {
        error_report("%s", error_get_pretty(err));
        printf("ERROR MEMORY\n");
    }

    memory_region_add_subregion(&dev_memory, 0, &edu->ram);

    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64, &edu->mmio);

    pci_register_bar(pdev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64, &dev_memory);
}

static void pci_edu_uninit(PCIDevice *pdev)
{
    PIMState *edu = PIM(pdev);

    qemu_mutex_lock(&edu->thr_mutex);
    edu->stopping = true;
    qemu_mutex_unlock(&edu->thr_mutex);

    qemu_mutex_lock(&edu->dma_mutex);
    edu->dma_stopping = true;
    qemu_mutex_unlock(&edu->dma_mutex);

    qemu_cond_signal(&edu->thr_cond);
    qemu_thread_join(&edu->thread);

    qemu_cond_signal(&edu->dma_cond);
    qemu_thread_join(&edu->dma_thread);

    qemu_cond_destroy(&edu->thr_cond);
    qemu_mutex_destroy(&edu->thr_mutex);

    qemu_cond_destroy(&edu->dma_cond);
    qemu_mutex_destroy(&edu->dma_mutex);

    timer_del(edu->dma_timer);
    timer_del(&edu->local_timer);
    msi_uninit(pdev);
}

static void edu_instance_init(Object *obj)
{
    PIMState *edu = PIM(obj);
    printf("Edu device loaded\n");

    edu->dma_buf = malloc(DMA_SIZE);
    edu->local_dma_buf = malloc(LOCAL_DMA_SIZE);
    memset(edu->dma_buf, 0x0, DMA_SIZE);
    memset(edu->local_dma_buf, 0x0, LOCAL_DMA_SIZE);

    // edu->dma_mask = (1UL << 28) - 1;
    // object_property_add_uint64_ptr(obj, "dma_mask",
    //                                &edu->dma_mask, OBJ_PROP_FLAG_READWRITE);
}

static void edu_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = pci_edu_realize;
    k->exit = pci_edu_uninit;
    k->vendor_id = PCI_VENDOR_ID_QEMU;
    k->device_id = EDU_DEVICE_ID;
    k->revision = 0x0;
    k->class_id = PCI_CLASS_OTHERS;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static void edu_instance_uninit(Object *obj)
{
    PIMState *edu = PIM(obj);
    printf("Uninit PIM\n");
    free(edu->dma_buf);
    free(edu->local_dma_buf);
}

static void pci_edu_register_types(void)
{
    static InterfaceInfo interfaces[] = {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    };
    static const TypeInfo edu_info = {
        .name          = TYPE_PCI_PIM_DEVICE,
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(PIMState),
        .instance_init = edu_instance_init,
        .instance_finalize = edu_instance_uninit,
        .class_init    = edu_class_init,
        .interfaces = interfaces,
    };

    type_register_static(&edu_info);
}
type_init(pci_edu_register_types)
