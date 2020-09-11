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

#define DMA_START       0x40000
#define DMA_SIZE        16384 /* IMPORTANT: update also pim.h and libpim.c*/

#define COPY_CMD 0x1

#define EDU_STATUS_COMPUTING    0x01
#define EDU_STATUS_IRQSUMM      0x80
#define EDU_DMA_RUN             0x1
#define EDU_DMA_DIR(cmd)        (cmd & 0x1)
#define EDU_DMA_FROM_PCI        0
#define EDU_DMA_TO_PCI          1

#define N_DMA_THREADS 8

struct dma_state {
    dma_addr_t src;
    dma_addr_t dst;
    dma_addr_t cnt;
    dma_addr_t cmd;
};

struct dma_buff {
    char *dma_buf;
    uint64_t start_addr;
    uint64_t end_addr;
};

typedef struct PIMState PIMState;

typedef struct dma_thread_state {
    struct dma_state dma;
    uint64_t dma_offset; // used to repart workloads

    QemuThread dma_thread;
    QemuMutex dma_mutex;
    QemuCond dma_cond;
    bool dma_stopping;

    QEMUTimer dma_timer;
    struct dma_buff buffer;

    int tid;
    PIMState *pim;
} dma_thread_state;

struct PIMState{
    PCIDevice pdev;
    MemoryRegion mmio;

    uint32_t irq_status;
    struct dma_state input_state;

    dma_thread_state threads[N_DMA_THREADS];

    uint64_t dma_mask;

    bool dma_running;
    int threads_done;
    int threads_issued;
};

static void pim_timer(void *opaque);
static void *pim_dma_thread(void *opaque);

static void init_dma_thread_state(PIMState *pim, int tid)
{
    dma_thread_state *state = &pim->threads[tid];
    state->tid = tid;
    state->pim = pim;

    state->buffer.dma_buf = malloc(DMA_SIZE);
    if (!state->buffer.dma_buf) {
        printf("Error in allocating thread buffer");
    }
    memset(state->buffer.dma_buf, 0x0, DMA_SIZE);
    state->buffer.start_addr = DMA_START + (DMA_SIZE * tid);
    state->buffer.end_addr = state->buffer.start_addr + DMA_SIZE;

    qemu_mutex_init(&state->dma_mutex);
    qemu_cond_init(&state->dma_cond);

    char buff[200];
    sprintf(buff, "pim_dma_thread_%d", tid);

    qemu_thread_create(&state->dma_thread, buff, pim_dma_thread,
                       state, QEMU_THREAD_JOINABLE);

    atomic_set(&state->dma_stopping, false);

    timer_init_ms(&state->dma_timer, QEMU_CLOCK_VIRTUAL, pim_timer, state);
}

static void del_dma_thread_state(dma_thread_state *state)
{
    free(state->buffer.dma_buf);

    atomic_set(&state->dma_stopping, true);

    qemu_cond_signal(&state->dma_cond);
    qemu_thread_join(&state->dma_thread);

    qemu_cond_destroy(&state->dma_cond);
    qemu_mutex_destroy(&state->dma_mutex);

    timer_del(&state->dma_timer);
}

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

static void host_memory_copy(struct dma_state *state)
{
    char *host_src, *host_dest;
    MemoryRegionSection section_src, section_dest;
    MemoryRegion *mr = get_system_memory();

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

static void pim_timer(void *opaque)
{
    dma_thread_state *thr_state = opaque;
    int err;
    struct dma_state *state = &thr_state->dma;

    if (EDU_DMA_DIR(state->cmd) == EDU_DMA_FROM_PCI) {
        // printf("MEM -> DEV\n");
        state->src += thr_state->dma_offset;

        uint64_t dst = state->dst + (DMA_SIZE * thr_state->tid);
        // printf("Destionation is %lx [%lx, %lx]\n", dst, thr_state->buffer.start_addr, DMA_SIZE);
        if(within(dst, thr_state->buffer.start_addr,
                    thr_state->buffer.end_addr)) { // dest is buffer-included

            // printf("Dest is buffer-included\n");
            edu_check_range(dst, state->cnt, thr_state->buffer.start_addr, DMA_SIZE);
            dst -= thr_state->buffer.start_addr;

            // printf("Use DMA\n");
            err = pci_dma_read(&thr_state->pim->pdev, state->src,
            thr_state->buffer.dma_buf + dst, state->cnt);

            // err = dma_memory_rw_relaxed(pci_get_address_space(&thr_state->pim->pdev), state->src, thr_state->buffer.dma_buf + dst, state->cnt, DMA_DIRECTION_TO_DEVICE);

            if(err != 0){
                perror("pci_dma_read");
                printf("ERROR %d\n", err);
            }
        } else {
            // printf("Use host\n");
            host_memory_copy(state);
        }

        // printf("Read src:%lx dest:%lx (base %lx), size:%lu\n", state->src,(dma_addr_t) thr_state->dma_buf + dst, (dma_addr_t)thr_state->dma_buf, state->cnt);

    } else {
        // printf("DEV -> MEM\n");


        uint64_t src = state->src + (DMA_SIZE * thr_state->tid);
        // printf("-Src is %lx\n", src);
        // printf("-Dest is %lx\n", state->dst);
        state->dst += thr_state->dma_offset;

        bool buffer_included = within(src, thr_state->buffer.start_addr,
                                        thr_state->buffer.end_addr);

        if(buffer_included) { // src is buffer_included
            src -= thr_state->buffer.start_addr;
        }

        // printf("Src is %lx\n", src);
        // printf("Dest is %lx\n", state->dst);

        uint64_t tot_size = state->cnt;

        while(tot_size > 0) {

            state->cnt = MIN(DMA_SIZE, tot_size);

            if(buffer_included) {

                // int64_t a = get_clock_realtime();
                err = pci_dma_write(&thr_state->pim->pdev, state->dst,
                    thr_state->buffer.dma_buf + src, state->cnt);

                // int64_t b = get_clock_realtime();
                // printf("took %ld\n", ((b-a)));

                // err = dma_memory_rw_relaxed(pci_get_address_space(&thr_state->pim->pdev), state->dst, thr_state->buffer.dma_buf + src, state->cnt, DMA_DIRECTION_FROM_DEVICE);

                if(err != 0){
                    perror("pci_dma_write");
                    printf("ERROR %d\n", err);
                }

            } else {
                // printf("Use host\n");
                host_memory_copy(state);
            }

            state->dst += DMA_SIZE;
            tot_size -= state->cnt;
        }

    }

    int td = atomic_add_fetch(&thr_state->pim->threads_done, 1);

    if (td == thr_state->pim->threads_issued) {
        atomic_set(&thr_state->pim->threads_done, 0);
        // qemu_mutex_lock_iothread();
        edu_raise_irq(thr_state->pim, DMA_IRQ);
        // printf("\tIRQ raised by thread %d\n", thr_state->tid);
        // qemu_mutex_unlock_iothread();
        atomic_set(&thr_state->pim->dma_running, false);
    }
}


// static void pim_dma_timer(void *opaque)
// {
    // printf("virt %ld host %ld  virtrt %ld rt %ld\n", qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), qemu_clock_get_ms(QEMU_CLOCK_HOST), qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL_RT), qemu_clock_get_ms(QEMU_CLOCK_REALTIME));
//     pim_timer(opaque, DMA_START, DMA_SIZE);
//     atomic_set(&dma_running, false);
// }

// static bool irq_triggered = false;

// static void pim_test(void *opaque)
// {
//     PIMState *edu = opaque;
//     // atomic_set(&dma_running, true);
//     // qemu_cond_signal(&edu->dma_cond);
//     // qemu_mutex_unlock(&edu->dma_mutex);
//     // edu_raise_irq(edu, DMA_IRQ);
//     // printf("IRQ raise\n");
//     // atomic_set(&irq_triggered, true);
//     edu_raise_irq(edu, DMA_IRQ);
//     // printf("virt %ld host %ld  virtrt %ld rt %ld\n", qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), qemu_clock_get_ms(QEMU_CLOCK_HOST), qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL_RT), qemu_clock_get_ms(QEMU_CLOCK_REALTIME));
//     // irq_triggered = true;
// }


static void dma_rw(PIMState *pim, bool write, dma_addr_t *val,
                    dma_addr_t *dma, bool timer)
{
    if (write && atomic_read(&pim->dma_running)) {
        printf("DMA_RW RETURN\n");
        return;
    }

    if (write) {
        *dma = *val;
    } else {
        *val = *dma;
    }

    if (timer) {

        uint64_t th_op_size = pim->input_state.cnt / N_DMA_THREADS;
        uint64_t th_offset = pim->input_state.cnt % N_DMA_THREADS;
        uint64_t written = 0;
        if(th_op_size > 0)
            pim->threads_issued = N_DMA_THREADS;
        else
            pim->threads_issued = 1;

        atomic_set(&pim->dma_running, true);

        for (int i=0; i < N_DMA_THREADS; i++) {
            pim->threads[i].dma_offset = written;
            pim->threads[i].dma.cmd = pim->input_state.cmd;
            pim->threads[i].dma.src = pim->input_state.src;
            pim->threads[i].dma.dst = pim->input_state.dst;
            pim->threads[i].dma.cnt = th_op_size;
            if(i == (N_DMA_THREADS -1))
                pim->threads[i].dma.cnt += th_offset;

            written += th_op_size + th_offset;
            th_offset = 0;

            // printf("\tThread %d src:%lx dest:%lx, size:%lu\n", i,
            //         (dma_addr_t) pim->threads[i].dma.src,
            //         (dma_addr_t) pim->threads[i].dma.dst,
            //         pim->threads[i].dma.cnt);

            if(pim->threads[i].dma.cnt == 0){
                break;
            }

            timer_mod(&pim->threads[i].dma_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL));
            // qemu_cond_signal(&pim->threads[i].dma_cond);
            // qemu_mutex_unlock(&pim->threads[i].dma_mutex);
        }
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
        case 0x18:
            val = edu->irq_status;
            break;
    }

    if(addr >= 0x110 && addr <= (0x110 + (DMA_SIZE * N_DMA_THREADS) - size)){
            val = 0;
            addr -= 0x110;
            uint64_t tid = addr / DMA_SIZE;
            uint64_t off = addr % DMA_SIZE;
            memcpy(&val, &edu->threads[tid].buffer.dma_buf[off], size);
            // memcpy(&val, &edu->dma_buf[addr - 0x110], size);
            // val = *((uint64_t *) &edu->dma_buf[addr - 0x110]);
            // TODO: sbagliato come si legge il buffer se il valore e' minore
            printf("Read 0x%lx, (read buff) return buffer %lx\n", addr, val);
    }

    return val;
}

static void edu_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                unsigned size)
{
    PIMState *edu = opaque;
    // printf("MMIO write %lx, size %u\n", addr, size);

    switch (addr) {

    case 0x20:
        edu_raise_irq(edu, val);
        // printf("Write 0x60, edu raise irq\n");
        break;
    case 0x24:
        edu_lower_irq(edu, val);
        // printf("Write 0x64, edu lower irq\n");
        break;
    case 0x30:
        // printf("Write 0x80, (src DMA) set %lx\n", val);
        dma_rw(edu, true, &val, &edu->input_state.src, false);
        break;
    case 0x38:
        dma_rw(edu, true, &val, &edu->input_state.dst, false);
        // printf("Write 0x88, (dest DMA) set %lx\n", val);
        break;
    case 0x40:
        dma_rw(edu, true, &val, &edu->input_state.cnt, false);
        // printf("Write 0x90, (size DMA) set %lu\n", val);
        break;
    case 0x48:
        switch (val >> 1)
        {
        case COPY_CMD:
            // printf("Copy command issued, val %lx\n", val);
            dma_rw(edu, true, &val, &edu->input_state.cmd, true);
            break;

        default:
            printf("Command %lu not recognized\n", val);
            break;
        }
        // printf("Write 0x98, (cmd DMA) set %lx. pos 1, start, pos 2 dir, pos 4 IR enable\n", val);
        break;
    case 0x6b:
        for(int i=0; i < N_DMA_THREADS; i++){
            memset(edu->threads[i].buffer.dma_buf, val, DMA_SIZE);
        }
        // printf("Buffer initialized to %lu\n", val);
        break;
    }

    // if(addr == 0x110){
    //     static uint64_t delay = 1;
    //     // printf("Delay is %ld\n", delay);
    //     // atomic_set(&dma_running, true);
    //     // qemu_cond_signal(&edu->dma_cond);
    //     // qemu_mutex_unlock(&edu->dma_mutex);
    //     // printf("MOD virt %ld host %ld  virtrt %ld rt %ld\n", qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), qemu_clock_get_ms(QEMU_CLOCK_HOST), qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL_RT), qemu_clock_get_ms(QEMU_CLOCK_REALTIME));
    //     timer_mod(edu->dma_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + delay);
    //     // printf("Pending %d\n", timer_pending(edu->dma_timer));
    //     // sleep(1);
    //     // delay += 1000;
    //     return;
    // }

    if(addr >= 0x110 && addr <= (0x110 + (DMA_SIZE * N_DMA_THREADS) - size)){
            addr -= 0x110;
            uint64_t tid = addr / DMA_SIZE;
            uint64_t off = addr % DMA_SIZE;
            memcpy(&edu->threads[tid].buffer.dma_buf[off], &val, size);

            // memcpy(&edu->dma_buf[addr - 0x110], &val, size);
            // *((uint64_t *) &edu->dma_buf[addr - 0x110]) = val;
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

static void *pim_dma_thread(void *opaque)
{
    dma_thread_state *state = opaque;

    while (1) {

        qemu_mutex_lock(&state->dma_mutex);
        // printf("\tQEMU thread %d locked...\n", state->tid);

        do {
            qemu_cond_wait(&state->dma_cond, &state->dma_mutex);
        } while (!atomic_read(&state->pim->dma_running) &&
                 !atomic_read(&state->dma_stopping));

        // printf("\tQEMU thread %d unlocked...\n", state->tid);

        if (state->dma_stopping) {
            qemu_mutex_unlock(&state->dma_mutex);
            break;
        }

        pim_timer(opaque);

        qemu_mutex_unlock(&state->dma_mutex);

        int td = atomic_add_fetch(&state->pim->threads_done, 1);

        if (td == state->pim->threads_issued) {
            atomic_set(&state->pim->threads_done, 0);
            qemu_mutex_lock_iothread();
            edu_raise_irq(state->pim, DMA_IRQ);
            // printf("\tIRQ raised by thread %d\n", state->tid);
            qemu_mutex_unlock_iothread();
            atomic_set(&state->pim->dma_running, false);
        }

    }

    return NULL;
}

static void pci_edu_realize(PCIDevice *pdev, Error **errp)
{
    PIMState *pim = PIM(pdev);
    uint8_t *pci_conf = pdev->config;

    pci_config_set_interrupt_pin(pci_conf, 1);

    if (msi_init(pdev, 0, 1, true, false, errp)) {
        return;
    }

    for (int i=0; i < N_DMA_THREADS; i++) {
        init_dma_thread_state(pim, i);
    }

    memory_region_init_io(&pim->mmio, OBJECT(pim), &edu_mmio_ops, pim,
                            "pim-mmio", 1 * MiB);

    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64, &pim->mmio);
}

static void pci_edu_unrealize(PCIDevice *pdev)
{
    PIMState *pim = PIM(pdev);

    for(int i=0; i < N_DMA_THREADS; i++)
        del_dma_thread_state(&pim->threads[i]);

    msi_uninit(pdev);
}

static void edu_instance_init(Object *obj)
{
    PIMState *pim = PIM(obj);
    printf("PIM device loaded\n");

    pim->threads_done = 0;
    pim->dma_running = false;

    // edu->dma_buf = malloc(DMA_SIZE);
    // edu->local_dma_buf = malloc(LOCAL_DMA_SIZE);
    // memset(edu->dma_buf, 0x0, DMA_SIZE);
    // memset(edu->local_dma_buf, 0x0, LOCAL_DMA_SIZE);

    // edu->dma_mask = (1UL << 28) - 1;
    // object_property_add_uint64_ptr(obj, "dma_mask",
    //                                &edu->dma_mask, OBJ_PROP_FLAG_READWRITE);
}

static void edu_instance_uninit(Object *obj)
{
    // PIMState *pim = PIM(obj);
    // printf("Uninit PIM\n");
    // for(int i=0; i < N_DMA_THREADS; i++)
    //     del_dma_thread_state(&pim->threads[i]);
}

static void edu_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = pci_edu_realize;
    k->exit = pci_edu_unrealize;
    k->vendor_id = PCI_VENDOR_ID_QEMU;
    k->device_id = EDU_DEVICE_ID;
    k->revision = 0x0;
    k->class_id = PCI_CLASS_OTHERS;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
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
