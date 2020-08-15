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

#define TYPE_PCI_PIM_DEVICE "pim"
#define PIM(obj)        OBJECT_CHECK(PIMState, obj, TYPE_PCI_PIM_DEVICE)

#define FACT_IRQ        0x00000001
#define DMA_IRQ         0x00000100

#define DMA_START       0x40000
#define DMA_SIZE        4096
// #define DMA_SIZE        (1 << 30)

typedef struct {
    PCIDevice pdev;
    MemoryRegion mmio;
    MemoryRegion ram;

    QemuThread thread;
    QemuMutex thr_mutex;
    QemuCond thr_cond;
    bool stopping;

    uint32_t addr4;
    uint32_t fact;
#define EDU_STATUS_COMPUTING    0x01
#define EDU_STATUS_IRQFACT      0x80
    uint32_t status;

    uint32_t irq_status;

#define EDU_DMA_RUN             0x1
#define EDU_DMA_DIR(cmd)        (((cmd) & 0x2) >> 1)
# define EDU_DMA_FROM_PCI       0
# define EDU_DMA_TO_PCI         1
#define EDU_DMA_IRQ             0x4
    struct dma_state {
        dma_addr_t src;
        dma_addr_t dst;
        dma_addr_t cnt;
        dma_addr_t cmd;
    } dma;
    QEMUTimer dma_timer;
    char *dma_buf;
    uint64_t dma_mask;
} PIMState;

MemoryRegion dev_memory;
// AddressSpace dev_addr_space;

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
    return start <= addr && addr <= end; // TODO: the <= end instead of < correct?
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

static dma_addr_t edu_clamp_addr(const PIMState *edu, dma_addr_t addr)
{
    // dma_addr_t res = addr & edu->dma_mask;
    dma_addr_t res = addr;

    if (addr != res) {
        printf("EDU: clamping DMA %#.16"PRIx64" to %#.16"PRIx64"!\n", addr, res);
    }

    return res;
}

static void edu_dma_timer(void *opaque)
{
    PIMState *edu = opaque;
    int err;

    if (!(edu->dma.cmd & EDU_DMA_RUN)) {
        return;
    }

    if (EDU_DMA_DIR(edu->dma.cmd) == EDU_DMA_FROM_PCI) {
        // printf("MEM -> DEV\n");

        uint64_t dst = edu->dma.dst;
        edu_check_range(dst, edu->dma.cnt, DMA_START, DMA_SIZE);
        dst -= DMA_START;

        err = pci_dma_read(&edu->pdev, edu_clamp_addr(edu, edu->dma.src),
                edu->dma_buf + dst, edu->dma.cnt);

        if(err != 0){
            perror("pci_dma_read");
            printf("ERROR %d\n", err);
        }

        // printf("Read src:%lx dest:%lx %lx, size:%lx\n", edu_clamp_addr(edu, edu->dma.src),(dma_addr_t) edu->dma_buf + dst, (dma_addr_t)edu->dma_buf, edu->dma.cnt);

    } else {
        // printf("DEV -> MEM\n");

        uint64_t src = edu->dma.src;
        src -= DMA_START;

        uint64_t tot_size = edu->dma.cnt;

        while(tot_size > 0) {

            edu->dma.cnt = MIN(DMA_SIZE, tot_size);

            err = pci_dma_write(&edu->pdev, edu_clamp_addr(edu, edu->dma.dst),
                edu->dma_buf + src, edu->dma.cnt);

            if(err != 0){
                perror("pci_dma_write");
                printf("ERROR %d\n", err);
            }

            edu->dma.dst += DMA_SIZE;
            tot_size -= edu->dma.cnt;
        }

    }

    edu->dma.cmd &= ~EDU_DMA_RUN;
    if (edu->dma.cmd & EDU_DMA_IRQ) {
        edu_raise_irq(edu, DMA_IRQ);
    }
}

static void dma_rw(PIMState *edu, bool write, dma_addr_t *val, dma_addr_t *dma,
                bool timer)
{
    if (write && (edu->dma.cmd & EDU_DMA_RUN)) {
        printf("DMA_RW RETURN\n");
        return;
    }

    if (write) {
        *dma = *val;
    } else {
        *val = *dma;
    }

    if (timer) {
        // edu_dma_timer(edu);
        timer_mod(&edu->dma_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL));
    }
}

static uint64_t edu_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    PIMState *edu = opaque;
    uint64_t val = ~0ULL;

    if (addr < 0x80 && size != 4) {
        return val;
    }

    if (addr >= 0x80 && addr <= 0x98 && size != 4 && size != 8) {
        return val;
    }

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
            val = edu->fact;
            // printf("Read 0x08 (fact), return %lx\n", val);
            qemu_mutex_unlock(&edu->thr_mutex);
            break;
        case 0x20:
            val = atomic_read(&edu->status);
            // printf("Read 0x20, return %lx. If 1, factorial, if 0, IR finished factorial\n", val);
            break;
        case 0x24:
            val = edu->irq_status;
            break;
        case 0x80:
            dma_rw(edu, false, &val, &edu->dma.src, false);
            // printf("Read 0x80, (src DMA) return %lx\n", val);
            break;
        case 0x88:
            dma_rw(edu, false, &val, &edu->dma.dst, false);
            // printf("Read 0x88, (dest DMA) return %lx\n", val);
            break;
        case 0x90:
            dma_rw(edu, false, &val, &edu->dma.cnt, false);
            // printf("Read 0x90, (size DMA) return %lx\n", val);
            break;
        case 0x98: /* This is to read the value of the register, NOT THE DMA */
            dma_rw(edu, false, &val, &edu->dma.cmd, false);
            // printf("Read 0x98, (cmd DMA) return %lx. pos 1, start, pos 2 dir, pos 4 IR enable\n", val);
            break;
    }

    if(addr >= 0x110 && addr <= (0x110 + DMA_SIZE - sizeof(uint64_t))){
            val = *((uint64_t *) &edu->dma_buf[addr - 0x110]);
            // printf("Read 0x%lx, (read buff) return buffer %lu\n", addr, val);
    }

    return val;
}

static void edu_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                unsigned size)
{
    PIMState *edu = opaque;

    if (addr < 0x80 && size != 4) {
        return;
    }

    if (addr >= 0x80 && size != 4 && size != 8) {
        printf("Size %u not accepted for addr >= 0x80\n", size);
        return;
    }

    switch (addr) {
    case 0x04:
        edu->addr4 = ~val;
        // printf("Write 0x04 (negation), set %x\n", edu->addr4);

        break;
    case 0x08:
        if (atomic_read(&edu->status) & EDU_STATUS_COMPUTING) {
            break;
        }
        /* EDU_STATUS_COMPUTING cannot go 0->1 concurrently, because it is only
         * set in this function and it is under the iothread mutex.
         */
        qemu_mutex_lock(&edu->thr_mutex);
        edu->fact = val;
        atomic_or(&edu->status, EDU_STATUS_COMPUTING);
        qemu_cond_signal(&edu->thr_cond);
        qemu_mutex_unlock(&edu->thr_mutex);
        // printf("Write 0x08 (fact), set %lx, status OR with COMPUTING\n", val);

        break;
    case 0x20:
        if (val & EDU_STATUS_IRQFACT) {
            atomic_or(&edu->status, EDU_STATUS_IRQFACT);
        } else {
            atomic_and(&edu->status, ~EDU_STATUS_IRQFACT);
        }
        // printf("Write 0x20, set status %x\n", edu->status);

        break;
    case 0x60:
        edu_raise_irq(edu, val);
        // printf("Write 0x60, edu raise irq\n");
        break;
    case 0x64:
        edu_lower_irq(edu, val);
        // printf("Write 0x64, edu lower irq\n");
        break;
    case 0x80:
        // printf("Write 0x80, (src DMA) set %lx size %u\n", val, size);
        dma_rw(edu, true, &val, &edu->dma.src, false);

        break;
    case 0x88:
        dma_rw(edu, true, &val, &edu->dma.dst, false);
        // printf("Write 0x88, (dest DMA) set %lx\n", val);

        break;
    case 0x90:
        dma_rw(edu, true, &val, &edu->dma.cnt, false);
        // printf("Write 0x90, (size DMA) set %lx\n", val);

        break;
    case 0x98:
        if (!(val & EDU_DMA_RUN)) {
            break;
        }
        dma_rw(edu, true, &val, &edu->dma.cmd, true);
        // printf("Write 0x98, (cmd DMA) set %lx. pos 1, start, pos 2 dir, pos 4 IR enable\n", val);

        break;
    case 0x110:
        // printf("Write 0x111, (memset) set %lx\n", val);

        if(val != 0x07){ // start | edu->ram | irq
            printf("Value has to be 0x07, it's %lx\n", val);
            val = 0x07;
        }

        dma_rw(edu, true, &val, &edu->dma.cmd, true);
        break;
    case 0x118:
        memset(edu->dma_buf, val, DMA_SIZE);
        // printf("Buffer initialized to %lu\n", val);
        break;

    case 0x120: {
            // This seems to work/
            char buff[5];

            // memset(buff, 'b', 5);
            // address_space_write(&dev_addr_space, 0x0, MEMTXATTRS_UNSPECIFIED, buff, 5);
            // memset(buff, 0, 5);
            // address_space_read(&dev_addr_space, 0x0, MEMTXATTRS_UNSPECIFIED, buff, 5);
            // printf("Buffer %.5s\n", buff); // reads PIM memory?


            memset(buff, 0, 5);
            int res = pci_dma_read(&edu->pdev, 0x1c0000000, buff, 5);
            if(res){
                perror("pci_dma_read");
                printf("ERROR %d\n", res);
            }
            printf("Buffer %x\n", buff[0]); // reads system ram

            memset(buff, 'a', 5);
            res = pci_dma_write(&edu->pdev, 0x1c0000000, buff, 5);
            if(res){
                perror("pci_dma_write");
                printf("ERROR %d\n", res);
            }
            printf("WRITE Buffer OK\n"); // reads system ram

            memset(buff, 0, 5);
            res = pci_dma_read(&edu->pdev, 0x1c0000000, buff, 5);
            if(res){
                perror("pci_dma_read");
                printf("ERROR %d\n", res);

            }
            printf("Buffer %x\n", buff[0]); // reads system ram

            break;
        }
    }


}

__attribute__ ((unused))
static const MemoryRegionOps edu_mmio_ops = {
    .read = edu_mmio_read,
    .write = edu_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },

};

/*
 * We purposely use a thread, so that users are forced to wait for the status
 * register.
 */
static void *edu_fact_thread(void *opaque)
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

        val = edu->fact;
        qemu_mutex_unlock(&edu->thr_mutex);

        while (val > 0) {
            ret += val--;
        }

        qemu_mutex_lock(&edu->thr_mutex);
        edu->fact = ret;
        // printf("\nFact set %d\n", ret);
        qemu_mutex_unlock(&edu->thr_mutex);
        atomic_and(&edu->status, ~EDU_STATUS_COMPUTING);

        if (atomic_read(&edu->status) & EDU_STATUS_IRQFACT) {
            qemu_mutex_lock_iothread();
            edu_raise_irq(edu, FACT_IRQ);
            // printf("\tIRQ raised\n");
            qemu_mutex_unlock_iothread();
            atomic_and(&edu->status, ~EDU_STATUS_IRQFACT);
        }
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

    timer_init_ms(&edu->dma_timer, QEMU_CLOCK_VIRTUAL, edu_dma_timer, edu);

    qemu_mutex_init(&edu->thr_mutex);
    qemu_cond_init(&edu->thr_cond);
    qemu_thread_create(&edu->thread, "edu", edu_fact_thread,
                       edu, QEMU_THREAD_JOINABLE);

    memory_region_init(&dev_memory, OBJECT(edu), "pim_memory", 1 * GiB);
    memory_region_set_enabled(&dev_memory, true);
    // address_space_init(&dev_addr_space, &dev_memory, "p_memory");

    memory_region_init_io(&edu->mmio, OBJECT(edu), &edu_mmio_ops, edu,
                    "edu-mmio", 1 * MiB);

    Error *err = NULL;
    memory_region_init_ram(&edu->ram, OBJECT(edu), "edu-ram", 1 * GiB, &err);

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
    qemu_cond_signal(&edu->thr_cond);
    qemu_thread_join(&edu->thread);

    qemu_cond_destroy(&edu->thr_cond);
    qemu_mutex_destroy(&edu->thr_mutex);

    timer_del(&edu->dma_timer);
    msi_uninit(pdev);
}

static void edu_instance_init(Object *obj)
{
    PIMState *edu = PIM(obj);
    printf("Edu device loaded\n");

    edu->dma_buf = malloc(DMA_SIZE);
    memset(edu->dma_buf, 0x0, DMA_SIZE);

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
    k->device_id = 0x11e8;
    k->revision = 0x10;
    k->class_id = PCI_CLASS_OTHERS;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static void edu_instance_uninit(Object *obj)
{
    PIMState *edu = PIM(obj);
    printf("Uninit PIM\n");
    free(edu->dma_buf);
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
