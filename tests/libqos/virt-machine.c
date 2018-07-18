/*
 * libqos driver framework
 *
 * Copyright (c) 2018 Emanuele Giuseppe Esposito <e.emanuelegiuseppe@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License version 2 as published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "libqos/malloc.h"
#include "libqos/qgraph.h"
#include "libqos/virtio-mmio.h"
#include "libqos/malloc-generic.h"

#define ARM_PAGE_SIZE               4096
#define VIRTIO_MMIO_BASE_ADDR       0x0A003E00
#define ARM_VIRT_RAM_ADDR           0x40000000
#define ARM_VIRT_RAM_SIZE           0x20000000
#define VIRTIO_MMIO_SIZE            0x00000200

typedef struct QVirtioMachine QVirtioMachine;

struct QVirtioMachine {
    QOSGraphObject obj;
    QGuestAllocator *alloc;
    QVirtioMMIODevice virtio_mmio;
};

static void virtio_destroy(QOSGraphObject *obj)
{
    QVirtioMachine *machine = (QVirtioMachine *) obj;
    generic_alloc_uninit(machine->alloc);
    g_free(obj);
}

static void *virtio_get_driver(void *object, const char *interface)
{
    QVirtioMachine *machine = object;
    if (!g_strcmp0(interface, "guest_allocator")) {
        return machine->alloc;
    }

    printf("%s not present in arm/virtio\n", interface);
    abort();
}

static QOSGraphObject *virtio_get_device(void *obj, const char *device)
{
    QVirtioMachine *machine = obj;
    if (!g_strcmp0(device, "virtio-mmio")) {
        return &machine->virtio_mmio.obj;
    }

    printf("%s not present in arm/virtio\n", device);
    abort();
}

static void *qos_create_machine_arm_virt(void)
{
    QVirtioMachine *machine = g_new0(QVirtioMachine, 1);

    machine->alloc = generic_alloc_init(ARM_VIRT_RAM_ADDR, ARM_VIRT_RAM_SIZE,
                                        ARM_PAGE_SIZE);
    qvirtio_mmio_init_device(&machine->virtio_mmio, VIRTIO_MMIO_BASE_ADDR,
                             VIRTIO_MMIO_SIZE);

    machine->obj.get_device = virtio_get_device;
    machine->obj.get_driver = virtio_get_driver;
    machine->obj.destructor = virtio_destroy;
    return machine;
}

static void virtio_machine(void)
{
    qos_node_create_machine("arm/virt", qos_create_machine_arm_virt);
    qos_node_contains("arm/virt", "virtio-mmio", NULL);
}

libqos_init(virtio_machine);
