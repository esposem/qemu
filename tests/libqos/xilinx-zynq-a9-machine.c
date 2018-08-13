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
#include "sdhci.h"

typedef struct QXilinx_zynq_a9Machine QXilinx_zynq_a9Machine;

struct QXilinx_zynq_a9Machine {
    QOSGraphObject obj;
    QGuestAllocator *alloc;
    QSDHCI_MemoryMapped sdhci;
};

static void xilinx_zynq_a9_destructor(QOSGraphObject *obj)
{
    g_free(obj);
}

static void *xilinx_zynq_a9_get_driver(void *object, const char *interface)
{
    QXilinx_zynq_a9Machine *machine = object;
    if (!g_strcmp0(interface, "memory")) {
        return &machine->alloc;
    }

    fprintf(stderr, "%s not present in arm/xilinx-zynq-a9\n", interface);
    g_assert_not_reached();
}

static QOSGraphObject *xilinx_zynq_a9_get_device(void *obj, const char *device)
{
    QXilinx_zynq_a9Machine *machine = obj;
    if (!g_strcmp0(device, "generic-sdhci")) {
        return &machine->sdhci.obj;
    }

    fprintf(stderr, "%s not present in arm/xilinx-zynq-a9\n", device);
    g_assert_not_reached();
}

static void *qos_create_machine_arm_xilinx_zynq_a9(void)
{
    QXilinx_zynq_a9Machine *machine = g_new0(QXilinx_zynq_a9Machine, 1);

    machine->obj.get_device = xilinx_zynq_a9_get_device;
    machine->obj.get_driver = xilinx_zynq_a9_get_driver;
    machine->obj.destructor = xilinx_zynq_a9_destructor;
    /* Datasheet: UG585 (v1.12.1) */
    qos_init_sdhci_smm(&machine->sdhci, 0xe0100000, &(QSDHCIProperties) {
        .version = 2,
        .baseclock = 0,
        .capab.sdma = true,
        .capab.reg = 0x69ec0080,
    });
    return &machine->obj;
}

static void xilinx_zynq_a9_register_nodes(void)
{
    qos_node_create_machine("arm/xilinx-zynq-a9",
                            qos_create_machine_arm_xilinx_zynq_a9);
    qos_node_contains("arm/xilinx-zynq-a9", "generic-sdhci", NULL);
}

libqos_init(xilinx_zynq_a9_register_nodes);
