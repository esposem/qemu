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

typedef struct QXlnx_zcu102Machine QXlnx_zcu102Machine;

struct QXlnx_zcu102Machine {
    QOSGraphObject obj;
    QGuestAllocator *alloc;
    QSDHCI_MemoryMapped sdhci;
};

static void xlnx_zcu102_destructor(QOSGraphObject *obj)
{
    g_free(obj);
}

static void *xlnx_zcu102_get_driver(void *object, const char *interface)
{
    QXlnx_zcu102Machine *machine = object;
    if (!g_strcmp0(interface, "memory")) {
        return &machine->alloc;
    }

    fprintf(stderr, "%s not present in aarch64/xlnx-zcu102\n", interface);
    g_assert_not_reached();
}

static QOSGraphObject *xlnx_zcu102_get_device(void *obj, const char *device)
{
    QXlnx_zcu102Machine *machine = obj;
    if (!g_strcmp0(device, "generic-sdhci")) {
        return &machine->sdhci.obj;
    }

    fprintf(stderr, "%s not present in aarch64/xlnx-zcu102\n", device);
    g_assert_not_reached();
}

static void *qos_create_machine_aarch64_xlnx_zcu102(void)
{
    QXlnx_zcu102Machine *machine = g_new0(QXlnx_zcu102Machine, 1);

    machine->obj.get_device = xlnx_zcu102_get_device;
    machine->obj.get_driver = xlnx_zcu102_get_driver;
    machine->obj.destructor = xlnx_zcu102_destructor;
    /* Datasheet: UG1085 (v1.7) */
    qos_init_sdhci_smm(&machine->sdhci, 0xff160000, &(QSDHCIProperties) {
        .version = 3,
        .baseclock = 0,
        .capab.sdma = true,
        .capab.reg = 0x280737ec6481
    });
    return &machine->obj;
}

static void xlnx_zcu102_register_nodes(void)
{
    qos_node_create_machine("aarch64/xlnx-zcu102", qos_create_machine_aarch64_xlnx_zcu102);
    qos_node_contains("aarch64/xlnx-zcu102", "generic-sdhci", NULL);
}

libqos_init(xlnx_zcu102_register_nodes);
