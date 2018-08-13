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

typedef struct QSabreliteMachine QSabreliteMachine;

struct QSabreliteMachine {
    QOSGraphObject obj;
    QGuestAllocator *alloc;
    QSDHCI_MemoryMapped sdhci;
};

static void sabrelite_destructor(QOSGraphObject *obj)
{
    g_free(obj);
}

static void *sabrelite_get_driver(void *object, const char *interface)
{
    QSabreliteMachine *machine = object;
    if (!g_strcmp0(interface, "memory")) {
        return &machine->alloc;
    }

    fprintf(stderr, "%s not present in arm/sabrelite\n", interface);
    g_assert_not_reached();
}

static QOSGraphObject *sabrelite_get_device(void *obj, const char *device)
{
    QSabreliteMachine *machine = obj;
    if (!g_strcmp0(device, "generic-sdhci")) {
        return &machine->sdhci.obj;
    }

    fprintf(stderr, "%s not present in arm/sabrelite\n", device);
    g_assert_not_reached();
}

static void *qos_create_machine_arm_sabrelite(void)
{
    QSabreliteMachine *machine = g_new0(QSabreliteMachine, 1);

    machine->obj.get_device = sabrelite_get_device;
    machine->obj.get_driver = sabrelite_get_driver;
    machine->obj.destructor = sabrelite_destructor;
    qos_init_sdhci_smm(&machine->sdhci, 0x02190000, &(QSDHCIProperties) {
        .version = 3,
        .baseclock = 0,
        .capab.sdma = true,
        .capab.reg = 0x057834b4,
    });
    return &machine->obj;
}

static void sabrelite_register_nodes(void)
{
    qos_node_create_machine("arm/sabrelite", qos_create_machine_arm_sabrelite);
    qos_node_contains("arm/sabrelite", "generic-sdhci", NULL);
}

libqos_init(sabrelite_register_nodes);
