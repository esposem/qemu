/*
 * QTest testcase for eepro100 NIC
 *
 * Copyright (c) 2013-2014 SUSE LINUX Products GmbH
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "libqos/qgraph.h"

typedef struct QEepro100 QEepro100;

struct QEepro100 {
    QOSGraphObject obj;
};

static const char *models[] = {
    "i82550",
    "i82551",
    "i82557a",
    "i82557b",
    "i82557c",
    "i82558a",
    "i82558b",
    "i82559a",
    "i82559b",
    "i82559c",
    "i82559er",
    "i82562",
    "i82801",
};

/* Tests only initialization so far. TODO: Replace with functional tests */
static void nop(void *obj, void *data, QGuestAllocator *alloc)
{
}

static void eepro100_destructor(QOSGraphObject *obj)
{
    QEepro100 *eepro100 = (QEepro100 *)obj;
    g_free(eepro100);
}

static void *eepro100_create(void *pci_bus, QGuestAllocator *alloc, void *addr)
{
    QEepro100 *eepro100 = g_new0(QEepro100, 1);
    eepro100->obj.destructor = eepro100_destructor;

    return &eepro100->obj;
}

static void eepro100_register_nodes(void)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(models); i++) {
        qos_node_create_driver(models[i], eepro100_create);
        qos_node_consumes(models[i], "pci-bus", NULL);
    }
}

libqos_init(eepro100_register_nodes);

static void register_eepro100_test(void)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(models); i++) {
        char *path;

        path = g_strdup_printf("%s-eepro100-test", models[i]);
        qos_add_test(path, models[i], nop, NULL);
        g_free(path);
    }
}

libqos_init(register_eepro100_test);
