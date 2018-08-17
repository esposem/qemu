/*
 * QTest testcase for e1000 NIC
 *
 * Copyright (c) 2013-2014 SUSE LINUX Products GmbH
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "libqos/qgraph.h"

typedef struct QE1000 QE1000;

struct QE1000 {
    QOSGraphObject obj;
};

static const char *models[] = {
    "e1000",
    "e1000-82540em",
    "e1000-82544gc",
    "e1000-82545em",
};

/* Tests only initialization so far. TODO: Replace with functional tests */
static void nop(void *obj, void *data, QGuestAllocator *alloc)
{
}

static void e1000_destructor(QOSGraphObject *obj)
{
    QE1000 *e1000 = (QE1000 *)obj;
    g_free(e1000);
}

static void *e1000_create(void *pci_bus, QGuestAllocator *alloc, void *addr)
{
    QE1000 *e1000 = g_new0(QE1000, 1);
    e1000->obj.destructor = e1000_destructor;

    return &e1000->obj;
}

static void e1000_register_nodes(void)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(models); i++) {
        qos_node_create_driver(models[i], e1000_create);
        qos_node_consumes(models[i], "pci-bus", NULL);
    }
}

libqos_init(e1000_register_nodes);

static void register_e1000_test(void)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(models); i++) {
        char *path;

        path = g_strdup_printf("%s-e1000-test", models[i]);
        qos_add_test(path, models[i], nop, NULL);
        g_free(path);
    }
}

libqos_init(register_e1000_test);
