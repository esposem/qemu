/*
 * QTest testcase for ES1370
 *
 * Copyright (c) 2014 SUSE LINUX Products GmbH
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "libqos/qgraph.h"

typedef struct QES1370 QES1370;

struct QES1370 {
    QOSGraphObject obj;
};

/* Tests only initialization so far. TODO: Replace with functional tests */
static void nop(void *obj, void *data, QGuestAllocator *alloc)
{
}

static void es1370_destructor(QOSGraphObject *obj)
{
    QES1370 *es1370 = (QES1370 *)obj;
    g_free(es1370);
}

static void *es1370_create(void *pci_bus, QGuestAllocator *alloc, void *addr)
{
    QES1370 *es1370 = g_new0(QES1370, 1);
    es1370->obj.destructor = es1370_destructor;

    return &es1370->obj;
}

static void es1370_register_nodes(void)
{
    qos_node_create_driver("ES1370", es1370_create);
    qos_node_consumes("ES1370", "pci-bus", NULL);
}

libqos_init(es1370_register_nodes);

static void register_es1370_test(void)
{
    qos_add_test("es1370-test", "ES1370", nop, NULL);
}

libqos_init(register_es1370_test);
