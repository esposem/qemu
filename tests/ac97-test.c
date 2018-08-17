/*
 * QTest testcase for AC97
 *
 * Copyright (c) 2014 SUSE LINUX Products GmbH
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "libqos/qgraph.h"

typedef struct QAC97 QAC97;

struct QAC97 {
    QOSGraphObject obj;
};

/* Tests only initialization so far. TODO: Replace with functional tests */
static void nop(void *obj, void *data, QGuestAllocator *alloc)
{
}

static void ac97_destructor(QOSGraphObject *obj)
{
    QAC97 *ac97 = (QAC97 *)obj;
    g_free(ac97);
}

static void *ac97_create(void *pci_bus, QGuestAllocator *alloc, void *addr)
{
    QAC97 *ac97 = g_new0(QAC97, 1);
    ac97->obj.destructor = ac97_destructor;

    return &ac97->obj;
}

static void ac97_register_nodes(void)
{
    qos_node_create_driver("AC97", ac97_create);
    qos_node_consumes("AC97", "pci-bus", NULL);
}

libqos_init(ac97_register_nodes);

static void register_ac97_test(void)
{
    qos_add_test("ac97-test", "AC97", nop, NULL);
}

libqos_init(register_ac97_test);
