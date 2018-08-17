/*
 * QTest testcase for tpci200 PCI-IndustryPack bridge
 *
 * Copyright (c) 2014 SUSE LINUX Products GmbH
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "libqos/qgraph.h"

typedef struct QTpci200 QTpci200;
typedef struct QIpack QIpack;

struct QIpack {

};
struct QTpci200 {
    QOSGraphObject obj;
    QIpack ipack;
};

/* Tests only initialization so far. TODO: Replace with functional tests */
static void nop(void *obj, void *data, QGuestAllocator *alloc)
{
}

/* tpci200 */
static void *tpci200_get_driver(void *obj, const char *interface)
{
    QTpci200 *tpci200 = obj;
    if (!g_strcmp0(interface, "ipack")) {
        return &tpci200->ipack;
    }

    fprintf(stderr, "%s not present in tpci200\n", interface);
    g_assert_not_reached();
}

static void tpci200_destructor(QOSGraphObject *obj)
{
    QTpci200 *tpci200 = (QTpci200 *) obj;
    g_free(tpci200);
}

static void *tpci200_create(void *pci_bus, QGuestAllocator *alloc, void *addr)
{
    QTpci200 *tpci200 = g_new0(QTpci200, 1);
    tpci200->obj.destructor = tpci200_destructor;
    tpci200->obj.get_driver = tpci200_get_driver;

    return &tpci200->obj;
}

static void tpci200_register_nodes(void)
{
    qos_node_create_driver("tpci200", tpci200_create);
    qos_node_consumes("tpci200", "pci-bus", &(QOSGraphEdgeOptions) {
        .extra_device_opts = "id=ipack0",
    });
    qos_node_produces("tpci200", "ipack");
}

libqos_init(tpci200_register_nodes);

static void register_tpci200_test(void)
{
    qos_add_test("tpci200-test", "tpci200", nop, NULL);
}

libqos_init(register_tpci200_test);
