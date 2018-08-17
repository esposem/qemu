/*
 * QTest testcase for PC-Net NIC
 *
 * Copyright (c) 2013-2014 SUSE LINUX Products GmbH
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "libqos/qgraph.h"

typedef struct QPcnet QPcnet;

struct QPcnet {
    QOSGraphObject obj;
};

/* Tests only initialization so far. TODO: Replace with functional tests */
static void pci_nop(void *obj, void *data, QGuestAllocator *alloc)
{
}

static void pcnet_destructor(QOSGraphObject *obj)
{
    QPcnet *pcnet = (QPcnet *)obj;
    g_free(pcnet);
}

static void *pcnet_create(void *pci_bus, QGuestAllocator *alloc, void *addr)
{
    QPcnet *pcnet = g_new0(QPcnet, 1);
    pcnet->obj.destructor = pcnet_destructor;

    return &pcnet->obj;
}

static void pcnet_register_nodes(void)
{
    qos_node_create_driver("pcnet", pcnet_create);
    qos_node_consumes("pcnet", "pci-bus", NULL);
}

libqos_init(pcnet_register_nodes);

static void register_pcnet_test(void)
{
    qos_add_test("pcnet-test", "pcnet", pci_nop, NULL);
}

libqos_init(register_pcnet_test);
