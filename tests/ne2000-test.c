/*
 * QTest testcase for ne2000 NIC
 *
 * Copyright (c) 2014 SUSE LINUX Products GmbH
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "libqos/qgraph.h"

typedef struct QNe2k_pci QNe2k_pci;

struct QNe2k_pci {
    QOSGraphObject obj;
};

/* Tests only initialization so far. TODO: Replace with functional tests */
static void pci_nop(void *obj, void *data, QGuestAllocator *alloc)
{
}

static void ne2k_pci_destructor(QOSGraphObject *obj)
{
    QNe2k_pci *ne2k_pci = (QNe2k_pci *)obj;
    g_free(ne2k_pci);
}

static void *ne2k_pci_create(void *pci_bus, QGuestAllocator *alloc, void *addr)
{
    QNe2k_pci *ne2k_pci = g_new0(QNe2k_pci, 1);
    ne2k_pci->obj.destructor = ne2k_pci_destructor;

    return &ne2k_pci->obj;
}

static void ne2000_register_nodes(void)
{
    qos_node_create_driver("ne2k_pci", ne2k_pci_create);
    qos_node_consumes("ne2k_pci", "pci-bus", NULL);
}

libqos_init(ne2000_register_nodes);

static void register_ne2000_test(void)
{
    qos_add_test("ne2000-test", "ne2k_pci", pci_nop, NULL);
}

libqos_init(register_ne2000_test);