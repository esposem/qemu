/*
 * QTest testcase for USB OHCI controller
 *
 * Copyright (c) 2014 HUAWEI TECHNOLOGIES CO., LTD.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "libqos/usb.h"
#include "libqos/qgraph.h"

typedef struct QOhci_pci QOhci_pci;

struct QOhci_pci {
    QOSGraphObject obj;
};

/* Tests only initialization so far. TODO: Replace with functional tests */
static void test_ohci_init(void *obj, void *data, QGuestAllocator *alloc)
{
}

static void test_ohci_hotplug(void *obj, void *data, QGuestAllocator *alloc)
{
    usb_test_hotplug("ohci", 1, NULL);
}

static void ohci_pci_destructor(QOSGraphObject *obj)
{
    QOhci_pci *ohci_pci = (QOhci_pci *)obj;
    g_free(ohci_pci);
}

static void *ohci_pci_create(void *pci_bus, QGuestAllocator *alloc, void *addr)
{
    QOhci_pci *ohci_pci = g_new0(QOhci_pci, 1);
    ohci_pci->obj.destructor = ohci_pci_destructor;

    return &ohci_pci->obj;
}

static void ohci_pci_register_nodes(void)
{
    qos_node_create_driver("pci-ohci", ohci_pci_create);
    qos_node_consumes("pci-ohci", "pci-bus", &(QOSGraphEdgeOptions) {
        .extra_device_opts = "id=ohci",
    });
}

libqos_init(ohci_pci_register_nodes);

static void register_ohci_pci_test(void)
{
    qos_add_test("ohci_pci-test-nop", "pci-ohci", test_ohci_init, NULL);
    qos_add_test("ohci_pci-test-hotplug", "pci-ohci", test_ohci_hotplug, NULL);
}

libqos_init(register_ohci_pci_test);
