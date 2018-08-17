/*
 * QTest testcase for vmxnet3 NIC
 *
 * Copyright (c) 2013-2014 SUSE LINUX Products GmbH
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "libqos/qgraph.h"

typedef struct QVmxnet3 QVmxnet3;

struct QVmxnet3 {
    QOSGraphObject obj;
};

/* Tests only initialization so far. TODO: Replace with functional tests */
static void nop(void *obj, void *data, QGuestAllocator *alloc)
{
}

static void vmxnet3_destructor(QOSGraphObject *obj)
{
    QVmxnet3 *vmxnet3 = (QVmxnet3 *)obj;
    g_free(vmxnet3);
}

static void *vmxnet3_create(void *pci_bus, QGuestAllocator *alloc, void *addr)
{
    QVmxnet3 *vmxnet3 = g_new0(QVmxnet3, 1);
    vmxnet3->obj.destructor = vmxnet3_destructor;

    return &vmxnet3->obj;
}

static void vmxnet3_register_nodes(void)
{
    qos_node_create_driver("vmxnet3", vmxnet3_create);
    qos_node_consumes("vmxnet3", "pci-bus", NULL);
}

libqos_init(vmxnet3_register_nodes);

static void register_vmxnet3_test(void)
{
    qos_add_test("vmxnet3-test", "vmxnet3", nop, NULL);
}

libqos_init(register_vmxnet3_test);
