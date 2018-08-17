/*
 * QTest testcase for NVMe
 *
 * Copyright (c) 2014 SUSE LINUX Products GmbH
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "libqos/qgraph.h"

typedef struct QNvme QNvme;

struct QNvme {
    QOSGraphObject obj;
};

/* Tests only initialization so far. TODO: Replace with functional tests */
static void nop(void *obj, void *data, QGuestAllocator *alloc)
{
}

static void nvme_destructor(QOSGraphObject *obj)
{
    QNvme *nvme = (QNvme *)obj;
    g_free(nvme);
}

static void *nvme_create(void *pci_bus, QGuestAllocator *alloc, void *addr)
{
    QNvme *nvme = g_new0(QNvme, 1);
    nvme->obj.destructor = nvme_destructor;

    return &nvme->obj;
}

static void nvme_register_nodes(void)
{
    qos_node_create_driver("nvme", nvme_create);
    qos_node_consumes("nvme", "pci-bus", &(QOSGraphEdgeOptions) {
        .extra_device_opts = "drive=drv0,serial=foo",
        .before_cmd_line = "-drive id=drv0,if=none,file=null-co://,format=raw",
    });
}

libqos_init(nvme_register_nodes);

static void register_nvme_test(void)
{
    qos_add_test("nvme-test", "nvme", nop, NULL);
}

libqos_init(register_nvme_test);
