/*
 * QTest testcase for VirtIO Serial
 *
 * Copyright (c) 2014 SUSE LINUX Products GmbH
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "libqos/virtio-serial.h"

/* Tests only initialization so far. TODO: Replace with functional tests */
static void virtio_serial_nop(void *obj, void *data, QGuestAllocator *alloc)
{
    /* no operation */
}

static void serial_hotplug(void *obj, void *data, QGuestAllocator *alloc)
{
    qtest_qmp_device_add("virtserialport", "hp-port", NULL);
    qtest_qmp_device_del("hp-port");
}

static void virtio_serial_test(void)
{
    qos_add_test("serial-nop", "virtio-serial", virtio_serial_nop, NULL);
    qos_add_test("serial-hotplug", "virtio-serial", serial_hotplug, NULL);
}

libqos_init(virtio_serial_test);
