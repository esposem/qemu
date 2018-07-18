/*
 * QTest testcase for VirtIO Console
 *
 * Copyright (c) 2014 SUSE LINUX Products GmbH
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "libqos/virtio.h"
#include "libqos/qgraph.h"

/* Tests only initialization so far. TODO: Replace with functional tests */
static void console_nop(void *obj, void *data, QGuestAllocator *alloc)
{
    /* no operation */
}

static void serialport_nop(void *obj, void *data, QGuestAllocator *alloc)
{
    /* no operation */
}

static void register_virtio_console_test(void)
{
    QOSGraphTestOptions opts = { };
    QOSGraphEdgeOptions *edge_opts = &opts.edge;

    edge_opts->before_cmd_line = "-device virtconsole,bus=vser0.0";
    qos_add_test("console-nop", "virtio-serial", console_nop, &opts);

    edge_opts->before_cmd_line = "-device virtserialport,bus=vser0.0";
    qos_add_test("serialport-nop", "virtio-serial", serialport_nop, &opts);
}

libqos_init(register_virtio_console_test);
