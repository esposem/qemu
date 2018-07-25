/*
 * QTest testcase for VirtIO Balloon
 *
 * Copyright (c) 2014 SUSE LINUX Products GmbH
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "libqos/qgraph.h"
#include "libqos/virtio-balloon.h"

/* Tests only initialization so far. TODO: Replace with functional tests */
static void balloon_nop(void *obj, void *data, QGuestAllocator *alloc)
{
}

static void register_virtio_balloon_test(void)
{
    qos_add_test("balloon-nop", "virtio-balloon", balloon_nop, NULL);
}

libqos_init(register_virtio_balloon_test);
