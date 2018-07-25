/*
 * libqos driver framework
 *
 * Copyright (c) 2018 Emanuele Giuseppe Esposito <e.emanuelegiuseppe@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License version 2 as published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "libqos/qgraph.h"
#include "libqos/virtio-balloon.h"

/* virtio-balloon-device */
static void qvirtio_balloon_device_destructor(QOSGraphObject *obj)
{
    QVirtioBalloonDevice *v_balloon = (QVirtioBalloonDevice *) obj;

    g_free(v_balloon);
}

static void *qvirtio_balloon_device_get_driver(void *object,
                                               const char *interface)
{
    QVirtioBalloonDevice *v_balloon = object;
    if (!g_strcmp0(interface, "virtio-balloon")) {
        return &v_balloon->balloon;
    }

    fprintf(stderr, "%s not present in virtio-balloon-device\n", interface);
    g_assert_not_reached();
}

static void *virtio_balloon_device_create(void *virtio_dev,
                                          QGuestAllocator *t_alloc,
                                          void *addr)
{
    QVirtioBalloonDevice *virtio_bdevice = g_new0(QVirtioBalloonDevice, 1);
    QVirtioBalloon *interface = &virtio_bdevice->balloon;

    interface->vdev = virtio_dev;

    virtio_bdevice->obj.destructor = qvirtio_balloon_device_destructor;
    virtio_bdevice->obj.get_driver = qvirtio_balloon_device_get_driver;

    return &virtio_bdevice->obj;
}

/* virtio-balloon-pci */
static void *qvirtio_balloon_pci_get_driver(void *object,
                                            const char *interface)
{
    QVirtioBalloonPCI *v_balloon = object;
    if (!g_strcmp0(interface, "virtio-balloon")) {
        return &v_balloon->balloon;
    }

    fprintf(stderr, "%s not present in virtio-balloon-pci\n", interface);
    g_assert_not_reached();
}

static void *virtio_balloon_pci_create(void *pci_bus, QGuestAllocator *t_alloc,
                                  void *addr)
{
    QVirtioBalloonPCI *virtio_bpci = g_new0(QVirtioBalloonPCI, 1);
    QVirtioBalloon *interface = &virtio_bpci->balloon;
    QOSGraphObject *obj = &virtio_bpci->pci_vdev.obj;


    virtio_pci_init(&virtio_bpci->pci_vdev, pci_bus, addr);
    interface->vdev = &virtio_bpci->pci_vdev.vdev;

    obj->get_driver = qvirtio_balloon_pci_get_driver;

    return obj;
}

static void virtio_balloon_register_nodes(void)
{
    QPCIAddress addr = {
        .devfn = QPCI_DEVFN(4, 0),
    };

    QOSGraphEdgeOptions opts = {
        .extra_device_opts = "addr=04.0",
    };

    /* virtio-balloon-device */
    qos_node_create_driver("virtio-balloon-device",
                            virtio_balloon_device_create);
    qos_node_consumes("virtio-balloon-device", "virtio", NULL);
    qos_node_produces("virtio-balloon-device", "virtio-balloon");

    /* virtio-balloon-pci */
    add_qpci_address(&opts, &addr);
    qos_node_create_driver("virtio-balloon-pci", virtio_balloon_pci_create);
    qos_node_consumes("virtio-balloon-pci", "pci-bus", &opts);
    qos_node_produces("virtio-balloon-pci", "virtio-balloon");
}

libqos_init(virtio_balloon_register_nodes);
