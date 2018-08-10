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
#include "libqos/virtio-serial.h"

/* virtio-serial-device */
static void qvirtio_serial_device_destroy(QOSGraphObject *obj)
{
    QVirtioSerialDevice *v_serial = (QVirtioSerialDevice *) obj;

    g_free(v_serial);
}

static void *qvirtio_serial_device_get_driver(void *object,
                                              const char *interface)
{
    QVirtioSerialDevice *v_serial = object;
    if (!g_strcmp0(interface, "virtio-serial")) {
        return &v_serial->serial;
    }

    printf("%s not present in virtio-serial-device\n", interface);
    abort();
}

static void *virtio_serial_device_create(void *virtio_dev,
                                         QGuestAllocator *t_alloc,
                                         void *addr)
{
    QVirtioSerialDevice *virtio_device = g_new0(QVirtioSerialDevice, 1);
    QVirtioSerial *interface = &virtio_device->serial;

    interface->vdev = virtio_dev;

    virtio_device->obj.destructor = qvirtio_serial_device_destroy;
    virtio_device->obj.get_driver = qvirtio_serial_device_get_driver;

    return &virtio_device->obj;
}

/* virtio-serial-pci */
static void *qvirtio_serial_pci_get_driver(void *object, const char *interface)
{
    QVirtioSerialPCI *v_serial = object;
    if (!g_strcmp0(interface, "virtio-serial")) {
        return &v_serial->serial;
    }

    printf("%s not present in virtio-serial-pci\n", interface);
    abort();
}

static void *virtio_serial_pci_create(void *pci_bus, QGuestAllocator *t_alloc,
                                      void *addr)
{
    QVirtioSerialPCI *virtio_spci = g_new0(QVirtioSerialPCI, 1);
    QVirtioSerial *interface = &virtio_spci->serial;
    QOSGraphObject *obj = &virtio_spci->pci_vdev.obj;

    virtio_pci_init(&virtio_spci->pci_vdev, pci_bus, addr);
    interface->vdev = &virtio_spci->pci_vdev.vdev;

    obj->get_driver = qvirtio_serial_pci_get_driver;

    return obj;
}

static void virtio_serial(void)
{
   QPCIAddress addr = {
        .devfn = QPCI_DEVFN(4, 0),
    };

    QOSGraphEdgeOptions edge_opts = { };

    /* virtio-serial-device */
    edge_opts.extra_device_opts = "id=vser0";
    qos_node_create_driver("virtio-serial-device",
                            virtio_serial_device_create);
    qos_node_consumes("virtio-serial-device", "virtio", &edge_opts);
    qos_node_produces("virtio-serial-device", "virtio-serial");

    /* virtio-serial-pci */
    edge_opts.extra_device_opts = "id=vser0,addr=04.0";
    add_qpci_address(&edge_opts, &addr);
    qos_node_create_driver("virtio-serial-pci", virtio_serial_pci_create);
    qos_node_consumes("virtio-serial-pci", "pci-bus", &edge_opts);
    qos_node_produces("virtio-serial-pci", "virtio-serial");
}

libqos_init(virtio_serial);
