/*
 * QTest testcase for VirtIO Block Device
 *
 * Copyright (c) 2014 SUSE LINUX Products GmbH
 * Copyright (c) 2014 Marc Marí
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "libqtest.h"
 #include "qemu/bswap.h"
#include "standard-headers/linux/virtio_blk.h"
#include "standard-headers/linux/virtio_pci.h"
#include "libqos/qgraph.h"
#include "libqos/virtio-blk.h"

#define TEST_IMAGE_SIZE         (64 * 1024 * 1024)
#define QVIRTIO_BLK_TIMEOUT_US  (30 * 1000 * 1000)
#define PCI_SLOT_HP             0x06

static char *tmp_path;

typedef struct QVirtioBlkReq {
    uint32_t type;
    uint32_t ioprio;
    uint64_t sector;
    char *data;
    uint8_t status;
} QVirtioBlkReq;

static char *drive_create(void)
{
    int fd, ret;
    char *t_path = g_strdup("/tmp/qtest.XXXXXX");

    /* Create a temporary raw image */
    fd = mkstemp(t_path);
    g_assert_cmpint(fd, >=, 0);
    ret = ftruncate(fd, TEST_IMAGE_SIZE);
    g_assert_cmpint(ret, ==, 0);
    close(fd);

    return t_path;
}

static inline void virtio_blk_fix_request(QVirtioDevice *d, QVirtioBlkReq *req)
{
#ifdef HOST_WORDS_BIGENDIAN
    const bool host_is_big_endian = true;
#else
    const bool host_is_big_endian = false;
#endif

    if (qvirtio_is_big_endian(d) != host_is_big_endian) {
        req->type = bswap32(req->type);
        req->ioprio = bswap32(req->ioprio);
        req->sector = bswap64(req->sector);
    }
}

static uint64_t virtio_blk_request(QGuestAllocator *alloc, QVirtioDevice *d,
                                   QVirtioBlkReq *req, uint64_t data_size)
{
    uint64_t addr;
    uint8_t status = 0xFF;

    g_assert_cmpuint(data_size % 512, ==, 0);
    addr = guest_alloc(alloc, sizeof(*req) + data_size);

    virtio_blk_fix_request(d, req);

    memwrite(addr, req, 16);
    memwrite(addr + 16, req->data, data_size);
    memwrite(addr + 16 + data_size, &status, sizeof(status));

    return addr;
}

static void test_basic(QVirtioDevice *dev, QGuestAllocator *alloc,
                       QVirtQueue *vq)
{
    QVirtioBlkReq req;
    uint64_t req_addr;
    uint64_t capacity;
    uint32_t features;
    uint32_t free_head;
    uint8_t status;
    char *data;

    capacity = qvirtio_config_readq(dev, 0);

    g_assert_cmpint(capacity, ==, TEST_IMAGE_SIZE / 512);

    features = qvirtio_get_features(dev);
    features = features & ~(QVIRTIO_F_BAD_FEATURE |
                    (1u << VIRTIO_RING_F_INDIRECT_DESC) |
                    (1u << VIRTIO_RING_F_EVENT_IDX) |
                    (1u << VIRTIO_BLK_F_SCSI));
    qvirtio_set_features(dev, features);

    /* Write and read with 3 descriptor layout */
    /* Write request */
    req.type = VIRTIO_BLK_T_OUT;
    req.ioprio = 1;
    req.sector = 0;
    req.data = g_malloc0(512);
    strcpy(req.data, "TEST");

    req_addr = virtio_blk_request(alloc, dev, &req, 512);

    g_free(req.data);

    free_head = qvirtqueue_add(vq, req_addr, 16, false, true);
    qvirtqueue_add(vq, req_addr + 16, 512, false, true);
    qvirtqueue_add(vq, req_addr + 528, 1, true, false);

    qvirtqueue_kick(dev, vq, free_head);

    qvirtio_wait_used_elem(dev, vq, free_head, NULL, QVIRTIO_BLK_TIMEOUT_US);
    status = readb(req_addr + 528);
    g_assert_cmpint(status, ==, 0);

    guest_free(alloc, req_addr);

    /* Read request */
    req.type = VIRTIO_BLK_T_IN;
    req.ioprio = 1;
    req.sector = 0;
    req.data = g_malloc0(512);

    req_addr = virtio_blk_request(alloc, dev, &req, 512);

    g_free(req.data);

    free_head = qvirtqueue_add(vq, req_addr, 16, false, true);
    qvirtqueue_add(vq, req_addr + 16, 512, true, true);
    qvirtqueue_add(vq, req_addr + 528, 1, true, false);

    qvirtqueue_kick(dev, vq, free_head);

    qvirtio_wait_used_elem(dev, vq, free_head, NULL, QVIRTIO_BLK_TIMEOUT_US);
    status = readb(req_addr + 528);
    g_assert_cmpint(status, ==, 0);

    data = g_malloc0(512);
    memread(req_addr + 16, data, 512);
    g_assert_cmpstr(data, ==, "TEST");
    g_free(data);

    guest_free(alloc, req_addr);

    if (features & (1u << VIRTIO_F_ANY_LAYOUT)) {
        /* Write and read with 2 descriptor layout */
        /* Write request */
        req.type = VIRTIO_BLK_T_OUT;
        req.ioprio = 1;
        req.sector = 1;
        req.data = g_malloc0(512);
        strcpy(req.data, "TEST");

        req_addr = virtio_blk_request(alloc, dev, &req, 512);

        g_free(req.data);

        free_head = qvirtqueue_add(vq, req_addr, 528, false, true);
        qvirtqueue_add(vq, req_addr + 528, 1, true, false);
        qvirtqueue_kick(dev, vq, free_head);

        qvirtio_wait_used_elem(dev, vq, free_head, NULL,
                               QVIRTIO_BLK_TIMEOUT_US);
        status = readb(req_addr + 528);
        g_assert_cmpint(status, ==, 0);

        guest_free(alloc, req_addr);

        /* Read request */
        req.type = VIRTIO_BLK_T_IN;
        req.ioprio = 1;
        req.sector = 1;
        req.data = g_malloc0(512);

        req_addr = virtio_blk_request(alloc, dev, &req, 512);

        g_free(req.data);

        free_head = qvirtqueue_add(vq, req_addr, 16, false, true);
        qvirtqueue_add(vq, req_addr + 16, 513, true, false);

        qvirtqueue_kick(dev, vq, free_head);

        qvirtio_wait_used_elem(dev, vq, free_head, NULL,
                               QVIRTIO_BLK_TIMEOUT_US);
        status = readb(req_addr + 528);
        g_assert_cmpint(status, ==, 0);

        data = g_malloc0(512);
        memread(req_addr + 16, data, 512);
        g_assert_cmpstr(data, ==, "TEST");
        g_free(data);

        guest_free(alloc, req_addr);
    }
}

static void basic(void *obj, void *data, QGuestAllocator *t_alloc)
{
    QVirtioBlk *blk_if = obj;
    QVirtQueue *vq;
    vq = qvirtqueue_setup(blk_if->vdev, t_alloc, 0);
    test_basic(blk_if->vdev, t_alloc, vq);
    qvirtqueue_cleanup(blk_if->vdev->bus, vq, t_alloc);

}

static void indirect(void *obj, void *u_data, QGuestAllocator *t_alloc)
{
    QVirtQueue *vq;
    QVirtioBlk *blk_if = obj;
    QVirtioDevice *dev = blk_if->vdev;
    QVirtioBlkReq req;
    QVRingIndirectDesc *indirect;
    uint64_t req_addr;
    uint64_t capacity;
    uint32_t features;
    uint32_t free_head;
    uint8_t status;
    char *data;

    capacity = qvirtio_config_readq(dev, 0);
    g_assert_cmpint(capacity, ==, TEST_IMAGE_SIZE / 512);

    features = qvirtio_get_features(dev);
    g_assert_cmphex(features & (1u << VIRTIO_RING_F_INDIRECT_DESC), !=, 0);
    features = features & ~(QVIRTIO_F_BAD_FEATURE |
                            (1u << VIRTIO_RING_F_EVENT_IDX) |
                            (1u << VIRTIO_BLK_F_SCSI));
    qvirtio_set_features(dev, features);

    vq = qvirtqueue_setup(dev, t_alloc, 0);

    /* Write request */
    req.type = VIRTIO_BLK_T_OUT;
    req.ioprio = 1;
    req.sector = 0;
    req.data = g_malloc0(512);
    strcpy(req.data, "TEST");

    req_addr = virtio_blk_request(t_alloc, dev, &req, 512);

    g_free(req.data);

    indirect = qvring_indirect_desc_setup(dev, t_alloc, 2);
    qvring_indirect_desc_add(indirect, req_addr, 528, false);
    qvring_indirect_desc_add(indirect, req_addr + 528, 1, true);
    free_head = qvirtqueue_add_indirect(vq, indirect);
    qvirtqueue_kick(dev, vq, free_head);

    qvirtio_wait_used_elem(dev, vq, free_head, NULL,
                           QVIRTIO_BLK_TIMEOUT_US);
    status = readb(req_addr + 528);
    g_assert_cmpint(status, ==, 0);

    g_free(indirect);
    guest_free(t_alloc, req_addr);

    /* Read request */
    req.type = VIRTIO_BLK_T_IN;
    req.ioprio = 1;
    req.sector = 0;
    req.data = g_malloc0(512);
    strcpy(req.data, "TEST");

    req_addr = virtio_blk_request(t_alloc, dev, &req, 512);

    g_free(req.data);

    indirect = qvring_indirect_desc_setup(dev, t_alloc, 2);
    qvring_indirect_desc_add(indirect, req_addr, 16, false);
    qvring_indirect_desc_add(indirect, req_addr + 16, 513, true);
    free_head = qvirtqueue_add_indirect(vq, indirect);
    qvirtqueue_kick(dev, vq, free_head);

    qvirtio_wait_used_elem(dev, vq, free_head, NULL,
                           QVIRTIO_BLK_TIMEOUT_US);
    status = readb(req_addr + 528);
    g_assert_cmpint(status, ==, 0);

    data = g_malloc0(512);
    memread(req_addr + 16, data, 512);
    g_assert_cmpstr(data, ==, "TEST");
    g_free(data);

    g_free(indirect);
    guest_free(t_alloc, req_addr);
    qvirtqueue_cleanup(dev->bus, vq, t_alloc);
}

static void config(void *obj, void *data, QGuestAllocator *t_alloc)
{
    QVirtioBlk *blk_if = obj;
    QVirtioDevice *dev = blk_if->vdev;
    int n_size = TEST_IMAGE_SIZE / 2;
    uint64_t capacity;

    capacity = qvirtio_config_readq(dev, 0);
    g_assert_cmpint(capacity, ==, TEST_IMAGE_SIZE / 512);

    qmp_discard_response("{ 'execute': 'block_resize', "
                         " 'arguments': { 'device': 'drive0', "
                         " 'size': %d } }", n_size);
    qvirtio_wait_config_isr(dev, QVIRTIO_BLK_TIMEOUT_US);

    capacity = qvirtio_config_readq(dev, 0);
    g_assert_cmpint(capacity, ==, n_size / 512);
}

static void msix(void *obj, void *u_data, QGuestAllocator *t_alloc)
{
    QVirtQueue *vq;
    QVirtioBlkPCI *blk = obj;
    QVirtioPCIDevice *pdev = &blk->pci_vdev;
    QVirtioDevice *dev = &pdev->vdev;
    QVirtioBlkReq req;
    int n_size = TEST_IMAGE_SIZE / 2;
    uint64_t req_addr;
    uint64_t capacity;
    uint32_t features;
    uint32_t free_head;
    uint8_t status;
    char *data;
    QOSGraphObject *blk_object = obj;
    QPCIDevice *pci_dev = blk_object->get_driver(blk_object, "pci-device");

    /* FIXME: add spapr support */
    if (qpci_has_buggy_msi(pci_dev)) {
        return;
    }

    qpci_msix_enable(pdev->pdev);
    qvirtio_pci_set_msix_configuration_vector(pdev, t_alloc, 0);

    capacity = qvirtio_config_readq(dev, 0);
    g_assert_cmpint(capacity, ==, TEST_IMAGE_SIZE / 512);

    features = qvirtio_get_features(dev);
    features = features & ~(QVIRTIO_F_BAD_FEATURE |
                            (1u << VIRTIO_RING_F_INDIRECT_DESC) |
                            (1u << VIRTIO_RING_F_EVENT_IDX) |
                            (1u << VIRTIO_BLK_F_SCSI));
    qvirtio_set_features(dev, features);

    vq = qvirtqueue_setup(dev, t_alloc, 0);
    qvirtqueue_pci_msix_setup(pdev, (QVirtQueuePCI *)vq, t_alloc, 1);

    qmp_discard_response("{ 'execute': 'block_resize', "
                         " 'arguments': { 'device': 'drive0', "
                         " 'size': %d } }", n_size);

    qvirtio_wait_config_isr(dev, QVIRTIO_BLK_TIMEOUT_US);

    capacity = qvirtio_config_readq(dev, 0);
    g_assert_cmpint(capacity, ==, n_size / 512);

    /* Write request */
    req.type = VIRTIO_BLK_T_OUT;
    req.ioprio = 1;
    req.sector = 0;
    req.data = g_malloc0(512);
    strcpy(req.data, "TEST");

    req_addr = virtio_blk_request(t_alloc, dev, &req, 512);

    g_free(req.data);

    free_head = qvirtqueue_add(vq, req_addr, 16, false, true);
    qvirtqueue_add(vq, req_addr + 16, 512, false, true);
    qvirtqueue_add(vq, req_addr + 528, 1, true, false);
    qvirtqueue_kick(dev, vq, free_head);

    qvirtio_wait_used_elem(dev, vq, free_head, NULL,
                           QVIRTIO_BLK_TIMEOUT_US);

    status = readb(req_addr + 528);
    g_assert_cmpint(status, ==, 0);

    guest_free(t_alloc, req_addr);

    /* Read request */
    req.type = VIRTIO_BLK_T_IN;
    req.ioprio = 1;
    req.sector = 0;
    req.data = g_malloc0(512);

    req_addr = virtio_blk_request(t_alloc, dev, &req, 512);

    g_free(req.data);

    free_head = qvirtqueue_add(vq, req_addr, 16, false, true);
    qvirtqueue_add(vq, req_addr + 16, 512, true, true);
    qvirtqueue_add(vq, req_addr + 528, 1, true, false);

    qvirtqueue_kick(dev, vq, free_head);


    qvirtio_wait_used_elem(dev, vq, free_head, NULL,
                           QVIRTIO_BLK_TIMEOUT_US);

    status = readb(req_addr + 528);
    g_assert_cmpint(status, ==, 0);

    data = g_malloc0(512);
    memread(req_addr + 16, data, 512);
    g_assert_cmpstr(data, ==, "TEST");
    g_free(data);

    guest_free(t_alloc, req_addr);

    /* End test */
    qpci_msix_disable(pdev->pdev);
    qvirtqueue_cleanup(dev->bus, vq, t_alloc);
}

static void idx(void *obj, void *u_data, QGuestAllocator *t_alloc)
{
    QVirtQueue *vq;
    QVirtioBlkPCI *blk = obj;
    QVirtioPCIDevice *pdev = &blk->pci_vdev;
    QVirtioDevice *dev = &pdev->vdev;
    QVirtioBlkReq req;
    uint64_t req_addr;
    uint64_t capacity;
    uint32_t features;
    uint32_t free_head;
    uint32_t write_head;
    uint32_t desc_idx;
    uint8_t status;
    char *data;
    QOSGraphObject *blk_object = obj;
    QPCIDevice *pci_dev = blk_object->get_driver(blk_object, "pci-device");

    /* FIXME: add spapr support */
    if (qpci_has_buggy_msi(pci_dev)) {
        return;
    }

    qpci_msix_enable(pdev->pdev);
    qvirtio_pci_set_msix_configuration_vector(pdev, t_alloc, 0);

    capacity = qvirtio_config_readq(dev, 0);
    g_assert_cmpint(capacity, ==, TEST_IMAGE_SIZE / 512);

    features = qvirtio_get_features(dev);
    features = features & ~(QVIRTIO_F_BAD_FEATURE |
                            (1u << VIRTIO_RING_F_INDIRECT_DESC) |
                            (1u << VIRTIO_F_NOTIFY_ON_EMPTY) |
                            (1u << VIRTIO_BLK_F_SCSI));
    qvirtio_set_features(dev, features);

    vq = qvirtqueue_setup(dev, t_alloc, 0);
    qvirtqueue_pci_msix_setup(pdev, (QVirtQueuePCI *)vq, t_alloc, 1);

    /* Write request */
    req.type = VIRTIO_BLK_T_OUT;
    req.ioprio = 1;
    req.sector = 0;
    req.data = g_malloc0(512);
    strcpy(req.data, "TEST");

    req_addr = virtio_blk_request(t_alloc, dev, &req, 512);

    g_free(req.data);

    free_head = qvirtqueue_add(vq, req_addr, 16, false, true);
    qvirtqueue_add(vq, req_addr + 16, 512, false, true);
    qvirtqueue_add(vq, req_addr + 528, 1, true, false);
    qvirtqueue_kick(dev, vq, free_head);

    qvirtio_wait_used_elem(dev, vq, free_head, NULL,
                           QVIRTIO_BLK_TIMEOUT_US);

    /* Write request */
    req.type = VIRTIO_BLK_T_OUT;
    req.ioprio = 1;
    req.sector = 1;
    req.data = g_malloc0(512);
    strcpy(req.data, "TEST");

    req_addr = virtio_blk_request(t_alloc, dev, &req, 512);

    g_free(req.data);

    /* Notify after processing the third request */
    qvirtqueue_set_used_event(vq, 2);
    free_head = qvirtqueue_add(vq, req_addr, 16, false, true);
    qvirtqueue_add(vq, req_addr + 16, 512, false, true);
    qvirtqueue_add(vq, req_addr + 528, 1, true, false);
    qvirtqueue_kick(dev, vq, free_head);
    write_head = free_head;

    /* No notification expected */
    status = qvirtio_wait_status_byte_no_isr(dev,
                                             vq, req_addr + 528,
                                             QVIRTIO_BLK_TIMEOUT_US);
    g_assert_cmpint(status, ==, 0);

    guest_free(t_alloc, req_addr);

    /* Read request */
    req.type = VIRTIO_BLK_T_IN;
    req.ioprio = 1;
    req.sector = 1;
    req.data = g_malloc0(512);

    req_addr = virtio_blk_request(t_alloc, dev, &req, 512);

    g_free(req.data);

    free_head = qvirtqueue_add(vq, req_addr, 16, false, true);
    qvirtqueue_add(vq, req_addr + 16, 512, true, true);
    qvirtqueue_add(vq, req_addr + 528, 1, true, false);

    qvirtqueue_kick(dev, vq, free_head);

    /* We get just one notification for both requests */
    qvirtio_wait_used_elem(dev, vq, write_head, NULL,
                           QVIRTIO_BLK_TIMEOUT_US);
    g_assert(qvirtqueue_get_buf(vq, &desc_idx, NULL));
    g_assert_cmpint(desc_idx, ==, free_head);

    status = readb(req_addr + 528);
    g_assert_cmpint(status, ==, 0);

    data = g_malloc0(512);
    memread(req_addr + 16, data, 512);
    g_assert_cmpstr(data, ==, "TEST");
    g_free(data);

    guest_free(t_alloc, req_addr);

    /* End test */
    qpci_msix_disable(pdev->pdev);

    qvirtqueue_cleanup(dev->bus, vq, t_alloc);
}

static void pci_hotplug(void *obj, void *data, QGuestAllocator *t_alloc)
{
    QVirtioBlkPCI *blk = obj;
    QVirtioPCIDevice *pdev = &blk->pci_vdev;
    QVirtioPCIDevice *dev;

    /* plug secondary disk */
    qpci_plug_device_test("virtio-blk-pci", "drv1", PCI_SLOT_HP,
                          "'drive': 'drive1'");

    dev = qvirtio_pci_device_find_slot(pdev->pdev->bus, VIRTIO_ID_BLOCK,
                                    PCI_SLOT_HP);
    g_assert(dev);
    qvirtio_pci_device_disable(dev);
    qvirtio_pci_device_free(dev);

    /* unplug secondary disk */
    qpci_unplug_acpi_device_test("drv1", PCI_SLOT_HP);
}

/*
 * Check that setting the vring addr on a non-existent virtqueue does
 * not crash.
 */
static void test_nonexistent_virtqueue(void *obj, void *data,
                                       QGuestAllocator *t_alloc)
{
    QVirtioBlkPCI *blk = obj;
    QVirtioPCIDevice *pdev = &blk->pci_vdev;
    QPCIBar bar0;
    QPCIDevice *dev;

    dev = qpci_device_find(pdev->pdev->bus, QPCI_DEVFN(4, 0));
    g_assert(dev != NULL);
    qpci_device_enable(dev);

    bar0 = qpci_iomap(dev, 0, NULL);

    qpci_io_writeb(dev, bar0, VIRTIO_PCI_QUEUE_SEL, 2);
    qpci_io_writel(dev, bar0, VIRTIO_PCI_QUEUE_PFN, 1);


    g_free(dev);
}

static void basic_resize(void *obj, void *data, QGuestAllocator *t_alloc)
{
    QVirtioBlk *blk_if = obj;
    QVirtioDevice *dev = blk_if->vdev;
    int n_size = TEST_IMAGE_SIZE / 2;
    uint64_t capacity;
    QVirtQueue *vq;

    vq = qvirtqueue_setup(dev, t_alloc, 0);

    test_basic(dev, t_alloc, vq);

    qmp_discard_response("{ 'execute': 'block_resize', "
                         " 'arguments': { 'device': 'drive0', "
                         " 'size': %d } }", n_size);

    qvirtio_wait_queue_isr(dev, vq, QVIRTIO_BLK_TIMEOUT_US);

    capacity = qvirtio_config_readq(dev, 0);
    g_assert_cmpint(capacity, ==, n_size / 512);

    qvirtqueue_cleanup(dev->bus, vq, t_alloc);

}

static void virtio_blk_test_setup(char **cmd_line)
{
    char *new_cmdline;

    tmp_path = drive_create();

    new_cmdline =
        g_strdup_printf("%s "
                        "-drive if=none,id=drive0,file=%s,format=raw "
                        "-drive if=none,id=drive1,file=null-co://,format=raw ",
                        *cmd_line, tmp_path);

    g_assert_nonnull(new_cmdline);

    g_free(*cmd_line);
    *cmd_line = new_cmdline;
}

static void virtio_blk_test_cleanup(void)
{
    unlink(tmp_path);
    g_free(tmp_path);
}

static void virtio_blk_test(void)
{
    QOSGraphTestOptions opts = {
        .before = virtio_blk_test_setup,
        .after = virtio_blk_test_cleanup,
    };

    qos_add_test("blk-indirect", "virtio-blk", indirect, &opts);
    qos_add_test("blk-config", "virtio-blk", config, &opts);
    qos_add_test("blk-basic", "virtio-blk", basic, &opts);
    qos_add_test("blk-basic-resize", "virtio-blk", basic_resize, &opts);

    /* tests just for virtio-blk-pci */
    qos_add_test("blk-msix", "virtio-blk-pci", msix, &opts);
    qos_add_test("blk-idx", "virtio-blk-pci", idx, &opts);
    qos_add_test("blk-nxvirtq", "virtio-blk-pci",
                      test_nonexistent_virtqueue, &opts);
    qos_add_test("blk-hotplug", "virtio-blk-pci", pci_hotplug, &opts);
}

libqos_init(virtio_blk_test);
