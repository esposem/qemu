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

#include <getopt.h>
#include "qemu/osdep.h"
#include "libqtest.h"
#include "qapi/qmp/qdict.h"
#include "qapi/qmp/qbool.h"
#include "qapi/qmp/qstring.h"
#include "qapi/qmp/qlist.h"
#include "libqos/malloc.h"
#include "libqos/qgraph.h"
#include "libqos/qgraph_extra.h"

static char *old_path;

/**
 * create_machine_name(): appends the architecture to @name if
 * @is_machine is valid.
 */
static void create_machine_name(const char **name, bool is_machine)
{
    const char *arch;
    if (!is_machine) {
        return;
    }
    arch = qtest_get_arch();
    *name = g_strconcat(arch, "/", *name, NULL);
}

/**
 * destroy_machine_name(): frees the given @name if
 * @is_machine is valid.
 */
static void destroy_machine_name(const char *name, bool is_machine)
{
    if (!is_machine) {
        return;
    }
    g_free((char *)name);
}

/**
 * apply_to_qlist(): using QMP queries QEMU for a list of
 * machines and devices available, and sets the respective node
 * as TRUE. If a node is found, also all its produced and contained
 * child are marked available.
 *
 * See qos_graph_node_set_availability() for more info
 */
static void apply_to_qlist(QList *list, bool is_machine)
{
    const QListEntry *p;
    const char *name;
    bool abstract;
    QDict *minfo;
    QObject *qobj;
    QString *qstr;
    QBool *qbol;

    for (p = qlist_first(list); p; p = qlist_next(p)) {
        minfo = qobject_to(QDict, qlist_entry_obj(p));
        g_assert(minfo);
        qobj = qdict_get(minfo, "name");
        g_assert(qobj);
        qstr = qobject_to(QString, qobj);
        g_assert(qstr);
        name = qstring_get_str(qstr);

        create_machine_name(&name, is_machine);
        qos_graph_node_set_availability(name, TRUE);

        qobj = qdict_get(minfo, "alias");
        if (qobj) {
            qstr = qobject_to(QString, qobj);
            g_assert(qstr);

            destroy_machine_name(name, is_machine);
            name = qstring_get_str(qstr);

            create_machine_name(&name, is_machine);
            qos_graph_node_set_availability(name, TRUE);
        }

        qobj = qdict_get(minfo, "abstract");
        if (qobj) {
            qbol = qobject_to(QBool, qobj);
            g_assert(qbol);
            abstract = qbool_get_bool(qbol);
            qos_delete_abstract_cmd_line(name, abstract);
        }

        destroy_machine_name(name, is_machine);
    }
}

/**
 * qos_set_machines_devices_available(): sets availability of qgraph
 * machines and devices.
 *
 * This function firstly starts QEMU with "-machine none" option,
 * and then executes the QMP protocol asking for the list of devices
 * and machines available.
 *
 * for each of these items, it looks up the corresponding qgraph node,
 * setting it as available. The list currently returns all devices that
 * are either machines or QEDGE_CONSUMED_BY other nodes.
 * Therefore, in order to mark all other nodes, it recursively sets
 * all its QEDGE_CONTAINS and QEDGE_PRODUCES child as available too.
 */
static void qos_set_machines_devices_available(void)
{
    QDict *response;
    QDict *args = qdict_new();
    QList *list;

    qtest_start("-machine none");
    response = qmp("{ 'execute': 'query-machines' }");
    g_assert(response);
    list = qdict_get_qlist(response, "return");
    g_assert(list);

    apply_to_qlist(list, TRUE);

    qobject_unref(response);

    qdict_put_bool(args, "abstract", TRUE);
    qdict_put_str(args, "implements", "device");

    response = qmp("{'execute': 'qom-list-types',"
                   " 'arguments': %p }", args);
    g_assert(qdict_haskey(response, "return"));
    list = qdict_get_qlist(response, "return");

    apply_to_qlist(list, FALSE);

    qtest_end();
    qobject_unref(response);

}

/* small API to remember a subset of the allocated objects */
typedef struct {
    QOSGraphObject *obj;
    char *name;
} CollectorData;

static CollectorData collector[(QOS_PATH_MAX_ELEMENT_SIZE * 2)];
static int collector_size;

static void destroy_objects(CollectorData *data)
{
    /* field @name is not free'd since it will be done when
     * the node hash map will be destroyed
     */
    QOSGraphObject *obj = data->obj;
    if (obj->destructor) {
        obj->destructor(obj);
    } else {
        printf("Warning: Node %s allocates but does not provide"
               " a destructor\n", data->name);
    }
}

static void add_to_collector(QOSGraphObject *obj, char *name)
{
    if (collector_size == (QOS_PATH_MAX_ELEMENT_SIZE * 2)) {
        printf("Warning: Collector full, more than %d object to allocate\n",
               collector_size);
        return;
    }
    collector[collector_size].name = name;
    collector[collector_size].obj = obj;
    collector_size++;
}

static void destroy_all_objects(void)
{
    int current = collector_size - 1;

    while (current >= 0) {
        destroy_objects(&collector[current]);
        current--;
    }
}

static void empty_collector(void)
{
    collector_size = 0;
}

static QGuestAllocator *get_machine_allocator(QOSGraphObject *obj, char *name)
{
    if (obj->get_driver) {
        return obj->get_driver(obj, "guest_allocator");
    } else {
        printf("Warning: machine %s must produce"
                "guest_allocator (returning NULL is fine)\n", name);
    }
    return NULL;
}

static void object_start_hw(QOSGraphObject *obj)
{
    if (obj->start_hw) {
        obj->start_hw(obj);
    }
}

static void restart_qemu_or_continue(char *path)
{
    /* compares the current command line with the
    * one previously executed: if they are the same,
    * don't restart QEMU, if they differ, stop previous
    * QEMU execution (if active) and restart it with
    * new command line
    */
    if (g_strcmp0(old_path, path)) {
        if (old_path) {
            qtest_end();
            g_free(old_path);
        }
        old_path = path;
        global_qtest = qtest_start(path);
    } else { /* if cmd line is the same, reset the guest */
        qmp_discard_response("{ 'execute': 'system_reset' }");
        qmp_eventwait("RESET");
    }
}

void qos_invalidate_command_line(void)
{
    old_path = NULL;
}

/**
 * allocate_objects(): given an array of nodes @arg,
 * walks the path invoking all constructors and
 * passing the corresponding parameter in order to
 * continue the objects allocation.
 * Once the test is reached, its function is executed.
 *
 * Since only the machine and QEDGE_CONSUMED_BY nodes actually
 * allocate something in the constructor, a garbage collector
 * saves their pointer in an array, so that after execution
 * they can be safely free'd.
 *
 * Note: as specified in walk_path() too, @arg is an array of
 * char *, where arg[0] is a pointer to the command line
 * string that will be used to properly start QEMU when executing
 * the test, and the remaining elements represent the actual objects
 * that will be allocated.
 *
 * The order of execution is the following:
 * 1) @before test function as defined in the given QOSGraphTestOptions
 * 2) start QEMU
 * 3) call all nodes constructor and get_driver/get_device depending on edge
 * 4) start the hardware (*_device_enable functions)
 * 5) start test
 * 6) @after test function as defined in the given QOSGraphTestOptions
 * 7) call all nodes destructor
 *
 */
static void allocate_objects(const void *arg)
{
    QOSEdgeType etype;
    QOSGraphEdge *edge = NULL;
    QOSGraphNode *node, *test_node;
    QOSGraphObject *obj;
    QGuestAllocator *machine_a = NULL;
    int current = 1, has_to_allocate = 0;
    void *void_obj = NULL;
    char **path = (char **) arg;

    node = qos_graph_get_node(path[current]);

    /* Before test */
    test_node = qos_graph_get_node(path[(g_strv_length(path) - 1)]);
    if (test_node->u.test.before) {
        test_node->u.test.before(&path[0]);
    }

    while (current < QOS_PATH_MAX_ELEMENT_SIZE) {

        /* Allocate objects */
        switch (node->type) {
        case QNODE_MACHINE:
            restart_qemu_or_continue(path[0]);

            void_obj = node->u.machine.constructor();
            machine_a = get_machine_allocator(void_obj, node->name);
            object_start_hw(void_obj);
            add_to_collector(void_obj, node->name);
            break;

        case QNODE_DRIVER:
            if (has_to_allocate) {
                void_obj = node->u.driver.constructor(void_obj, machine_a,
                                             qos_graph_get_edge_arg(edge));
                add_to_collector(void_obj, node->name);
            }
            /* drivers can have an initializer even if they are contained */
            object_start_hw(void_obj);
            break;

        case QNODE_TEST:
            g_assert(test_node == node);
            /* Execute test */
            node->u.test.function(void_obj, node->u.test.arg, machine_a);

            /* After test */
            if (test_node->u.test.after) {
                test_node->u.test.after();
            }

            /* Cleanup */
            g_free(path);
            destroy_all_objects();
            empty_collector();
            return;

        default:
            break;
        }

        edge = qos_graph_get_edge(path[current], path[(current + 1)]);
        etype = qos_graph_get_edge_type(path[current], path[(current + 1)]);
        current++;
        node = qos_graph_get_node(path[current]);

        obj = void_obj;

        /* follow edge and get object for next node constructor */
        switch (etype) {
        case QEDGE_PRODUCES:
            void_obj = obj->get_driver(void_obj, path[current]);
            break;

        case QEDGE_CONSUMED_BY:
            has_to_allocate = 1;
            break;

        case QEDGE_CONTAINS:
            void_obj = obj->get_device(void_obj, path[current]);
            break;
        }
    }
}

/*
 * in this function, 2 path will be built:
 * str_path, a one-string path (ex "pc/i440FX-pcihost/...")
 * ro_path, a string-array path (ex [0] = "pc", [1] = "i440FX-pcihost").
 *
 * str_path will be only used to build the test name, and won't need the
 * architecture name at beginning, since it will be added by qtest_add_func().
 *
 * ro_path is used to allocate all constructors of the path nodes.
 * Each name in this array except position 0 must correspond to a valid
 * QOSGraphNode name.
 * Position 0 is special, initially contains just the <machine> name of
 * the node, (ex for "x86_64/pc" it will be "pc"), used to build the test
 * path (see below). After it will contain the command line used to start
 * qemu with all required devices.
 *
 * Note that the machine node name must be with format <arch>/<machine>
 * (ex "x86_64/pc"), because it will identify the node "x86_64/pc"
 * and start QEMU with "-M pc". For this reason,
 * when building str_path, ro_path
 * initially contains the <machine> at position 0 ("pc"),
 * and the node name at position 1 (<arch>/<machine>)
 * ("x86_64/pc"), followed by the rest of the nodes.
 */
static void walk_path(QOSGraphNode *orig_path, int len)
{
    QOSGraphNode *path;

    /* etype set to QEDGE_CONSUMED_BY so that machine can add command line */
    QOSEdgeType etype = QEDGE_CONSUMED_BY;

    /* twice QOS_PATH_MAX_ELEMENT_SIZE since each edge can have its arg */
    char **ro_path = g_new0(char *, (QOS_PATH_MAX_ELEMENT_SIZE * 2));
    int ro_path_size = 0;

    char *machine = NULL, *arch = NULL;
    char *after_cmd = NULL, *before_cmd = NULL;
    char *node_name = orig_path->name, *gfreed, *str_path;

    GString *cmd_line = g_string_new("");


    path = qos_graph_get_node(node_name); /* root */
    node_name = qos_graph_get_edge_dest(path->path_edge); /* machine name */

    qos_separate_arch_machine(node_name, &arch, &machine);
    ro_path[ro_path_size++] = arch;
    ro_path[ro_path_size++] = machine;

    do {
        path = qos_graph_get_node(node_name);
        node_name = qos_graph_get_edge_dest(path->path_edge);

        if (before_cmd) {
            g_string_append_printf(cmd_line, "%s ", before_cmd);
        }

        /* append node command line + previous edge command line */
        if (path->command_line && etype == QEDGE_CONSUMED_BY) {
            g_string_append(cmd_line, path->command_line);
            if (after_cmd) {
                g_string_append_printf(cmd_line, "%s ", after_cmd);
            }
        }

        ro_path[ro_path_size++] = qos_graph_get_edge_name(path->path_edge);
        /* detect if edge has command line args */
        after_cmd = qos_graph_get_edge_after_cmd_line(path->path_edge);
        before_cmd = qos_graph_get_edge_before_cmd_line(path->path_edge);
        etype = qos_graph_get_edge_type(path->name, node_name);

    } while (path->path_edge);


    /* here position 0 has <arch>/<machine>, position 1 <machine>.
     * the path must not have the <arch>, that's why ro_path  + 1
     */
    str_path = g_strjoinv("/", (ro_path + 1));
    gfreed = g_string_free(cmd_line, FALSE);
    /* put arch/machine in position 1 so allocate_objects can do its work
     * and add the command line at position 0.
     */
    ro_path[0] = g_strdup(gfreed);
    ro_path[1] = arch;

    qtest_add_data_func(str_path, ro_path, allocate_objects);

    g_free(str_path);
}



/**
 * main(): heart of the qgraph framework.
 *
 * - Initializes the glib test framework
 * - Creates the graph by invoking the various _init constructors
 * - Starts QEMU to mark the available devices
 * - Walks the graph, and each path is added to
 *   the glib test framework (walk_path)
 * - Runs the tests, calling allocate_object() and allocating the
 *   machine/drivers/test objects
 * - Cleans up everything
 */
int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qos_graph_init();
    module_call_init(MODULE_INIT_LIBQOS);
    qos_set_machines_devices_available();

    qos_graph_foreach_test_path(walk_path);
    g_test_run();
    qos_graph_destroy();
    return 0;
}
