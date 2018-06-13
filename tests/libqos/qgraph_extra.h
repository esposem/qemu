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

#ifndef QGRAPH_EXTRA_H
#define QGRAPH_EXTRA_H

/* This header is declaring additional helper functions defined in
 * libqos/qgraph.c
 * It should not be included in tests
 */

#include "libqos/qgraph.h"

typedef struct QOSGraphMachine QOSGraphMachine;
typedef struct QOSGraphEdgeList QOSGraphEdgeList;
typedef enum QOSEdgeType QOSEdgeType;
typedef enum QOSNodeType QOSNodeType;

/* callback called when the walk path algorithm found a
 * valid path
 */
typedef void (*QOSTestCallback) (QOSGraphNode *path, int len);

/* edge types*/
enum QOSEdgeType {
    QEDGE_CONTAINS,
    QEDGE_PRODUCES,
    QEDGE_CONSUMED_BY
};

/* node types*/
enum QOSNodeType {
    QNODE_MACHINE,
    QNODE_DRIVER,
    QNODE_INTERFACE,
    QNODE_TEST
};

/* Graph Node */
struct QOSGraphNode {
    QOSNodeType type;
    bool available;     /* set by QEMU via QMP, used during graph walk */
    bool visited;       /* used during graph walk */
    char *name;         /* used to identify the node */
    char *command_line; /* used to start QEMU at test execution */
    union {
        struct {
            QOSBeforeDriver before; /* used when the driver needs additional
                                     * command line mixed with file descriptors
                                     * or reference made by invoking functions
                                     */
            QOSCreateDriverFunc constructor;
        } driver;
        struct {
            QOSCreateMachineFunc constructor;
        } machine;
        struct {
            QOSTestFunc function;
            void *arg;
            QOSBeforeTest before;
            QOSAfterTest after;
        } test;
    } u;

    /**
     * only used when traversing the path, never rely on that except in the
     * qos_traverse_graph callback function
     */
    QOSGraphEdge *path_edge;
};

/**
 * qos_graph_get_node(): returns the node mapped to that @key.
 * It performs an hash map search O(1)
 *
 * Returns: on success: the %QOSGraphNode
 *          otherwise: #NULL
 */
QOSGraphNode *qos_graph_get_node(const char *key);

/**
 * qos_graph_has_node(): returns #TRUE if the node
 * has map has a node mapped to that @key.
 */
bool qos_graph_has_node(const char *node);

/**
 * qos_graph_get_node_type(): returns the %QOSNodeType
 * of the node @node.
 * It performs an hash map search O(1)
 * Returns: on success: the %QOSNodeType
 *          otherwise: #-1
 */
QOSNodeType qos_graph_get_node_type(const char *node);

/**
 * qos_graph_get_node_availability(): returns the availability (boolean)
 * of the node @node.
 */
bool qos_graph_get_node_availability(const char *node);

/**
 * qos_graph_get_edge(): returns the edge
 * linking of the node @node with @dest.
 *
 * Returns: on success: the %QOSGraphEdge
 *          otherwise: #NULL
 */
QOSGraphEdge *qos_graph_get_edge(const char *node, const char *dest);

/**
 * qos_graph_get_edge_type(): returns the edge type
 * of the edge linking of the node @start with @dest.
 *
 * Returns: on success: the %QOSEdgeType
 *          otherwise: #-1
 */
QOSEdgeType qos_graph_get_edge_type(const char *start, const char *dest);

/**
 * qos_graph_get_edge_dest(): returns the name of the node
 * pointed as destination of edge @edge.
 *
 * Returns: on success: the destination
 *          otherwise: #NULL
 */
char *qos_graph_get_edge_dest(QOSGraphEdge *edge);

/**
 * qos_graph_has_edge(): returns #TRUE if there
 * exists an edge from @start to @dest.
 */
bool qos_graph_has_edge(const char *start, const char *dest);

/**
 * qos_graph_get_edge_arg(): returns the args assigned
 * to that @edge.
 *
 * Returns: on success: the arg
 *          otherwise: #NULL
 */
void *qos_graph_get_edge_arg(QOSGraphEdge *edge);

/**
 * qos_graph_get_edge_after_cmd_line(): returns the arg
 * command line that will be added after the node cmd
 * line.
 *
 * Returns: on success: the char* arg
 *          otherwise: #NULL
 */
char *qos_graph_get_edge_after_cmd_line(QOSGraphEdge *edge);

/**
 * qos_graph_get_edge_before_cmd_line(): returns the arg
 * command line that will be added before the node cmd
 * line.
 *
 * Returns: on success: the char* arg
 *          otherwise: #NULL
 */
char *qos_graph_get_edge_before_cmd_line(QOSGraphEdge *edge);

/**
 * qos_graph_get_edge_name(): returns the name
 * assigned to the destination node (different only)
 * if there are multiple devices with the same node name
 * e.g. a node has two "generic-sdhci", "emmc" and "sdcard"
 * there will be two edges with edge_name ="emmc" and "sdcard"
 *
 * Returns always the char* edge_name
 */
char *qos_graph_get_edge_name(QOSGraphEdge *edge);

/**
 * qos_graph_get_machine(): returns the machine assigned
 * to that @node name.
 *
 * It performs a search only trough the list of machines
 * (i.e. the QOS_ROOT child).
 *
 * Returns: on success: the %QOSGraphNode
 *          otherwise: #NULL
 */
QOSGraphNode *qos_graph_get_machine(const char *node);

/**
 * qos_graph_has_machine(): returns #TRUE if the node
 * has map has a node mapped to that @node.
 */
bool qos_graph_has_machine(const char *node);


/**
 * qos_print_graph(): walks the graph and prints
 * all machine-to-test paths.
 */
void qos_print_graph(void);

/**
 * qos_graph_foreach_test_path(): executes the Depth First search
 * algorithm and applies @fn to all discovered paths.
 *
 * See qos_traverse_graph() in qgraph.c for more info on
 * how it works.
 */
void qos_graph_foreach_test_path(QOSTestCallback fn);

/**
 * qos_destroy_object(): calls the destructor for @obj
 */
void qos_destroy_object(QOSGraphObject *obj);

/**
 * qos_separate_arch_machine(): separate arch from machine.
 * This function requires every machine @name to be in the form
 * <arch>/<machine_name>, like "arm/raspi2" or "x86_64/pc".
 *
 * The function will split then the string in two parts,
 * assigning @arch to point to <arch>/<machine_name>, and
 * @machine to <machine_name>.
 *
 * For example, "x86_64/pc" will be split in this way:
 * *arch = "x86_64/pc"
 * *machine = "pc"
 *
 * Note that this function *does not* allocate any new string,
 * but just sets the pointer *arch and *machine to the respective
 * part of the string.
 */
void qos_separate_arch_machine(char *name, char **arch, char **machine);

/**
 * qos_delete_abstract_cmd_line(): if @abstract is #TRUE, delete the
 * command line present in node mapped with key @name.
 *
 * This function is called when the QMP query returns a node with
 * { "abstract" : <boolean> } attribute.
 */
void qos_delete_abstract_cmd_line(const char *name, bool abstract);

/**
 * qos_graph_node_set_availability(): sets the node identified
 * by @node with availability @av.
 */
void qos_graph_node_set_availability(const char *node, bool av);

#endif
