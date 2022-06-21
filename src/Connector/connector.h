#ifndef __CONNECTOR_H__
#define __CONNECTOR_H__

#include "../Utility/utility.h"

/**
 * \struct Master_node
 * This structure is used to represent a node/cell with all its connections
 * to the other cells nearby.
 * The idea is that each cell in the environment is full described by its area
 * and the cells connected to it. Under this view, each cell is the master node
 * of a sector made up of cells direcly connected to it.
 * There are teo kinds of cells connection available:
 *  - adjacent ones: i.e. north, east, south and west.
 *    In this case it is assumed that there are two or more points in common.
 *  - diagonal_ones: i.e. north-east, south-east, south-west and north-west.
 *    In this case it is assumed that there is just one point in common.
 * 
 * Attributes:
 * @param master: polygon\*. It is a reference to the cell naming the node.
 * @param mean_area: double. It is the mean area of the cells connected to the
 *                           master node.
 * @param adjacent_connections: map<string, polygon\*>. The adjacent cells.
 * @param diagonal_connections: map<string, polygon\*>. The diagonal cells.
 */
typedef struct Master_node
{
    polygon* master = NULL;
    double mean_area = 0.0;
    map<string, polygon*> adjacent_connections;
    map<string, polygon*> diagonal_connections;
}Master_node;

/**
 * \struct Connection_map
 * This struct represents all the Master_node instances existing in the 
 * environment. The structure also provides the methods to menage the
 * connections among the elements.
 *
 * Attributes:
 * @param connections: map<string, Master_node>. It represents the relations
 *                     among the Master_node instances.
 *
 * Available methods are:
 * @see add_elemnt(polygon\* p): It is the method allowing to add a polygon to
 *                               the map of Master_node instances. Its
 *                               connections will be found automatically.
 * @see update_mean_area(string id): It is the method allowing to re-compute
 *                                   the area of a specific Master_node
 *                                   specified by its identifier.
 * @see unify(string to_update, string to_remove): It is the method allowing
 *      to merge two cells in one and also updates the connections of the 
 *      new element.
 * @see aggregate(): It is the method which will process the entire connection
 *                   map and will aggregate the smallest cells to the others
 *                   following a specified criterion (Usually area size).
 * @see embed_connections(string, to_update, string to_remove,
 *                        bool_use_diagonal): It is the method which will fix
 *                        the connections among merged cells.
 * @see global_mean_area(): It is the method which will compute the mean area
 *                          across the connection_map.
 * @see info(): This method it is used to print relevant informations about
 *              the connections. (i.e. master_node area, diagonal and adjacent
 *                                cells.)
 * @see min_max_element_area(string id, bool diagonal, bool min): It is the
 *      method which, given the id of a Master_node in the connections
 *      attribute, will return the element having the minimum/maximum area
 *      in its connections depending by the flag min in the arguments.
 * @see find_pddl_connections(): It is the method used to translate the
 *      environment representation contained in the structure into its 
 *      PDDL representation.
 * @see elements(): It is the method which will return a map containg the
 *      reference to the Master_node instances indexed by their id.
 */
typedef struct Connection_map
{
        map<string, Master_node> connections;

        void add_element(polygon* p);
        void update_mean_area(string id);
        void unify(string to_update, string to_remove);
        void aggregate();
        void embed_connections(string to_update, string to_remove);
        double global_mean_area();
        void info();
        string min_max_element_area(string id, bool diagonal=false,
                                    bool min=true);
        string find_pddl_connections();
        map<string, polygon*> elements();
} Connection_map;

#endif
