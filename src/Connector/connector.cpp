#include "connector.h"

/**
 * \fun update_mean_area(string id)
 * This method will force the re-computation of the mean area in the master
 * node sector.
 * @param id: string. It is the identifier of the Master_node instance which
 *            area has to be re-computed.
 */
void Connection_map::update_mean_area(string id)
{
    map<string, polygon *> connection_subset = connections[id].adjacent_connections;
    map<string, polygon *>::iterator upper_it = connection_subset.begin();

    double average_area = connections[id].master->area; // area of the master
    int elements = 1;
    while (upper_it != connection_subset.end())
    {
        average_area += upper_it->second->area;
        upper_it++;
        elements += 1;
    };
    if (connections[id].diagonal_connections.size() > 0)
    {
        upper_it = connections[id].diagonal_connections.begin();
        while (upper_it != connections[id].diagonal_connections.end())
        {
            average_area += upper_it->second->area;
            upper_it++;
            elements += 1;
        }
    }
    average_area /= elements;

    connections[id].mean_area = average_area;
};

/**
 * \fun add_element(polygon\* p)
 * This method allows to add an istance of type polygon\* to the list of
 * elements in connections with the Master_node. By default this element
 * will became a Master_node itself and will have a sector associated.
 * The sector is defined by the cells connected to the element.
 * Available connections are:
 *  - adjacent -> 2 or more points in common.
 *  - diagonal -> 1 point in common.
 * Master_node with no connections can exist.
 * When a new element is added the code will search its connections with all
 * the other elements in the structure and if a connection is found both
 * elements connections list will be updated.
 *
 * @param p: polygon\*. It is the addres of the element to add to the struct.
 */
void Connection_map::add_element(polygon *p)
{
    if (p == NULL)
    {
        cout << "Polygon " << p->id << " is NULL. Skipped." << endl;
        return;
    }
    if (connections.find(p->id) != connections.end())
    {
        cout << "Label ID already inside" << endl;
        return;
    }

    Master_node tmp;
    tmp.master = p;
    tmp.mean_area = p->area;
    connections[p->id] = tmp;

    Polygon_boost p_in_boost = p->to_boost_polygon();

    map<string, Master_node>::iterator c_it;
    for (c_it = connections.begin(); c_it != connections.end(); c_it++)
    {
        // Take master
        Polygon_boost tmp_boost = c_it->second.master->to_boost_polygon();
        if (boost::geometry::touches(p_in_boost, tmp_boost) &&
            c_it->first != p->id)
        {
            int common_points = p->points_in_common(c_it->second.master);
            // Only lateral connection allowed -> no diagonal
            if (common_points > 1)
            {
                // geometries touches->connected->update connections map
                // New entry update
                connections[p->id].adjacent_connections[c_it->first] = c_it->second.master;
                // Old entry update
                connections[c_it->first].adjacent_connections[p->id] = p;
            }
            else if (common_points == 1)
            {
                connections[p->id].diagonal_connections[c_it->first] = c_it->second.master;
                connections[c_it->first].diagonal_connections[p->id] = p;
            }
            update_mean_area(p->id);
            update_mean_area(c_it->first);
        };
    };
};

/**
 * \fun min_max_element_area(string id, bool diagonal, bool min)
 * This method returns the element in the connection lists of the Master_node,
 * addressed by the id, having the minimum/maximum area of the list.
 *
 * @param id: string. It is the string identifier of the Master_node.
 * @param diagonal: bool. It is a flag telling whether or not to consider the
 *                  cells in diagonal connection with the Master_node.
 *                  Default set to false.
 * @param min: bool. It is the flag telling whether or not to return the
 *             element which area is the minimum in the list. Default set to
 *             true.
 *
 * @return string: It is the identifier of the element satisying the
 *         requirements. If no element has been found the method returns "NaN".
 */
string Connection_map::min_max_element_area(string id, bool diagonal, bool min)
{
    string chosen = "NaN";
    Master_node node = connections[id];
    map<string, polygon *> connections_subset;

    if (diagonal == false)
    {
        connections_subset = node.adjacent_connections;
    }
    else
    {
        connections_subset = node.diagonal_connections;
    }

    if (connections_subset.size() > 0)
    {
        double area_min_max = 0;

        map<string, polygon *>::iterator supp_it = connections_subset.begin();

        map<double, string> values;
        while (supp_it != connections_subset.end())
        {
            point_node *centroid_node = node.master->centroid;
            point_node *cell_centroid = supp_it->second->centroid;

            if (centroid_node == NULL)
            {
                cout << node.master->id << " is NULL" << endl;
            }

            if (cell_centroid == NULL)
            {
                cout << supp_it->second->id << " is NULL" << endl;
            }

            double distance = centroid_node->distance(cell_centroid);

            //  If same value exist -> increment result with a small quantity
            //  to preserve order in exploration
            while (values.count(distance) == 1)
            {
                distance += 1e-4;
            }

            values[distance] = supp_it->first;
            supp_it++;
        }

        supp_it = connections_subset.begin();
        while (supp_it != connections_subset.end())
        {
            map<double, string>::iterator nearest = values.upper_bound(0);
            polygon *nearest_pol = connections_subset[nearest->second];
            if (min == true)
            {
                if (area_min_max == 0 ||
                    nearest_pol->area < area_min_max)
                {
                    chosen = nearest->second;
                    area_min_max = nearest_pol->area;
                }
            }
            else
            {
                if (area_min_max == 0 ||
                    nearest_pol->area > area_min_max)
                {
                    chosen = nearest->second;
                    area_min_max = nearest_pol->area;
                }
            }
            supp_it++;
            values.erase(nearest);
        }
    }
    return chosen;
}

/**
 * \fun embed_connections(string to_update, string to_remove)
 * This method is used to merge the connections among two Master_nodes which
 * will be merged together.
 *
 * @param to_update: string. It is the identifier of the element which will
 *        inherit the connections.
 * @param to_remove: string. It is the identifier of the element which will
 *        give the connections and will be merged.
 */
void Connection_map::embed_connections(string to_update, string to_remove)
{
    Master_node unified_node = connections[to_update]; // will include unify
    Master_node unify_node = connections[to_remove];   // included by unified

    map<string, polygon *> unified;
    map<string, polygon *> to_unify;
    map<string, polygon *>::iterator unify_it;
    map<string, polygon *> check;

    // Do embedding by the adjacent connections
    unified = unified_node.adjacent_connections;
    to_unify = unify_node.adjacent_connections;
    unify_it = to_unify.begin();

    if (to_unify.size() > 0)
    {
        // Parse the connections of the node and include the unkown ones
        while (unify_it != to_unify.end())
        {
            // if includer node does not have the connection, add it
            // also avoid to include the connection to itself
            if (unified.count(unify_it->first) == 0 &&
                unify_it->first != to_update && unify_it->first != to_remove)
            {
                unified[unify_it->first] = unify_it->second;
            }

            // Update the connections in the cells connected to the one
            // to remove
            check = connections[unify_it->first].adjacent_connections;
            check.erase(to_remove); // remove included cell id

            // Added includer id to the cell -> inheritance of connection
            // due to union
            if (check.count(to_update) == 0)
            {
                check[to_update] = unified_node.master;
            }

            // update the connections in the connection map
            connections[unify_it->first].adjacent_connections = check;

            // Update the mean area of the cell due to the change in its
            // connections
            update_mean_area(unify_it->first);

            // Go to next connection in the included cell
            unify_it++;
        }
        // remove included cell id from the connections of the includer
        // It is safe now -> all connections have been fixed.
        unified.erase(to_remove);

        // Update the coy connections
        unified_node.adjacent_connections = unified;

        // Update the connection map master node of the includer
        connections[to_update] = unified_node;
    }

    // Do embedding for the diagonal connections
    unified = unified_node.diagonal_connections;
    to_unify = unify_node.diagonal_connections;
    unify_it = to_unify.begin();

    if (to_unify.size() > 0)
    {
        // Parse the connections of the node and include the unkown ones
        while (unify_it != to_unify.end())
        {
            // if includer node does not have the connection, add it
            // also avoid to include the connection to itself
            if (unified.count(unify_it->first) == 0 &&
                unify_it->first != to_update && unify_it->first != to_remove)
            {
                unified[unify_it->first] = unify_it->second;
            }

            // Update the connections in the cells connected to the one
            // to remove
            check = connections[unify_it->first].diagonal_connections;
            check.erase(to_remove); // remove included cell id

            // Added includer id to the cell -> inheritance of connection
            // due to union
            if (check.count(to_update) == 0)
            {
                check[to_update] = unified_node.master;
            }

            // update the connections in the connection map
            connections[unify_it->first].diagonal_connections = check;

            // Update the mean area of the cell due to the change in its
            // connections
            update_mean_area(unify_it->first);

            // Go to next connection in the included cell
            unify_it++;
        }
        // remove included cell id from the connections of the includer
        // It is safe now -> all connections have been fixed.
        unified.erase(to_remove);

        // Update the coy connections
        unified_node.diagonal_connections = unified;

        // Update the connection map master node of the includer
        connections[to_update] = unified_node;
        update_mean_area(to_update);
    }
}

/**
 * \fun unify(string to_update, string to_remove)
 * This method will phisically merge two Master_nodes and also handles their
 * connections.
 *
 * @param to_update: string. It is the identifier of the element which will
 *        inherit everything from the second argument.
 * @param to_remove: string. It is the identifier of the element which will
 *        give all the connections and that willbe merged.
 */
void Connection_map::unify(string to_update, string to_remove)
{
    Master_node *unified_node = &connections[to_update];
    Master_node *unify_node = &connections[to_remove];

    // Update resulting polygon
    // Unify the polygons
    vector<Polygon_boost> output;
    if (unified_node->master == NULL)
    {
        cout << "Polygon: " << to_update << " is null" << endl;
    }

    if (unify_node->master == NULL)
    {
        cout << "Polygon: " << to_remove << " is null" << endl;
    }

    Polygon_boost unified_boost = unified_node->master->to_boost_polygon();

    Polygon_boost to_unify_boost = unify_node->master->to_boost_polygon();

    boost::geometry::union_(unified_boost, to_unify_boost, output);
    polygon *result = boost_polygon_to_polygon(output[output.size() - 1]);

    if (result == NULL)
    {
        cout << to_update << " is NULL" << endl;
    }

    result->id = to_update;
    // fixed in method boost_polygon_to_polygon

    // Update union
    unified_node->master = result;

    // merge connections
    embed_connections(to_update, to_remove);

    // Update average area
    update_mean_area(to_update);

    // Delete entry from map
    connections.erase(to_remove);
};

/**
 * \fun aggregate()
 * This method will process the entire connections attribute and merge
 * together all the Master_nodes which area is small than half the global area.
 */
void Connection_map::aggregate()
{
    map<string, Master_node>::iterator c_it;
    vector<string> valid_id;

    for (c_it = connections.begin(); c_it != connections.end(); c_it++)
    {
        valid_id.push_back(c_it->first);
    }

    int available_id = valid_id.size();
    double global_area = global_mean_area();
    
    for (int i = 0; i < available_id; i++)
    {
        string id = valid_id[i];
        double master_area = connections[id].master->area;

        if (master_area < 0.5 * global_area) //*master_sector_area)
        {      
            string includer = "NaN";

            // Check over adjacent cells -> higher priority
            includer = min_max_element_area(id, false, true);

            if (includer == "NaN") // if doesn't work rely on diagonals
            {
                includer = min_max_element_area(id, true, true);
            }

            if (includer == "NaN") // Cell with no connections
            {
                cout << id << " has nothing to merge with" << endl;
                return;
            }
            else
            {
                unify(includer, id);
                available_id--;
                valid_id.erase(valid_id.begin() + i);
                i--;
            }
        }
    }
};

/**
 * \fun global_mean_area()
 * This method will compute the mean area over the Master_node instances
 * added to the structure.
 *
 * @return mean: The global mean area.
 */
double Connection_map::global_mean_area()
{
    double average = 0;
    map<string, Master_node>::iterator c_it;
    for (c_it = connections.begin(); c_it != connections.end(); c_it++)
    {
        average += c_it->second.mean_area;
    };
    
    if(connections.size() > 0)
    {
        average /= connections.size();
    }
    return average;
};


/**
 * \fun info()
 * This method will print the connection lists of each Master_node and also
 * their identifier and area.
 * The global mean_area will be also shown.
 */
void Connection_map::info()
{
    cout << "Elements in connection_map: " << connections.size() << endl;
    map<string, Master_node>::iterator c_it;
    for (c_it = connections.begin(); c_it != connections.end(); c_it++)
    {
        cout << c_it->first << ": " << endl;
        cout << "\t- Self Area: " << c_it->second.master->area << endl;
        cout << "\t- Sector Area: " << c_it->second.mean_area << endl;
        cout << "\t- Adjacent: ";

        map<string, polygon *> subset = c_it->second.adjacent_connections;
        map<string, polygon *>::iterator conn_iter = subset.begin();
        while (conn_iter != subset.end())
        {
            cout << conn_iter->first;
            conn_iter++;

            if (conn_iter != subset.end())
            {
                cout << " <-> ";
            }
        };
        cout << endl;

        subset = c_it->second.diagonal_connections;
        if (subset.size() > 0)
        {
            map<string, polygon *>::iterator conn_iter = subset.begin();
            cout << "\t- Diagonal: ";
            while (conn_iter != subset.end())
            {
                cout << conn_iter->first;
                conn_iter++;

                if (conn_iter != subset.end())
                {
                    cout << " <-> ";
                }
            };
            cout << endl;
        }
    };
    cout << "Global average: " << global_mean_area() << endl;
    cout << endl;
};

/**
 * \fun elements()
 * This method is used to return a reference to all the elements existing
 * int the struct.
 *
 * @return map<string, polygon\*>: The reference of the elements addressed by
 *                                 their identifiers.
 */
map<string, polygon *> Connection_map::elements()
{
    map<string, polygon *> elements;
    map<string, Master_node>::iterator c_it;
    for (c_it = connections.begin(); c_it != connections.end(); c_it++)
    {
        elements[c_it->first] = c_it->second.master;
    };
    return elements;
};

/**
 * \fun find_pddl_conenctions()
 * This method will transform the existing connections and Master_node instance
 * into a PDDL code suitable for the task.
 * The PDDL code will just express which Master_node instances are connected
 * together and the kind of connection.
 *
 * @return string: It is the PDDL piece of code resulting from the
 *                 transformation.
 */
string Connection_map::find_pddl_connections()
{
    typedef map<string, polygon *>::const_iterator conn_it;
    map<string, Master_node>::const_iterator c_it = connections.cbegin();
    string pddl_connections = "";
    while (c_it != connections.end())
    {
        // Write adjacent connections  first
        conn_it tmp_it = c_it->second.adjacent_connections.cbegin();
        while (tmp_it != c_it->second.adjacent_connections.cend())
        {
            pddl_connections += "\t\t( connected " + c_it->first +
                                " " + tmp_it->first + " )\n";
            tmp_it++;
        }

        tmp_it = c_it->second.diagonal_connections.cbegin();
        while (tmp_it != c_it->second.diagonal_connections.cend())
        {
            pddl_connections += "\t\t( is_diagonal " + c_it->first +
                                " " + tmp_it->first + " )\n";
            tmp_it++;
        }
        c_it++;
    }
    cout << pddl_connections << endl;
    return pddl_connections;
};
