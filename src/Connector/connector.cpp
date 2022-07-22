#include "connector.h"

/**
 * \fun Connection_map(logger *log)
 * Connection_map costructor
 * @param log: logger. File logger
 */

Connection_map::Connection_map(logger *_log)
{
    log = _log;
}

/**
 * \fun to_log(string msg)
 * log function.
 * @param msg: string. Messagge to write in the log file.
 */
void Connection_map::to_log(string msg)
{
    log->add_event("Connection map: " + msg);
}

/**
 * \fun connection_ids(bool diagonal)
 * This method return all the adjacent cells with the Master_node.
 * @param diagonal: bool. Flag to get all the adjacent diagonal cell's id.
 * @return vector<string>: vector with all the ids 
 */
vector<string> Master_node::connection_ids(bool diagonal)
{
    map<string, polygon *>::const_iterator it_s;
    map<string, polygon *>::const_iterator it_e;
    vector<string> ids;

    if (diagonal == false)
    {
        it_s = adjacent_connections.cbegin();
        it_e = adjacent_connections.cend();
    }
    else
    {
        it_s = diagonal_connections.cend();
        it_e = diagonal_connections.cend();
    }

    while (it_s != it_e)
    {
        ids.push_back(it_s->first);
        it_s++;
    }
    return ids;
}

/**
 * \fun ids()
 * This method returns all the Master_node ids.
 * @return vector<string>: All the Master_node ids. 
 */
vector<string> Connection_map::ids()
{
    map<string, Master_node>::const_iterator it = connections.cbegin();
    vector<string> ids;

    while (it != connections.cend())
    {   
        cout << "ID: " << it->first << endl;
        ids.push_back(it->first);
        it++;
    }
    return ids;
}

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

    // cout << "Converted " << p->id << " to boost" << endl;

    map<string, Master_node>::iterator c_it;
    for (c_it = connections.begin(); c_it != connections.end(); c_it++)
    {
        // Take master
        // cout << "Evaluating " << c_it->first << endl;
        if (c_it->second.master == NULL)
        {
            cout << c_it->first << " Is NULL" << endl;
        }
        Polygon_boost tmp_boost = c_it->second.master->to_boost_polygon();

        // cout << "Converted " << c_it->second.master->id << " to boost" << endl;

        if (boost::geometry::touches(p_in_boost, tmp_boost) &&
            c_it->first != p->id)
        {
            int common_points = p->points_in_common(c_it->second.master);

            // cout << p->id << " and " << c_it->first << " have " << common_points

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
    // vector<Polygon_boost> output;
    if (unified_node->master == NULL)
    {
        cout << "Polygon: " << to_update << " is null" << endl;
    }

    if (unify_node->master == NULL)
    {
        cout << "Polygon: " << to_remove << " is null" << endl;
    }

    // Polygon_boost unified_boost = unified_node->master->to_boost_polygon();

    // Polygon_boost to_unify_boost = unify_node->master->to_boost_polygon();

    // boost::geometry::union_(unified_boost, to_unify_boost, output);
    // polygon *result = boost_polygon_to_polygon(output[output.size() - 1]);

    polygon *result = merge(unified_node->master, unify_node->master);

    if (result == NULL)
    {
        cout << to_update << " is NULL" << endl;
        return;
    }

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
    cout << "Called aggregator" << endl;
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

        if (master_area < AGGREGATE * global_area) //*master_sector_area)
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

    if (connections.size() > 0)
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
    to_log("Started finding PDDL connections");

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
            pddl_connections += "\t\t( connected " + c_it->first +
                                " " + tmp_it->first + " )\n"
                                                      "\t\t( is_diagonal " +
                                c_it->first +
                                " " + tmp_it->first + " )\n";
            tmp_it++;
        }
        c_it++;
    }
    to_log("Ended Finding PDDL connections");
    return pddl_connections;
};

/**
 * \fun erase_MasterNode(string id)
 * This method delete a Master node with the given id.
 * @param id: string. Cell id to delete. 
 */

void Connection_map::erase_MasterNode(string id)
{
    if (connections.count(id) == 0)
    {
        to_log("No Master Node " + id + " found. Nothing to remove.");
        return;
    }

    map<string, polygon *>::iterator it;
    if (connections[id].adjacent_connections.size() > 0)
    {
        it = connections[id].adjacent_connections.begin();
        while (it != connections[id].adjacent_connections.end())
        {
            connections[it->first].adjacent_connections.erase(id);
            connections[it->first].diagonal_connections.erase(id);
            it++;
        }
    }

    if (connections[id].diagonal_connections.size() > 0)
    {
        it = connections[id].diagonal_connections.begin();
        while (it != connections[id].diagonal_connections.end())
        {
            connections[it->first].adjacent_connections.erase(id);
            connections[it->first].diagonal_connections.erase(id);
            it++;
        }
    }
    connections.erase(id);
}

/**
 * \fun ensure_LOS(list_of_obstacles *ob_l)
 * This method ensures that each cells is in line of sight with it's adjacents. If not subcells 
 * are created.
 * @param ob_l: list_of_obstacles. List of obstacles to use to see if two cells are in LOS.
 */
void Connection_map::ensure_LOS(list_of_obstacles *ob_l)
{
    cout << "Evaluating LOS" << endl;
    if (ob_l == NULL)
    {
        to_log("Obstacle list provided is NULL.");
        return;
    }

    vector<string> available_id = ids();

    int i = 0;
    int id_size = available_id.size();
    while (i < id_size)
    {
        Master_node *node = &connections[available_id[i]];
        vector<string> conn_id = node->connection_ids();
        bool LOS_point = true;
        int j = 0;
        while (j < conn_id.size() && LOS_point == true)
        {

            polygon *second_cell = node->adjacent_connections[conn_id[j]];
            Edge *comm_edge = find_common_edge(node->master,
                                               second_cell);

            polygon *ob = ob_l->offset_head;
            while (ob != NULL && LOS_point == true)
            {
                LOS_point = los(node->master->centroid,
                                second_cell->centroid,
                                ob);
                ob = ob->pnext;
            }

            if (LOS_point == false)
            {
                cout << endl;

                list_of_polygons *pl = new list_of_polygons;
                pl->append_other_list(subset_over_middle_point(node->master));

                pl->append_other_list(subset_over_middle_point(second_cell));

                int new_master_nodes = pl->size;

                erase_MasterNode(available_id[i]);
                erase_MasterNode(conn_id[j]);

                available_id.erase(available_id.begin() + i);
                i -= 1;

                int del_idx = -1;
                int k = 0;

                while (k < available_id.size() && del_idx == -1)
                {
                    if (available_id[k] == conn_id[j])
                    {
                        del_idx = k;
                    }
                    k++;
                }

                if (del_idx != -1)
                {
                    available_id.erase(available_id.begin() + del_idx);
                }

                polygon *tmp = pl->head;

                while (tmp != NULL)
                {
                    add_element(tmp->copy());
                    tmp = tmp->pnext;
                }

                id_size = available_id.size();
            }
            j++;
        }
        i++;
    }

    // Inefficient code
    vector<Polygon_boost> ob;
    vector<Polygon_boost> pols;

    polygon *ob_temp = ob_l->offset_head;
    while (ob_temp != NULL)
    {
        ob.push_back(ob_temp->to_boost_polygon());
        ob_temp = ob_temp->pnext;
    }

    map<string, polygon *> els = elements();
    map<string, polygon *>::const_iterator el_it = els.cbegin();
    while (el_it != els.cend())
    {
        Polygon_boost cell = el_it->second->to_boost_polygon();
        Polygon_boost output;
        boost::geometry::convex_hull(cell, output);
        pols.push_back(output);
        el_it++;
    }

    pols = difference_of_vectors(pols, ob);
    empty();
    string base_id = "CELL_";
    for (int i = 0; i < pols.size(); i++)
    {
        add_element(boost_polygon_to_polygon(pols[i], base_id + to_string(i)));
    }
}

string Connection_map::make_cells_predicates()
{
    to_log("Started making cells PDDL predicates");
    vector<string> cells_id = ids();
    if (cells_id.size() > 0)
    {
        string cells_predicates = "";
        for(int i=0; i<cells_id.size(); i++)
        {
            cells_predicates += "\t\t(is_" + cells_id[i] + " ?c - location)\n";
        }
        return cells_predicates;
    }
    to_log("Ended making cells PDDL predicates");
    return "NaN";
}

string Connection_map::make_cells_conditional_distances(string cost_name)
{
    to_log("Started computing cells distances into PDDL conditional effects.");
    string conditional_distances = "\n\t\t\t\t; Cells distances costs\n";
    map<string, Master_node>::const_iterator c_it = connections.cbegin();
    map<string, polygon*>::const_iterator conn_it_start;
    map<string, polygon*>::const_iterator conn_it_end;

    map<string, double> computed_distances;

    while(c_it != connections.cend())
    {
        point_node * master_centroid = c_it->second.master->centroid;
        
        // Compute distance for adjacent cells
        vector< map<string, polygon*> * > conns;
        conns.push_back(&connections[c_it->first].adjacent_connections);
        conns.push_back(&connections[c_it->first].diagonal_connections);
        for(int i=0; i<conns.size(); i++)
        {
            conn_it_start = conns[i]->cbegin();
            conn_it_end = conns[i]->cend();
            while(conn_it_start != conn_it_end)
            {  
                string couple = conn_it_start->first + "_" + c_it->first;
                string couple_simm = c_it->first + "_" + conn_it_start->first;
                if (computed_distances.count(couple) == 0 &&
                    computed_distances.count(couple_simm) == 0)
                {
                    point_node * cell_centroid = conn_it_start->second->centroid; 
                    double distance = master_centroid->distance(cell_centroid);
                    string cost = PDDL_conditional_cost(c_it->first,
                                                        conn_it_start->first,
                                                        distance,
                                                        cost_name);
                    conditional_distances += cost;
                    computed_distances[couple] = distance;
                }
                conn_it_start++;
            }
        }
        c_it++;
    }
    to_log("Ended computing cells distances into PDDL conditional effects.");
    return conditional_distances;
}

void Connection_map::overwrite(map<string, Master_node> new_connections)
{
    to_log("Overwriting connections");
    connections = new_connections;
}

void Connection_map::empty()
{
    to_log("Starting emptying structure");
    connections.clear();
    to_log("Structure emptied.");
}
