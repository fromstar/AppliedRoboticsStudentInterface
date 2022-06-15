#include "connector.h"

void Connection_map::update_mean_area(string id)
{
    // unordered_map<string, polygon*> connections_subset = connections[id].second;
    // unordered_map<string, polygon*>::iterator upper_it = connections_subset.begin();
    map<string, polygon *> connection_subset = connections[id].adjacent_connections;
    map<string, polygon *>::iterator upper_it = connection_subset.begin();

    double average_area = connections[id].master->area; // area of the master
    int elements = 0;
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

    /*
    unordered_map<string, polygon*> tmp_m;
    tmp_m[p->id] = p;
    connections[p->id] = make_pair(p->area, tmp_m);
    */
    Master_node tmp;
    tmp.master = p;
    tmp.mean_area = p->area;
    connections[p->id] = tmp;

    // cout << "Added polygon: " << p->id << endl;

    Polygon_boost p_in_boost = p->to_boost_polygon();

    // map<string, pair<double, unordered_map<string, polygon*>>>::iterator c_it;
    map<string, Master_node>::iterator c_it;
    for (c_it = connections.begin(); c_it != connections.end(); c_it++)
    {
        // Take master
        Polygon_boost tmp_boost = c_it->second.master->to_boost_polygon();
        if (boost::geometry::touches(p_in_boost, tmp_boost))
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

string Connection_map::min_max_element_area(string id, bool diagonal, bool min)
{
    // string id = "NaN";
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
                    // supp_it->second->area < area_min_max
                    nearest_pol->area < area_min_max)
                {
                    chosen = nearest->second; // supp_it->first;
                    // area_min_max = supp_it->second->area;
                    area_min_max = nearest_pol->area;
                }
            }
            else
            {
                if (area_min_max == 0 ||
                    // supp_it->second->area > area_min_max
                    nearest_pol->area > area_min_max)
                {
                    chosen = nearest->second;         // supp_it->first;
                    area_min_max = nearest_pol->area; // supp_it->second->area;
                }
            }
            supp_it++;
            values.erase(nearest);
        }
    }
    return chosen;
}

void Connection_map::embed_connections(string to_update, string to_remove,
                                       bool use_diagonal)
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

void Connection_map::unify(string to_update, string to_remove)
{
    // map<string, pair<double, unordered_map<string, polygon*>>>::iterator c_it;
    // unordered_map<string, polygon*> unified = connections[to_update].second;
    // unordered_map<string, polygon*> to_unify = connections[to_remove].second;
    // unordered_map<string, polygon*>::iterator unify_it = to_unify.begin();

    // map<string, Master_node>::iterator c_it;
    Master_node *unified_node = &connections[to_update];
    Master_node *unify_node = &connections[to_remove];

    // map<string, polygon*> *unified = &unified_node.adjacent_connections;
    // map<string, polygon*> *to_unify = &unify_node.adjacent_connections;
    // map<string, polygon*>::iterator unify_it = to_unify->begin();

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
    // result->area = boost::geometry::area(output[output.size()-1]);

    // Update union
    // delete unified_node->master;  // free memory
    unified_node->master = result;

    // merge connections
    embed_connections(to_update, to_remove);

    // Update average area
    update_mean_area(to_update);

    // Delete entry from map
    connections.erase(to_remove);
};

void Connection_map::aggregate()
{
    map<string, Master_node>::iterator c_it;
    vector<string> valid_id;

    for (c_it = connections.begin(); c_it != connections.end(); c_it++)
    {
        valid_id.push_back(c_it->first);
    }

    int available_id = valid_id.size();
    // for(c_it = connections.begin(); c_it != connections.end(); c_it++)
    for (int i = 0; i < available_id; i++)
    {
        string id = valid_id[i];
        // double master_sector_area = connections[id].mean_area;
        double master_area = connections[id].master->area;
        double global_area = global_mean_area();

        if (master_area < 0.65 * global_area) //*master_sector_area)
        {
            string includer = "NaN";

            // Check over adjacent cells -> higher priority
            // includer = min_max_element_area(connections[id].adjacent_connections,
            //                                 false);
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

double Connection_map::global_mean_area()
{
    double average = 0;
    // map<string, pair<double, unordered_map<string, polygon*>>>::iterator c_it;
    map<string, Master_node>::iterator c_it;
    for (c_it = connections.begin(); c_it != connections.end(); c_it++)
    {
        average += c_it->second.mean_area;
    };
    average /= connections.size();
    return average;
};

void Connection_map::info()
{
    cout << "Elements in connection_map: " << connections.size() << endl;
    // map<string, pair<double, unordered_map<string, polygon*>>>::iterator c_it;
    map<string, Master_node>::iterator c_it;
    for (c_it = connections.begin(); c_it != connections.end(); c_it++)
    {
        cout << c_it->first << ": " << endl;
        cout << "\t- Self Area: " << c_it->second.master->area << endl;
        cout << "\t- Sector Area: " << c_it->second.mean_area << endl;
        cout << "\t- Adjacent: ";

        // unordered_map<string, polygon*> connections_subset = c_it->second.second;
        // unordered_map<string, polygon*>::iterator in_it = connections_subset.begin();
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

map<string, polygon *> Connection_map::elements()
{
    map<string, polygon *> elements;
    // map<string, pair<double, unordered_map<string, polygon*>>>::iterator c_it;
    map<string, Master_node>::iterator c_it;
    for (c_it = connections.begin(); c_it != connections.end(); c_it++)
    {
        // string tmp_string = c_it->first;
        // polygon* tmp_pol = c_it->second.second.begin()->second;
        // elements[tmp_string] = tmp_pol;

        elements[c_it->first] = c_it->second.master;
    };
    return elements;
};

string Connection_map::find_pddl_connections()
{
    typedef map<string, polygon *>::iterator conn_it;
    map<string, Master_node>::iterator c_it = connections.begin();
    string pddl_connections = "";
    while (c_it != connections.end())
    {
        // Write adjacent connections  first
        conn_it tmp_it = c_it->second.adjacent_connections.begin();
        while (tmp_it != c_it->second.adjacent_connections.end())
        {
            pddl_connections += "\t\t( connected " + c_it->first +
                                " " + tmp_it->first + " )\n";
            tmp_it++;
        }

        tmp_it = c_it->second.diagonal_connections.begin();
        while (tmp_it != c_it->second.diagonal_connections.end())
        {
            pddl_connections += "\t\t( is_diagonal " + c_it->first +
                                " " + tmp_it->first + " )\n";
            tmp_it++;
        }
        c_it++;
    }
    return pddl_connections;
};
