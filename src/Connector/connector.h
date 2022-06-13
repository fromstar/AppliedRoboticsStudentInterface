#ifndef __CONNECTOR_H__
#define __CONNECTOR_H__

#include "../Utility/utility.h"

typedef struct Master_node
{
    polygon* master = NULL;
    double mean_area = 0.0;
    map<string, polygon*> adjacent_connections;
    map<string, polygon*> diagonal_connections;

}Master_node;

typedef struct Connection_map
{
    // Unordered_map ensured that the first element is the cell giving key
        // map<string, pair<double, unordered_map<string, polygon*>>> connections;
        
        map<string, Master_node> connections;

        void add_element(polygon* p);
        void update_mean_area(string id);
        void unify(string to_update, string to_remove);
        void aggregate();
        void embed_connections(string to_update, string to_remove,
                               bool use_diagonal=false);
        double global_mean_area();
        void info();
        string min_max_element_area(string id, bool diagonal=false,
                                    bool min=true);
        string find_pddl_connections();
        map<string, polygon*> elements();
} Connection_map;

#endif
