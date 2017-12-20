#ifndef DIJKSTRA_HPP_
#define DIJKSTRA_HPP_

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>                    //Biggest int: std::numeric_limits<int>::max()
#include "array_queue.hpp"
#include "array_stack.hpp"
#include "heap_priority_queue.hpp"
#include "hash_graph.hpp"


namespace ics {


class Info {
  public:
    Info() { }

    Info(std::string a_node) : node(a_node) { }

    bool operator==(const Info &rhs) const { return cost == rhs.cost && from == rhs.from; }

    bool operator!=(const Info &rhs) const { return !(*this == rhs); }

    friend std::ostream &operator<<(std::ostream &outs, const Info &i) {
      outs << "Info[" << i.node << "," << i.cost << "," << i.from << "]";
      return outs;
    }

    //Public instance variable definitions
    std::string node = "?";
    int cost = std::numeric_limits<int>::max();
    std::string from = "?";
  };


  bool gt_info(const Info &a, const Info &b) { return a.cost < b.cost; }

  typedef ics::HashGraph<int>                   DistGraph;
  typedef ics::HeapPriorityQueue<Info, gt_info> CostPQ;
  typedef ics::HashOpenMap<std::string, Info, ics::HashGraph<std::string>::hash_str>   CostMap;
  typedef ics::pair<std::string, Info>          CostMapEntry;

//Return the final_map as specified in the lecture-note description of
//  extended Dijkstra algorithm
  CostMap extended_dijkstra(const DistGraph &g, std::string start_node) {
        // Declare anwser_map to be empty
        /*
         * The info_map to contain each node
         in the graph as a key in the map, with its associated value a newly
         constructed/initialized object of Info for that node.
         */
        CostMap info_map;
        CostMap answer_map; // empty for now
        for(auto node : g.all_nodes()){
            info_map[node.first] = Info(node.first);
        }
        /*
         Update the start node in the info_map by setting its total_cost to 0
         (since we start at that node, the cost to reach it is 0).
         */
        info_map[start_node].cost = 0;

        CostPQ info_pq;
        // load info_pq with the contents of info_map
        for(auto el : info_map){
            info_pq.enqueue(el.second);
        }
        while(!info_map.empty()){
            Info v = info_pq.dequeue();
            if(v.cost == Info().cost){
                break;
            }
            while(answer_map.has_key(v.node)){
                v=info_pq.dequeue(); // dequeues first element
            }
            std::string min_node = v.node;
            int min_cost = v.cost;
            /* Remove this key->value from the info_map and put it into the
              answer_map.
             */
            info_map.erase(min_node);
            answer_map.put(min_node,Info(min_node));
            answer_map[min_node].cost = min_cost;
            answer_map[min_node].from = v.from;

            for(auto d : info_map){
                if(!answer_map.has_key(d.first)){
                    if(d.second.cost==Info().cost){
                    }
                    //int num = min_cost+g.edge_value(min_node,d.first);
                    if(g.has_edge(min_node,d.first)&& d.second.cost > (min_cost+g.edge_value(min_node,d.first))){
                        d.second.cost = (min_cost+g.edge_value(min_node,d.first));
                        d.second.from = min_node;
                        info_pq.enqueue(d.second);
                    }
//                    if(info_map[d].cost == std::numeric_limits<int>::max()|| info_map[d].cost > min_cost+g.edge_value(min_node,d)){
//                        info_map[d].cost = min_cost+g.edge_value(min_node,d);
//                        info_map[d].from = min_node;
//                        info_pq.enqueue(d);
//                    }
                }
            }

        }
        return answer_map;
    }




//Return a queue whose front is the start node (implicit in answer_map) and whose
//  rear is the end node
  ArrayQueue <std::string> recover_path(const CostMap &answer_map, std::string end_node) {
        // create ArrayQueue to return
        ArrayQueue<std::string> to_return;
        auto i = answer_map.begin();
        while(end_node !="?"){
            for(auto node : answer_map){
                if(node.first == end_node){
                    to_return.enqueue(end_node);
                    end_node = node.second.from;
                }
            }
        }

        return to_return;
  }


}

#endif /* DIJKSTRA_HPP_ */
