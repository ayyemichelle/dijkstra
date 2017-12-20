#include <string>
#include <iostream>
#include <fstream>
#include "ics46goody.hpp"
#include "array_queue.hpp"
#include "hash_graph.hpp"
#include "dijkstra.hpp"



std::string get_node_in_graph(const ics::DistGraph& g, std::string prompt, bool allow_QUIT) {
  std::string node;
  for(;;) {
    node = ics::prompt_string(prompt + " (must be in graph" + (allow_QUIT ? " or QUIT" : "") + ")");
    if ( (allow_QUIT && node == "QUIT") || g.has_node(node) )
      break;
  }
  return node;
}


int main() {
  try {
      std::ifstream in_file;
      ics::HashGraph<int> graph;
      ics::CostMap answer;
      ics::safe_open(in_file, "Enter graph file name", "flightcost.txt");
      graph.load(in_file, ";");
      //std::cout << graph.all_nodes() << std::endl;
      std::cout << graph << std::endl;
      std::string start = get_node_in_graph(graph, "Enter start node", false);
      answer = extended_dijkstra(graph, start);
      std::cout << answer << std::endl;
      std::string end = get_node_in_graph(graph, "Enter stop node", true);
      while(end != "QUIT"){
          std::cout << "Cost is " << answer[end].cost <<"; path is " <<recover_path(answer, end) << std::endl;
          end = get_node_in_graph(graph, "Enter stop node",true);
      }


  } catch (ics::IcsError& e) {
    std::cout << e.what() << std::endl;
  }

  return 0;
}
