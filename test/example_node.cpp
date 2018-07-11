#include "graph_utils.h"

#include <string>
#include <iostream>

int main(int argc, char* argv[])
{
  // Preallocate output variables
  size_t num_poses;
  std::map<std::pair<size_t,size_t>, graph_utils::Transform> transforms;
  std::list<std::pair<size_t,size_t>> loop_closure_list;

  graph_utils::parse_g2o_file("/home/lajoiepy/Documents/master/mit/graph_data/CSAIL.g2o", num_poses, transforms, loop_closure_list);
  std::cout << num_poses << std::endl;

  return 0;
}