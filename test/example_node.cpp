#include "graph_utils.h"

#include <string>
#include <iostream>

int main(int argc, char* argv[])
{
  size_t num_poses;
  std::vector<graph_utils::Transform> transforms = graph_utils::parse_g2o_file("/home/lajoiepy/Documents/master/mit/graph_data/CSAIL.g2o", num_poses);
  std::cout << num_poses << std::endl;

  return 0;
}