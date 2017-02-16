#include <tree_generator/YamlCreator.hpp>

namespace tree_generator{

  YamlCreator::YamlCreator(){}
  YamlCreator::~YamlCreator(){}

  void YamlCreator::addField(std::string label, std::string field, int value)
  {
    YAML::Node label_node;

    if (base_node_["sarafun"][label])
    {
      label_node = base_node_["sarafun"][label];
    }

    label_node[field] = value;
    base_node_["sarafun"][label] = label_node;
  }

  void YamlCreator::writeFile(std::string file_path)
  {
    std::ofstream file;
    file.open(file_path);
    file << base_node_;
    file.close();
  }
}
