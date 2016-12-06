#ifndef __YAML_CREATOR__
#define __YAML_CREATOR__

#include <fstream>
#include <yaml-cpp/yaml.h>

namespace tree_generator{
  /*
    Will maintain all the required parameters for a specific
    sarafun keyframe label, and maintain a YAML node with the
    information.
  */
  class YamlCreator {
  public:
    YamlCreator();
    ~YamlCreator();

    void addField(std::string label, std::string field, int value);
    void writeFile(std::string file_path);
  private:
    YAML::Node base_node_;
  };
}
#endif
