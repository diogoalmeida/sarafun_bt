#ifndef __YAML_CREATOR__
#define __YAML_CREATOR__

#include <fstream>
#include <yaml-cpp/yaml.h>

namespace tree_generator{
  /**
    The actionlib implementation of the SARAFun actions require
    information on the KF label id in their goal messages.
    This class allows for these parameters to be stored for future
    loading and usage through the parameter server.
  **/
  class YamlCreator {
  public:
    YamlCreator();
    ~YamlCreator();

    /**
      Adds a field with a given value to a given label.
      If the label does not exist in the base node, it will be created.

      @param label The keyframe label
      @param field The parameter field name
      @param value The parameter value.
    **/
    void addField(std::string label, std::string field, int value);
    void writeFile(std::string file_path);
  private:
    YAML::Node base_node_;
  };
}
#endif
