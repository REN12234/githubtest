#ifndef PARAMETER_HANDLER
#define PARAMETER_HANDLER

#include "yaml-cpp/yaml.h"
#include <string>
#include <vector>

class ParamHandler {
public:
  ParamHandler(const std::string &file_name) {
    try {
      config_ = YAML::LoadFile(file_name);
      fileLoaded = true;
    } catch (std::exception& e) {
      fileLoaded = false;
    }
  }

  virtual ~ParamHandler(){};

  bool getString(const std::string &key, std::string &str_value){
    try {
      str_value = config_[key].as<std::string>();
    } catch (std::exception &e) {
      return false;
    }
    return true;
  }

  bool getString(const std::string &category, const std::string &key, std::string & str_value){
    try {
      str_value = config_[category][key].as<std::string>();
    } catch(std::exception &e) {
      return false;
    }
    return true;
  }

  template<typename T>
  bool getVector(const std::string &key, std::vector<T> &vec_value) {
    try {
      vec_value = config_[key].as<std::vector<T> >();
    } catch (std::exception &e) {
      return false;
    }
    return true;
  }

  template<typename T>
  bool getVector(const std::string &category, const std::string &key, std::vector<T> &vec_value) {
    try {
      vec_value = config_[category][key].as<std::vector<T>>();
    } catch (std::exception &e) {
      return false;
    }
    return true;
  }

  template<typename T>
  bool get2DArray(const std::string &category, const std::string &key, std::vector<std::vector<T> > &vec_value) {
    try {
      vec_value = config_[category][key].as<std::vector<std::vector<T> > >();
    } catch (std::exception &e) {
      return false;
    }
    return true;
  }

  template<typename T>
  bool getValue(const std::string &key, T &T_value) {
    try {
      T_value = config_[key].as<T>();
    } catch (std::exception &e) {
      return false;
    }
    return true;
  }

  template<typename T>
  bool getValue(const std::string &category, const std::string &key, T &T_value) {
    try {
      T_value = config_[category][key].as<T>();
      return true;
    } catch (std::exception &e) {
      return false;
    }
    return true;
  }

  bool getBoolean(const std::string & category, const std::string &key, bool &bool_value){
    try {
      bool_value = config_[category][key].as<bool>();
      return true;
    } catch (std::exception &e) {
      return false;
    }
    return true;
  }


  std::vector<std::string> getKeys() {
    std::vector<std::string> v;
    v.reserve(config_.size());
    for(auto it = config_.begin(); it != config_.end(); it++) {
      v.push_back(it->first.as<std::string>());
    }
    return v;
  }


bool getBoolean(const std::string &key, bool &bool_value) {
  try {
    bool_value = config_[key].as<bool>();
  } catch (std::exception &e) {
    return false;
  }
  return true;
}

bool getInteger(const std::string &key, int &int_value) {
  try {
    int_value = config_[key].as<int>();
  } catch (std::exception &e) {
    return false;
  }
  return true;
}

  bool fileOpenedSuccessfully() {
    return fileLoaded;
  }

protected:
  YAML::Node config_;

private:
  bool fileLoaded = false;
};

#endif
