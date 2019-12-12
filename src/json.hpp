//Simple JSON implementation, not making use of templates or really anything special.
//Unicode \u escapes are stored as UTF-8 bytes, no support for other encodings because
//other encodings are wack.
//It prioritizes a small header size for fast compile times, and refuses to use templates
//for faster link time.

#include <vector>
#include <string>
#include <stdexcept>
#include <memory>

enum json_type {
  JNULL, JNUM, JBOOL, JSTRING,
  JARRAY, JOBJECT
};

class json {
  struct impl;
  json_type kind;
  std::unique_ptr<impl> pImpl;
  std::string str_value;
  double dbl_value;
  bool bool_value;
  public:
  //These need to be defined, not declared, as default to make pImpl+unique_ptr work.
  json(json&& toBeMoved);
  json& operator=(json&& toBeMoved);
  ~json();
  //Initialization Methods
  //You can make JSON values this way and nest them to make an awkward JSON literal.
  json();
  json(const json& orig);
  json& operator=(const json& orig);

  inline json& set_null() { kind = JNULL; return *this; }
  json(std::nullptr_t);
  inline bool is_null() const { return kind == JNULL; }

  inline json& set_number(double val) { kind = JNUM; dbl_value = val; return *this; }
  json(double val);
  json(int val);
  inline bool is_number() const { return kind == JNUM; }
  inline double get_number() const {
    if(!is_number()) throw std::runtime_error("I am not a number");
    return dbl_value;
  }

  inline json& set_bool(bool val) { kind = JBOOL; bool_value = val; return *this; }
  json(bool val);
  inline bool is_bool() const { return kind == JBOOL; }
  inline bool get_bool() const {
    if(!is_bool()) throw std::runtime_error("I am not a boolean");
    return bool_value;
  }

  inline json& set_string(std::string val) { kind = JSTRING; str_value = val; return *this; }
  json(std::string val);
  json(const char* val);
  inline bool is_string() const { return kind == JSTRING; }
  inline std::string get_string() const {
    if(!is_string()) throw std::runtime_error("I am not a string");
    return str_value;
  }

  json& set_array(std::initializer_list<json> vals);
  inline static json array(std::initializer_list<json> init) {
    return json(std::move(init));
  }
  json(std::initializer_list<json> vals);
  inline bool is_array() const { return kind == JARRAY; }

  json& set_object(std::initializer_list<std::pair<std::string, json>> vals);
  json(std::initializer_list<std::pair<std::string, json>> vals);
  inline static json object(std::initializer_list<std::pair<std::string, json>> init) {
    return json(std::move(init));
  }
  inline bool is_object() const { return kind == JOBJECT; }
  //Modification Methods
  //These will let you modify the JSON object.
  //Because I'm lazy, it just returns the
  //underlying vector/unordered_map to work on directly.
  std::vector<json>& array_data();
  const std::vector<json>& array_data_const() const;
  std::vector<std::pair<std::string, json>>& object_data();
  const std::vector<std::pair<std::string, json>>& object_data_const() const;
  json& operator[](const std::string& key);
  json& operator[](int key);
  std::vector<std::pair<std::string, json>>::iterator find(const std::string& key);
  //Parsing
  //This will populate the object with the data contained in the string.
  //Helper functions parse each kind of JSON data.
  static json parse(const std::string& data);
  std::string to_string() const;
};
