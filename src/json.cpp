#include "json.hpp"
#include <sstream>
#include <iomanip>
#include <iostream>
#include <ios>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <algorithm>

class json::impl {
  friend json;
  //pairs represents the content of a JOBJECT
  //values represents the content of a JARRAY
  //Neither are used when in another mode.
  //Also, it's purely a quirk of g++ that this works.
  //std::vector of an incomplete type isn't supposed to work.
  std::vector<json> values;
  std::vector<std::pair<std::string, json>> pairs;
  public:
  //Initialization Methods
  //You can make JSON values this way and nest them to make an awkward JSON literal.
  impl();
  impl(const impl& orig);
  impl& operator=(const impl& orig);

  json& set_array(std::initializer_list<json> vals);

  json& set_object(std::initializer_list<std::pair<std::string, json>> vals);

  //Parsing
  //This will populate the object with the data contained in the string.
  //Helper functions parse each kind of JSON data.
  private:
  static json parse(const std::string& data, int &idx);
  static json parse_array(const std::string& data, int& idx);
  static json parse_object(const std::string& data, int& idx);
  static void skip_whitespace(const std::string& data, int& idx);
  static json parse_string(const std::string& data, int& idx);
  static std::string utf8_to_16_escaped(const std::string& text);
};

json::impl::impl() = default;
json::impl::impl(const json::impl& orig) = default;
json::impl& json::impl::operator=(const json::impl& orig) = default;

json json::parse(const std::string& data) {
  int idx = 0;
  return json::impl::parse(data, idx);
}

std::vector<json>& json::array_data() { return pImpl->values; }
const std::vector<json>& json::array_data_const() const { return pImpl->values; }
std::vector<std::pair<std::string, json>>& json::object_data() { return pImpl->pairs; }
const std::vector<std::pair<std::string, json>>& json::object_data_const() const { return pImpl->pairs; }

json& json::set_array(std::initializer_list<json> vals) {
  kind = JARRAY;
  pImpl->pairs.clear();
  pImpl->values.clear();
  pImpl->values.reserve(vals.size());
  pImpl->values.insert(pImpl->values.begin(), vals);
  return *this;
}

json& json::set_object(std::initializer_list<std::pair<std::string, json>> vals) {
  kind = JOBJECT;
  pImpl->pairs.clear();
  pImpl->values.clear();
  pImpl->pairs.reserve(vals.size());
  pImpl->pairs.insert(pImpl->pairs.begin(), vals);
  return *this;
}

std::string json::to_string() const {
  if(is_null()) return "null";
  if(is_bool()) return get_bool() ? "true" : "false";
  if(is_string()) return json::impl::utf8_to_16_escaped(get_string());
  if(is_number()) {
    std::ostringstream ret;
    ret << get_number();
    return ret.str();
  }
  if(is_array()) {
    std::string ret = "[";
    bool loopRanOnce = false;
    for(auto& obj: array_data_const()) {
      if(loopRanOnce) ret += ',';
      ret += obj.to_string();
      loopRanOnce = true;
    }
    ret += ']';
    return ret;
  }
  if(is_object()) {
    std::string ret = "{";
    bool loopRanOnce = false;
    for(auto& obj: object_data_const()) {
      if(loopRanOnce) ret += ',';
      auto str = json().set_string(obj.first).to_string();
      ret += str;
      ret += ':';
      ret += obj.second.to_string();
      loopRanOnce = true;
    }
    ret += '}';
    return ret;
  }
  throw std::runtime_error("Unknown object type");
}

json json::impl::parse(const std::string& data, int &idx) {
  //Read until any of [, ", {, n, f, t, # is found.
  char c;
  skip_whitespace(data, idx);
  c = data[idx];
  if(c == '[') return parse_array(data, idx);
  if(c == '"') return parse_string(data, idx);
  if(c == '{') return parse_object(data, idx);
  if(c == 'n') {
    idx += 4;
    return std::move(json().set_null());
  }
  if(c == 't') {
    idx += 4;
    return std::move(json().set_bool(true));
  }
  if(c == 'f') {
    idx += 5;
    return std::move(json().set_bool(false));
  }
  //JSON numbers may not begin with a '.'
  if(c == '-' || ('0' <= c && c <= '9')) {
    const char* dat = data.c_str() + idx;
    char* end;
    double ret = strtod(dat, &end);
    idx += end - dat;
    return std::move(json().set_number(ret));
  }
  throw std::runtime_error("Failed to find JSON object at column " + std::to_string(idx));
}

json json::impl::parse_array(const std::string& data, int& idx) {
  auto ret = std::move(json().set_array({}));
  idx++;
  do {
    if(data[idx] == ']') { idx++; break; }
    ret.array_data().push_back(parse(data, idx));
    skip_whitespace(data, idx);
  } while(data[idx++] == ',');
  //We ended the array, it better have been with a ].
  if(data[idx - 1] != ']') throw std::runtime_error("Expected ] not " + std::to_string(data[idx - 1]));
  return ret;
}

json json::impl::parse_object(const std::string& data, int& idx) {
  auto ret = std::move(json().set_object({}));
  idx++;
  do {
    skip_whitespace(data, idx);
    if(data[idx] == '}') { idx++; break; }
    auto key = parse(data, idx).get_string();
    skip_whitespace(data, idx);
    if(data[idx++] != ':') throw std::runtime_error("No colon after key.");
    auto val = parse(data, idx);
    skip_whitespace(data, idx);
    ret.object_data().push_back({key, std::move(val)});
  } while(data[idx++] == ',');
  //We ended the object, it better have been with a }.
  if(data[idx - 1] != '}') throw std::runtime_error("Expected } not " + std::to_string(data[idx - 1]) + " on original text of " + data);
  return ret;
}

void json::impl::skip_whitespace(const std::string& data, int& idx) {
  char c;
  while((c = data[idx]) == ' ' || c == 0x0D || c == 0x0A || c == 0x09) idx++;
}

static int parse_hex(char c) {
  if('0' <= c && c <= '9') return 0 + (c - '0');
  if('a' <= c && c <= 'z') return 10 + (c - 'a');
  if('A' <= c && c <= 'Z') return 10 + (c - 'A');
  return 0;
}

json json::impl::parse_string(const std::string& data, int& idx) {
  idx++;
  uint32_t lastSurrogatePair = 0;
  std::string ret;
  unsigned char c;
  while((c = data[idx]) != '"') {
    if(c == '\\') {
      c = data[++idx];
      if(c == '\\' || c == '/' || c == '"') {
        ret.push_back(c);
        idx++;
        continue;
      }
      if(c == 'b') { ret.push_back(0x8); continue; }
      if(c == 'f') { ret.push_back(0xc); continue; }
      if(c == 'n') { ret.push_back(0xa); continue; }
      if(c == 'r') { ret.push_back(0xd); continue; }
      if(c == 't') { ret.push_back(0x9); continue; }
      if(c == 'u') {
        uint32_t codepoint = 0;
        for(int i = 0; i < 4; i++) {
          codepoint <<= 4;
          codepoint |= parse_hex(data[++idx]);
        }
        //High Surrogate Pair
        if(0xD800 <= codepoint && codepoint < 0xDC00) {
          lastSurrogatePair = codepoint;
          idx++;
          continue;
        }
        //Low Surrogate Pair
        if(0xDC00 <= codepoint && codepoint < 0xDFFF) {
          codepoint = codepoint & 0x3FF;
          codepoint |= (lastSurrogatePair & 0x3FF) << 10;
          codepoint += 0x10000;
        }
        //Now, to encode this as UTF8!
        if(codepoint < (1 << 7)) {
          ret.push_back(codepoint);
        } else if(codepoint < (1 << 11)) {
          ret.push_back(0b11000000 + (codepoint >> 6));
          ret.push_back(0b10000000 + (codepoint & 0x3F));
        } else if(codepoint < (1 << 16)) {
          ret.push_back(0b11100000 + (codepoint >> 12));
          ret.push_back(0b10000000 + ((codepoint >> 6) & 0x3F));
          ret.push_back(0b10000000 + (codepoint & 0x3F));
        } else if(codepoint < (1 << 21)) {
          ret.push_back(0b11110000 + (codepoint >> 18));
          ret.push_back(0b10000000 + ((codepoint >> 12) & 0x3F));
          ret.push_back(0b10000000 + ((codepoint >> 6) & 0x3F));
          ret.push_back(0b10000000 + (codepoint & 0x3F));
        }
      }
    } else {
      ret.push_back(c);
    }
    idx++;
  }
  idx++;
  return std::move(json().set_string(ret));
}

std::string json::impl::utf8_to_16_escaped(const std::string& text) {
  std::stringstream ret; ret << "\"";
  for(int i = 0; i < text.size(); i++) {
    unsigned int byte = (unsigned char)text[i];
    if(byte < (1 << 7)) {
      if(byte == 0x8) ret << "\\b";
      else if(byte == 0xc) ret << "\\f";
      else if(byte == 0xa) ret << "\\n";
      else if(byte == 0xd) ret << "\\r";
      else if(byte == 0x9) ret << "\\t";
      else if(byte == '"') ret << "\\\"";
      else if(byte == '/') ret << "\\/";
      else if(byte == '\\') ret << "\\\\";
      else if(byte < 32) ret << "\\u00" << std::hex << std::setfill('0') << std::setw(2) << (int)byte;
      else ret << (char)byte;
    } else if(byte < 0xE0) {
      //11 bits, ends at U+07FF
      int32_t codepoint = 0;
      codepoint |= ((unsigned char)text[i++] & 0x1F) << 6;
      codepoint |= ((unsigned char)text[i  ] & 0x3F) << 0;
      ret << "\\u";
      ret << std::hex << std::setfill('0') << std::setw(4) << codepoint;
    } else if(byte < 0xF0) {
      //16 bits, ends at U+FFFF
      int32_t codepoint = 0;
      codepoint |= ((unsigned char)text[i++] & 0xF) << 12;
      codepoint |= ((unsigned char)text[i++] & 0x3F) << 6;
      codepoint |= ((unsigned char)text[i  ] & 0x3F) << 0;
      ret << "\\u";
      ret << std::hex << std::setfill('0') << std::setw(4) << codepoint;
    } else {
      //21 bits, ends at U+10FFFF
      uint32_t codepoint = 0;
      codepoint |= ((unsigned char)text[i++] & 0x7) << 18;
      codepoint |= ((unsigned char)text[i++] & 0x3F) << 12;
      codepoint |= ((unsigned char)text[i++] & 0x3F) << 6;
      codepoint |= ((unsigned char)text[i  ] & 0x3F) << 0;
      if(codepoint < 0x10000) {
        ret << "\\u";
        ret << std::hex << std::setfill('0') << std::setw(4) << codepoint;
      } else {
        codepoint -= 0x10000;
        ret << "\\u";
        ret << std::hex << std::setfill('0') << std::setw(4) << (0xD800 + (codepoint >> 10));
        ret << "\\u";
        ret << std::hex << std::setfill('0') << std::setw(4) << (0xDC00 + (codepoint & 0x3FF));
      }
    }
  }
  ret << '"';
  return ret.str();
}

json::json(const json& orig):
kind(orig.kind),
pImpl(std::make_unique<json::impl>(*orig.pImpl)),
str_value(orig.str_value),
dbl_value(orig.dbl_value),
bool_value(orig.bool_value)
{}

json& json::operator=(const json& orig) {
  kind = orig.kind;
  pImpl = std::make_unique<json::impl>(*orig.pImpl);
  str_value = orig.str_value;
  dbl_value = orig.dbl_value;
  bool_value = orig.bool_value;
  return *this;
}

json::~json() = default;

json::json(json&& bruh) = default;
json& json::operator=(json&& bruh) = default;


json::json(): pImpl(std::make_unique<json::impl>()) {}
json::json(nullptr_t): pImpl(std::make_unique<json::impl>()) { set_null(); }
json::json(double val): pImpl(std::make_unique<json::impl>()) { set_number(val); }
json::json(int val): pImpl(std::make_unique<json::impl>()) { set_number(val); }
json::json(bool val): pImpl(std::make_unique<json::impl>()) { set_bool(val); }
json::json(std::string val): pImpl(std::make_unique<json::impl>()) { set_string(val); }
json::json(const char* val): pImpl(std::make_unique<json::impl>()) { set_string(val); }
json::json(std::initializer_list<json> vals): pImpl(std::make_unique<json::impl>()) { set_array(std::move(vals)); }
json::json(std::initializer_list<std::pair<std::string, json>> vals): pImpl(std::make_unique<json::impl>()) { set_object(std::move(vals)); }

json& json::operator[](const std::string& key) {
  for(auto& pair: object_data()) {
    if(pair.first == key) return pair.second;
  }
  object_data().insert(object_data().end(), {
    {key, json()}
  });
  return (object_data().end() - 1)->second;
}

json& json::operator[](int key) {
  return array_data()[key];
}
std::vector<std::pair<std::string, json>>::iterator json::find(const std::string& key) {
  return std::find_if(object_data().begin(), object_data().end(), [&](const std::pair<std::string, json>& kv) {
    return key == kv.first;
  });
}