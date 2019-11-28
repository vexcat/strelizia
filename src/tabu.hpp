#pragma once

#include <string>
#include "main.h"
#include "json.hpp"
#include <functional>

enum AddressKind {
  EVENT,
  REPLY
};

struct Message {
  AddressKind addressKind;
  std::string address;
  std::string id;
  json content;
  Message();
  explicit Message(const std::string& text);
  std::string text();
  void send();
  void bigSend();
  double number(const std::string& key) {
    return content[key].get_number();
  }
  int integer(const std::string& key) {
    return content[key].get_number();
  }
  std::string string(const std::string& key) {
    return content[key].get_string();
  }
  bool boolean(const std::string& key) {
    return content[key].get_bool();
  }
};

Message tabu_send(const std::string& topic, json content = json::object({}));
Message tabu_send(Message toReply, json content = json::object({}));
Message tabu_send_big(const std::string& topic, json content = json::object({}));
Message tabu_send_big(Message toReply, json content = json::object({}));

//Main listener adders, void(inputs)
void tabu_on(const std::string& topic, std::function<void(Message)> listener, bool async = false);
void tabu_on(Message repliedTo, std::function<void(Message, Message)> listener, bool async = false);
//Calls previous listener adders, and replies with a json value.
inline void tabu_reply_on(const std::string& topic, std::function<json(Message)> listener) {
  tabu_on(topic, [=](Message received) {
    tabu_send_big(received, listener(received));
  }, true);
}
inline void tabu_reply_on(Message& repliedTo, std::function<json(Message, Message)> listener) {
  tabu_on(repliedTo, [=](const Message reply, Message original) {
    tabu_send_big(reply, listener(reply, original));
  }, true);
}
//Argumentless wrappers
inline void tabu_on(const std::string& topic, std::function<void()> listener, bool async = false) {
  tabu_on(topic, [=](Message) { listener(); }, async);
}
inline void tabu_on(Message repliedTo, std::function<void()> listener, bool async = false) {
  tabu_on(repliedTo, [=](Message, Message ) { listener(); }, async);
}
inline void tabu_reply_on(const std::string& topic, std::function<json()> listener) {
  tabu_on(topic, [=](Message received) {
    tabu_send_big(received, listener());
  }, true);
}
inline void tabu_reply_on(Message repliedTo, std::function<json()> listener) {
  tabu_on(repliedTo, [=](Message reply, Message original) {
    tabu_send_big(reply, listener());
  }, true);
}

void tabu_handler(const std::string& line);

extern pros::Mutex tabu_lock;
void tabu_say(const std::string& text);

void tabu_help(const std::string& topic, const json& help);

inline json tlabel(const std::string& text) {
  return json::object({{"kind", "label"}, {"text", text}});
}

inline json tnum(const std::string& key, const std::string& label = "") {
  return json::object({{"kind", "number"}, {"key", key}, {"label", (label == "" ? key : label)}});
}

inline json tstr(const std::string& key, const std::string& label = "") {
  return json::object({{"kind", "string"}, {"key", key}, {"label", (label == "" ? key : label)}});
}

inline json tbool(const std::string& key, const std::string& label = "") {
  return json::object({{"kind", "bool"}, {"key", key}, {"label", (label == "" ? key : label)}});
}

inline json treplyaction(const std::string& js) {
  return json::object({{"kind", "reply_action"}, {"do", js}});
}