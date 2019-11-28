#include "main.h"
#include "tabu.hpp"
#include "entropy.hpp"

//Creates a random alphanumeric string of length len.
std::string makeid(int len) {
  std::string ret;
  static const char alphanum[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
  for (int i = 0; i < len; i++) {
    ret += alphanum[get_random() % (sizeof(alphanum) - 1)];
  }
  return ret;
}

//Makes an empty message object with a new ID.
Message::Message() {
    id = makeid(8);
    content = json::object({});
}

//Parses a message object from a message string.
//The caller is expected to base64-decode the
//incoming message. The inverse of the text() method.
Message::Message(const std::string& text) {
  if(text[0] == '=') {
    addressKind = EVENT;
  } else if(text[0] == '@') {
    addressKind = REPLY;
  } else {
    throw std::runtime_error("No such address kind.");
  }
  auto addressEnd = text.find_first_of('/');
  if(addressEnd == text.size()) {
    throw std::runtime_error("No delimeter.");
  }
  //Really hacky way to parse out any \u0070's that pop up in the address
  address = json::parse("\"" + text.substr(1, addressEnd - 1) + "\"").get_string();
  addressEnd++;
  auto idEnd = text.find_first_of('/', addressEnd);
  auto idLen = idEnd - addressEnd;
  id = text.substr(addressEnd, idLen);
  idEnd++;
  content = json::parse(text.substr(idEnd));
}

//Forms a string representing this message.
//=TOPIC/ID123456/"JSON message data"
std::string Message::text() {
  return (addressKind == EVENT ? "=" : "@") + address + "/" + id + "/" + content.to_string();
}

//Sends this message over USB serial.
void Message::send() {
  tabu_say(text());
}

//Send this message using in small blocks using the
//special "file-transfer" topic. As this waits for
//a response from the receiving device, this will never
//overflow the serial buffer.
//Note: Sending a big message will block the caller.
void Message::bigSend() {
  int bigPos = 0;
  std::string dataStr = content.to_string();
  do {
    auto nextData = dataStr.substr(bigPos, 512);
    bigPos += nextData.size();
    Message segment;
    segment.addressKind = EVENT;
    segment.address = "file-transfer";
    segment.content = json::object({
      {"origId", id},
      {"origAddr", (addressKind == EVENT ? "=" : "@") + address},
      {"nextData", nextData},
      {"done", (bool)(bigPos == dataStr.size())}
    });
    segment.send();
    bool waitingForReply = true;
    tabu_on(segment, [&](Message reply, Message original) {
      waitingForReply = false;
    });
    while(waitingForReply) {
      pros::delay(1);
    }
  } while(bigPos < dataStr.size());
}

//Runs a callable and copyable type, T, as a pros::Task.
//This really should be in its own file.
template<typename T>
void runLambdaAsync(T copyableRunnable) {
  T* copy = new T(copyableRunnable);
  pros::Task([](void* param) {
    T innerCopy = T(*((T*)param));
    delete (T*)param;
    innerCopy();
  }, copy, "runLambdaAsync");
}

class TabuLock {
  bool taken = false;
  public:
  TabuLock() {
    take();
  }
  ~TabuLock() {
    if(taken) give();
  }
  void give() {
    tabu_lock.give();
    taken = false;
  }
  void take() {
    bool wasTaken = tabu_lock.take(500);
    if(!wasTaken) {
      std::cerr << "We've got a problem." << std::endl;
      char *badPtr = nullptr;
      *badPtr = '#';
    }
    taken = true;
  }
};

//Storage for topic listeners.
std::vector<std::pair<std::string, std::function<void(Message)>>> topicListeners;
void push_topic(const std::string& topic, std::function<void(Message)> listener) {
  TabuLock lk;
  topicListeners.push_back({topic, listener});
}
//The provided function will be called when the given topic is received.
//Setting async = true makes the function run in the background.
void tabu_on(const std::string& topic, std::function<void(Message)> listener, bool async) {
  if(async) {
    auto sync = listener;
    listener = [=](Message notif) {
      runLambdaAsync([=]() {
        sync(notif);
      });
    };
  }
  push_topic(topic, std::move(listener));
}

using ReplyListener = std::pair<Message, std::function<void(Message, Message)>>;
//Storage for reply listeners.
std::vector<ReplyListener> replyListeners;
//Orphans occur when a reply is received before a reply listener is registered.
std::vector<Message> orphanReplies;
void push_reply(Message msg, std::function<void(Message, Message)> listener) {
  TabuLock lk;
  for(int i = 0; i < orphanReplies.size(); i++) {
    if(orphanReplies[i].address == msg.id) {
      listener(orphanReplies[i], msg);
      orphanReplies.erase(orphanReplies.begin() + i);
      return;
    }
  }
  replyListeners.push_back({msg, listener});
}
//The provided function will be called when the given message ID is replied to.
//Setting async = true makes the function run in the background.
void tabu_on(Message msg, std::function<void(Message, Message)> listener, bool async) {
  if(async) {
    auto sync = listener;
    listener = [=](Message reply, Message original) {
      runLambdaAsync([=]() {
        sync(reply, original);
      });
    };
  }
  push_reply(msg, std::move(listener));
}

json helpRegistry = json::object({});
void tabu_help(const std::string& topic, const json& help) {
  TabuLock lk;
  helpRegistry[topic] = help;
}

//Constructs and sends an EVENT message object.
Message tabu_send(const std::string& topic, json content) {
  Message msg;
  msg.address = topic;
  msg.content = content;
  msg.addressKind = EVENT;
  msg.send();
  return msg;
}

//Constructs and sends a REPLY message object.
Message tabu_send(Message message, json content) {
  Message msg;
  msg.address = message.id;
  msg.content = content;
  msg.addressKind = REPLY;
  msg.send();
  return msg;
}

//Constructs and bigSend()s an EVENT message object.
Message tabu_send_big(const std::string& topic, json content) {
  Message msg;
  msg.address = topic;
  msg.content = content;
  msg.addressKind = EVENT;
  msg.bigSend();
  return msg;
}

//Constructs and bigSend()s a REPLY message object.
Message tabu_send_big(Message message, json content) {
  Message msg;
  msg.address = message.id;
  msg.content = content;
  msg.addressKind = REPLY;
  msg.bigSend();
  return msg;
}

// ----- Critical Handler Code ----- //
// Code here operates on central tabu
// data and uses TabuLock.

//Accumulating data for "file-transfer" events.
std::unordered_map<std::string, json> ongoingTransfers;
json& updateXfer(Message& msg) {
  TabuLock lk;
  auto origID = msg.content["origID"].get_string();
  auto& xfer = ongoingTransfers[origID];
  auto txt = xfer["text"].is_string() ? xfer["text"].get_string() : "";
  xfer = json::object({
    {"text", txt + msg.content["nextData"].get_string()},
    {"address", msg.content["origAddr"].get_string()},
    {"id", origID}
  });
  return xfer;
}

std::vector<decltype(topicListeners)::value_type> matchingTopicListeners(const Message& msg) {
  TabuLock lk;
  std::vector<decltype(topicListeners)::value_type> matching;
  for(auto& listener: topicListeners) {
    if(listener.first == msg.address) {
      matching.push_back(listener);
    }
  }
  return matching;
}

std::vector<ReplyListener> matchingReplyListeners(const Message& msg) {
  TabuLock lk;
  std::vector<ReplyListener> matching;
  replyListeners.erase(std::remove_if(replyListeners.begin(), replyListeners.end(),
  [&](const ReplyListener& parent) {
      if(parent.first.id == msg.address) {
        matching.push_back(parent);
        return true;
      }
      return false;
  }), replyListeners.end());
  if(matching.empty()) orphanReplies.push_back(msg);
  return matching;
}

// ----- Message/Line Handler -----

void tabu_init();
//Parses and handles a line of serial input, calling appropriate listeners and critical sections.
bool tabu_handler_first_call = true;
void tabu_handler(const std::string& line) {
  try {
    if(tabu_handler_first_call) {
      tabu_handler_first_call = false;
      tabu_init();
    }
    Message msg(line);
    if(msg.addressKind == EVENT) {
      auto matching = matchingTopicListeners(msg);
      for(auto& listener: matching) {
        try {
          listener.second(msg);
        } catch(...) {
          printf(("Caught an exception in listener for " + listener.first + "\n").c_str());
        }
      }
    } else {
      if(msg.addressKind == REPLY) {
        for(auto& parent: matchingReplyListeners(msg)) {
          try {
            parent.second(msg, parent.first);
          } catch(...) {
            printf("Caught exception in reply handler.\n");
          }
        }
      }
    }
  } catch(const std::runtime_error& ex) {
    printf("Caught exception %s\n", ex.what());
  } catch(...) {
    printf("Sorry, I don't know what to do with %s.\n", line.c_str());
  }
}

// ----- Output -----

//Sends a line of text to serial output, after encoding it.
pros::Mutex tabu_lock;
void tabu_say(const std::string& text) {
  TabuLock lk;
  puts(text.c_str());
}

// ----- Initialization -----

void tabu_init() {
  //Retrieves an index of all robot tests.
  tabu_reply_on("help", []() -> json {
    return helpRegistry;
  });
  //Handles large file transfers
  tabu_on("file-transfer", [](Message msg) {
    auto &xfer = updateXfer(msg);
    //Always send a reply
    tabu_send(msg);
    if(msg.content["done"].get_bool()) {
      Message constructed;
      constructed.addressKind = xfer["address"].get_string()[0] == '=' ? EVENT : REPLY;
      constructed.address = xfer["address"].get_string().substr(1);
      constructed.id = xfer["id"].get_string();
      constructed.content = json::parse(xfer["text"].get_string());
      { TabuLock lk; ongoingTransfers.erase(ongoingTransfers.find(constructed.id)); }
      tabu_handler(constructed.text());
    }
  });
}