#include "main.h"
#include "tabu.hpp"
#include "entropy.hpp"

using json = nlohmann::json;

//Functions for encoding and decoding tabu's
//variant of base64. It is almost the same as
//normal URL-safe base64, but replacing 'p'
//with '@' as it is the PROS command character.
namespace encoding {

  std::string encode(const std::string& data) {
    static constexpr char sEncodingTable[] = {
      'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
      'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P',
      'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
      'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
      'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
      'o', '@', 'q', 'r', 's', 't', 'u', 'v',
      'w', 'x', 'y', 'z', '0', '1', '2', '3',
      '4', '5', '6', '7', '8', '9', '-', '_'
    };

    size_t in_len = data.size();
    size_t out_len = 4 * ((in_len + 2) / 3);
    std::string ret(out_len, '\0');
    size_t i;
    char *p = const_cast<char*>(ret.c_str());

    for (i = 0; i < in_len - 2; i += 3) {
      *p++ = sEncodingTable[(data[i] >> 2) & 0x3F];
      *p++ = sEncodingTable[((data[i] & 0x3) << 4) | ((int) (data[i + 1] & 0xF0) >> 4)];
      *p++ = sEncodingTable[((data[i + 1] & 0xF) << 2) | ((int) (data[i + 2] & 0xC0) >> 6)];
      *p++ = sEncodingTable[data[i + 2] & 0x3F];
    }
    if (i < in_len) {
      *p++ = sEncodingTable[(data[i] >> 2) & 0x3F];
      if (i == (in_len - 1)) {
        *p++ = sEncodingTable[((data[i] & 0x3) << 4)];
        *p++ = '=';
      }
      else {
        *p++ = sEncodingTable[((data[i] & 0x3) << 4) | ((int) (data[i + 1] & 0xF0) >> 4)];
        *p++ = sEncodingTable[((data[i + 1] & 0xF) << 2)];
      }
      *p++ = '=';
    }

    return ret;
  }

  std::string decode(const std::string& input) {
    std::string out;
    static constexpr unsigned char kDecodingTable[] = {
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 62, 64, 64,
      52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 64, 64, 64, 64, 64, 64,
      41,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14,
      15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 64, 64, 64, 64, 63,
      64, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
      64, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
      64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64
    };

    size_t in_len = input.size();
    if (in_len % 4 != 0) throw std::runtime_error("Input data size is not a multiple of 4");

    size_t out_len = in_len / 4 * 3;
    if (input[in_len - 1] == '=') out_len--;
    if (input[in_len - 2] == '=') out_len--;

    out.resize(out_len);

    for (size_t i = 0, j = 0; i < in_len;) {
      uint32_t a = input[i] == '=' ? 0 & i++ : kDecodingTable[static_cast<int>(input[i++])];
      uint32_t b = input[i] == '=' ? 0 & i++ : kDecodingTable[static_cast<int>(input[i++])];
      uint32_t c = input[i] == '=' ? 0 & i++ : kDecodingTable[static_cast<int>(input[i++])];
      uint32_t d = input[i] == '=' ? 0 & i++ : kDecodingTable[static_cast<int>(input[i++])];

      uint32_t triple = (a << 3 * 6) + (b << 2 * 6) + (c << 1 * 6) + (d << 0 * 6);

      if (j < out_len) out[j++] = (triple >> 2 * 8) & 0xFF;
      if (j < out_len) out[j++] = (triple >> 1 * 8) & 0xFF;
      if (j < out_len) out[j++] = (triple >> 0 * 8) & 0xFF;
    }

    return out;
  }

}

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
    content = json::object();
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
  address = text.substr(1, addressEnd - 1);
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
  return (addressKind == EVENT ? "=" : "@") + address + "/" + id + "/" + content.dump();
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
  std::string dataStr = content.dump();
  do {
    auto nextData = dataStr.substr(bigPos, 256);
    bigPos += nextData.size();
    Message segment;
    segment.addressKind = EVENT;
    segment.address = "file-transfer";
    segment.content = {
      {"origId", id},
      {"origAddr", (addressKind == EVENT ? "=" : "@") + address},
      {"nextData", nextData},
      {"done", (bool)(bigPos == dataStr.size())}
    };
    segment.send();
    bool waitingForReply = true;
    tabu_on(segment, [&](const Message& reply, const Message& original) {
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
  }, copy);
}

//Storage for topic listeners.
std::vector<std::pair<std::string, std::function<void(const Message&)>>> topicListeners;
//The provided function will be called when the given topic is received.
//Setting async = true makes the function run in the background.
void tabu_on(const std::string& topic, std::function<void(const Message&)> listener, bool async) {
  if(async) {
    auto sync = listener;
    listener = [=](const Message& notif) {
      runLambdaAsync([=]() {
        sync(notif);
      });
    };
  }
  tabu_lock.take(TIMEOUT_MAX);
  topicListeners.push_back({topic, listener});
  tabu_lock.give();
}

using ReplyListener = std::pair<Message, std::function<void(const Message&, const Message&)>>;
//Storage for reply listeners.
std::vector<ReplyListener> replyListeners;
//Orphans occur when a reply is received before a reply listener is registered.
std::vector<Message> orphanReplies;
//The provided function will be called when the given message ID is replied to.
//Setting async = true makes the function run in the background.
void tabu_on(const Message& msg, std::function<void(const Message&, const Message&)> listener, bool async) {
  if(async) {
    auto sync = listener;
    listener = [=](const Message& reply, const Message& original) {
      runLambdaAsync([=]() {
        sync(reply, original);
      });
    };
  }
  tabu_lock.take(TIMEOUT_MAX);
  for(int i = 0; i < orphanReplies.size(); i++) {
    if(orphanReplies[i].address == msg.id) {
      listener(orphanReplies[i], msg);
      orphanReplies.erase(orphanReplies.begin() + i);
      return;
    }
  }
  replyListeners.push_back({msg, listener});
  tabu_lock.give();
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
Message tabu_send(const Message& message, json content) {
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
Message tabu_send_big(const Message& message, json content) {
  Message msg;
  msg.address = message.id;
  msg.content = content;
  msg.addressKind = REPLY;
  msg.bigSend();
  return msg;
}

//Accumulating data for "file-transfer" events.
std::unordered_map<std::string, json> ongoingTransfers;
//Parses and handles a line of serial input, calling appropriate listeners.
void tabu_handler(const std::string& line) {
  try {
    Message msg(encoding::decode(line));
    if(msg.addressKind == EVENT) {
      if(msg.address == "file-transfer") {
        tabu_lock.take(TIMEOUT_MAX);
        auto origID = msg.content["origID"].get<std::string>();
        if(ongoingTransfers.find(origID) == ongoingTransfers.end()) {
          ongoingTransfers[origID] = {
            {"text", ""},
            {"address", msg.content["origAddr"].get<std::string>()},
            {"id", origID}
          };
        }
        auto& xfer = ongoingTransfers[origID];
        xfer["text"] = xfer["text"].get<std::string>() + msg.content["nextData"].get<std::string>();
        tabu_lock.give();
        //Always send a reply
        tabu_send(msg);
        if(msg.content["done"].get<bool>()) {
          Message constructed;
          constructed.addressKind = xfer["address"].get<std::string>()[0] == '=' ? EVENT : REPLY;
          constructed.address = xfer["address"].get<std::string>().substr(1);
          constructed.id = xfer["id"].get<std::string>();
          constructed.content = json::parse(xfer["text"].get<std::string>());
          tabu_lock.take(TIMEOUT_MAX);
          ongoingTransfers.erase(ongoingTransfers.find(origID));
          tabu_lock.give();
          tabu_handler(encoding::encode(constructed.text()));
        }
      } else {
        tabu_lock.take(TIMEOUT_MAX);
        std::vector<decltype(topicListeners)::value_type> matching;
        for(auto& listener: topicListeners) {
          if(listener.first == msg.address) {
            matching.push_back(listener);
          }
        }
        tabu_lock.give();
        for(auto& listener: matching) {
          try {
            listener.second(msg);
          } catch(...) {
            printf(("Caught an exception in listener for " + listener.first + "\n").c_str());
          }
        }
      }
    } else {
      if(msg.addressKind == REPLY) {
        auto found = false;
        std::vector<ReplyListener> matching;
        tabu_lock.take(TIMEOUT_MAX);
        replyListeners.erase(std::remove_if(replyListeners.begin(), replyListeners.end(),
        [&](const ReplyListener& parent) {
            if(parent.first.id == msg.address) {
              matching.push_back(parent);
              found = true;
              return true;
            }
            return false;
        }), replyListeners.end());
        if(matching.empty()) {
          orphanReplies.push_back(msg);
        }
        tabu_lock.give();
        for(auto& parent: matching) {
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

//Sends a line of text to serial output, after encoding it.
pros::Mutex tabu_lock;
void tabu_say(const std::string& text) {
  tabu_lock.take(TIMEOUT_MAX);
  puts(encoding::encode(text).c_str());
  tabu_lock.give();
}