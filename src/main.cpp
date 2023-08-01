#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <sys/select.h>

struct Message {
  explicit Message(const std::string &str) : msg_(str){};
  ~Message(){};

  uint8_t *encrypt(size_t *size) {
	// Header constant
    static const char *const header = "HELLO";

	// Allocate mem for encoded message and header
    uint8_t *buf = reinterpret_cast<uint8_t *>(std::malloc(std::strlen(header) + msg_.length()));

	// Copy header to mem
    std::memcpy(buf, header, std::strlen(header));

	// Encode message
    for (auto &c : msg_) {
      c++;
    }

	// Copy message to mem
    std::memcpy(buf + std::strlen(header), msg_.c_str(), msg_.length());

	// Check if size pointer null
    if (size) {

	  // OK
      *size = std::strlen(header) + msg_.length();
    } else {

	  // Size pointer is null - error
      return nullptr;
    }

	// OK
	return buf;
  }

private:
  std::string msg_;
};

int32_t main(int32_t argc, char *argv[]) {
}
