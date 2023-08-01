#include <arpa/inet.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <linux/in.h>
#include <string>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

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

void usage() { std::fprintf(stderr, "Missing parameter(s) -d <serial device file> -b <baudrate> -p <udp_port>\r\n"); }

int32_t main(int32_t argc, char *argv[]) {
  static constexpr auto nstrings = 10u;
  static constexpr auto timeout_val_s = 5u;
  int32_t rc, sp_fd, baudrate = 0, sock_fd, param = 1;
  uint16_t udp_port = 0;
  char *serial_dev_file{nullptr};
  struct termios tty;
  char read_buf[256u]{0x00};
  struct sockaddr_in broadcast_addr;

  while ((rc = ::getopt(argc, argv, "d:b:p:")) != -1) {
    switch (rc) {
    case 'd': {
      serial_dev_file = optarg;
    } break;

    case 'b': {
      baudrate = std::atoi(optarg);
    } break;

    case 'p': {
      udp_port = std::atoi(optarg);
    } break;

    default:
      break;
    }
  }

  if (!serial_dev_file || !baudrate || !udp_port) {
    usage();
    goto error;
  }

  if ((sp_fd = ::open(serial_dev_file, O_RDWR)) < 0) {
    std::fprintf(stderr, "Error open(): %s, %s (%s:%i)\r\n", serial_dev_file, ::strerror(errno), __FILE__, __LINE__);
    goto error;
  }

  if (::tcgetattr(sp_fd, &tty) != 0) {
    std::fprintf(stderr, "Error tcgetattr(): %s (%s:%i)\r\n", strerror(errno), __FILE__, __LINE__);
    goto error;
  }

  // Setup serial port
  tty.c_cflag &= ~PARENB; // No parity
  tty.c_cflag &= ~CSTOPB; // One stop-bit
  tty.c_cflag &= ~CSIZE;  // Set data size 8 bit
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
  tty.c_cflag |= CREAD | CLOCAL; // Eanble reading and ingore modem specific signals
  tty.c_lflag &= ~ICANON;        // Disable canonical mode to read lines from serial dev
  tty.c_lflag &= ~ECHO;          // Disable echo
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST;                // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR;                // Prevent conversion of newline to carriage return/line feed
  tty.c_cc[VTIME] = timeout_val_s * 10; // Timeout in deciseconds
  tty.c_cc[VMIN] = 0;

  // Only 2 baudrates supported
  switch (baudrate) {
  case 9600: {
    baudrate = B9600;
  } break;

  case 115200: {
    baudrate = B115200;
  } break;

  default: {
    std::fprintf(stderr, "Wrong baudrate, 9600 and 115200 only supported (%s:%i)\r\n", __FILE__, __LINE__);
    goto error;
  } break;
  }

  // Set in/out baudrate
  if ((rc = ::cfsetispeed(&tty, baudrate)) < 0) {

    std::fprintf(stderr, "Error cfsetispeed(): %s (%s:%i)\r\n", ::strerror(errno), __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::cfsetospeed(&tty, baudrate)) < 0) {

    std::fprintf(stderr, "Error cfsetispeed(): %s (%s:%i)\r\n", ::strerror(errno), __FILE__, __LINE__);
    goto error;
  }

  // Save settings
  if (::tcsetattr(sp_fd, TCSANOW, &tty) != 0) {
    std::fprintf(stderr, "Error tcsetattr(): %s (%s:%i)\r\n", ::strerror(errno), __FILE__, __LINE__);
    return 1;
  }

  // Create udp broadcast socket
  if ((sock_fd = ::socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    std::fprintf(stderr, "Error socket(): %s (%s:%i)\r\n", ::strerror(errno), __FILE__, __LINE__);
    goto error;
  }

  // Create udp broadcast socket
  if ((rc = ::setsockopt(sock_fd, SOL_SOCKET, SO_BROADCAST, (char *)&param, sizeof(param))) < 0) {
    std::fprintf(stderr, "Error setsockopt(): %s (%s:%i)\r\n", ::strerror(errno), __FILE__, __LINE__);
    goto error;
  }

  std::memset((void *)&broadcast_addr, 0, sizeof(struct sockaddr_in));
  broadcast_addr.sin_family = AF_INET;
  broadcast_addr.sin_addr.s_addr = ::htonl(INADDR_BROADCAST);
  broadcast_addr.sin_port = ::htons(udp_port);

  // Try to read
  for (uint32_t i = 0; i < nstrings; i++) {
    if ((rc = ::read(sp_fd, &read_buf, sizeof(read_buf))) < 0) {
      ::close(sp_fd);
      std::fprintf(stderr, "Error read(): %s (%s:%i)\r\n", ::strerror(errno), __FILE__, __LINE__);
      goto error;
    }

    std::string read_str = read_buf;
    size_t encrypted_size;

    if (read_str.length()) {
      // Handle string
      uint8_t *enc_buf = Message(read_str).encrypt(&encrypted_size);
      std::printf("Read data encoded: %s, sending to UDP broadcast\r\n", std::string(reinterpret_cast<char *>(enc_buf), encrypted_size).c_str());

      // Send to UDP broadcast
      if ((rc = ::sendto(sock_fd, enc_buf, encrypted_size, 0, reinterpret_cast<struct sockaddr *>(&broadcast_addr), sizeof(struct sockaddr_in))) < 0) {
        std::fprintf(stderr, "Error sendto(): %s (%s:%i)\r\n", ::strerror(errno), __FILE__, __LINE__);
        ::close(sock_fd);
        goto error;
      }

      std::memset(read_buf, '\0', sizeof(read_buf));
      std::free(enc_buf);
    } else {

      // Empty string
      std::printf("Empty string or timeout recvd\r\n");
    }
  }

exit:
  ::close(sp_fd);
  ::close(sock_fd);
  return 0;

error:
  return -1;
}
