#include "net.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "slam/logging.h"
#include "slam/serialisation.h"

#define BUFFER_SIZE 4096
static char _buf[BUFFER_SIZE];

static serialisation_buffer_t buffer = {
    .data = &_buf[0],
    .length = 0,
    .capacity = 0,
};

int client_fd = -1;

extern bool can_process_message(const char id[4]);
extern void process_message(const char id[4],
                            const serialisation_buffer_t *msg);

static int set_nonblocking(int fd) {
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) return -1;
  return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

bool is_connected() { return client_fd >= 0; }

int connect_to_server(const char *ip, int port) {
  int sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    perror("socket");
    return -1;
  }

  struct sockaddr_in server_addr = {
      .sin_family = AF_INET,
      .sin_port = htons(port),
  };

  if (inet_pton(AF_INET, ip, &server_addr.sin_addr) <= 0) {
    ERROR("Invalid server address: %s", ip);
    close(sockfd);
    return -1;
  }

  if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) <
      0) {
    close(sockfd);
    return -1;  // silently fail, caller will retry
  }

  set_nonblocking(sockfd);
  client_fd = sockfd;
}

void disconnect_client(void) {
  if (client_fd >= 0) {
    close(client_fd);
    client_fd = -1;
    buffer.length = 0;
    INFO("Disconnected from server");
  }
}

void handle_streams(void) {
  if (client_fd < 0) return;

  ssize_t n = recv(client_fd, buffer.data + buffer.length,
                   BUFFER_SIZE - buffer.length, 0);
  if (n < 0) {
    if (errno != EWOULDBLOCK && errno != EAGAIN) {
      perror("recv");
      disconnect_client();
    }
    return;
  } else if (n == 0) {
    disconnect_client();
    return;
  }

  buffer.length += (size_t)n;

  size_t offset = 0;
  while (buffer.length - offset >= SLAM_SERIALISATION_HEADER_LEN) {
    char id[SLAM_SERIALISATION_ID_LEN];
    uint16_t msg_len = 0;

    if (read_header(&buffer, offset, id, &msg_len) < 0 ||
        !can_process_message(id)) {
      continue;
    }

    if (buffer.length - offset < msg_len) break;

    serialisation_buffer_t view = {
        .data = buffer.data + offset,
        .length = msg_len,
        .capacity = msg_len,
    };
    process_message(id, &view);

    offset += msg_len;
  }

  if (offset > 0) {
    memmove(buffer.data, buffer.data + offset, buffer.length - offset);
    buffer.length -= offset;
  }
}
