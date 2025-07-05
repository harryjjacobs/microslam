#ifndef NET_H
#define NET_H

#pragma once

#include <stdbool.h>

#include "slam/types.h"

bool is_connected();
int connect_to_server(const char *ip, int port);
void disconnect_client(void);
void handle_streams(void);

#endif
