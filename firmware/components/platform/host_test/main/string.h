#ifndef MICROSLAM_STRING_H
#define MICROSLAM_STRING_H

#include <stdlib.h>
#include <string.h>

typedef struct {
  char *data;       // pointer to the data buffer
  size_t capacity;  // size of the data buffer
  size_t length;    // current position in the buffer
} string_t;

void string_init(string_t *str) {
  str->capacity = 16;
  str->length = 0;
  str->data = (char *)malloc(str->capacity);
  if (str->data) str->data[0] = '\0';
}

void string_append(string_t *str, const char *suffix) {
  size_t suffix_len = strlen(suffix);
  size_t required = str->length + suffix_len + 1;

  if (required > str->capacity) {
    while (str->capacity < required) {
      str->capacity *= 2;
    }
    str->data = (char *)realloc(str->data, str->capacity);
    if (!str->data) {
      str->capacity = 0;
      str->length = 0;
      return;  // allocation failed
    }
  }

  if (str->data) {
    memcpy(str->data + str->length, suffix, suffix_len + 1);
    str->length += suffix_len;
  }
}

void string_free(string_t *str) {
  if (!str->data) return;
  free(str->data);
  str->data = NULL;
  str->length = 0;
  str->capacity = 0;
}

void string_clear(string_t *str) {
  if (!str || !str->data) return;
  str->length = 0;
  str->data[0] = '\0';
}

size_t string_length(const string_t *str) { return str ? str->length : 0; }

#endif
