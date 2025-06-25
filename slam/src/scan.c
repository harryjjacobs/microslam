#include <slam/scan.h>

void scan_init(scan_t *scan, float bearing_error, float range_error) {
  scan_reset(scan);
  scan->bearing_error = bearing_error;
  scan->range_error = range_error;
}

void scan_reset(scan_t *scan) {
  scan->hits = 0;
  for (size_t i = 0; i < 360; i++) {
    scan->range[i] = 0;
  }
}

void scan_add(scan_t *scan, unsigned short bearing, float range) {
  scan->range[bearing] = range;
  if (range > 1e-6f) {
    scan->hits++;
  }
}