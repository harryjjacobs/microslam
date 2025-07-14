#include <slam/scan.h>

void scan_reset(scan_t *scan) {
  scan->hits = 0;
  for (size_t i = 0; i < 360; i++) {
    scan->range[i] = 0;
  }
}

void scan_add(scan_t *scan, uint16_t bearing, uint16_t range) {
  scan->range[bearing] = range;
  if (range > 1e-6f) {
    scan->hits++;
  }
}

bool scan_valid(const scan_t *scan, int index) {
  return scan->range[index] > 1e-6f;
}
