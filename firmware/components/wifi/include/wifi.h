#ifndef MICROSLAM_WIFI_H
#define MICROSLAM_WIFI_H

#include <stdbool.h>
#include <stdint.h>

#ifndef WIFI_SSID
#define WIFI_SSID ("microslam_AP")
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ("microsl4m")
#endif

void wifi_ap_init(void);
void wifi_ap_deinit(void);

#endif
