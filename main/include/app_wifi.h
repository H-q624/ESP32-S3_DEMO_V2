#ifndef _APP_WIFI_H_
#define _APP_WIFI_H_

#include <cstddef>
#include <cstring>

/**
 * @brief Initialize WiFi station mode and connect to AP
 * 
 * @param ssid WiFi SSID
 * @param password WiFi password
 */
void app_wifi_init_sta(const char* ssid, const char* password);

#endif /* _APP_WIFI_H_ */