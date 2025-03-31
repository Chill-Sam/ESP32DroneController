#include "ControllerLink.h"

ControllerLink::ControllerLink(const char *ssid, const char *password,
                               const char *websocket)
    : ssid(ssid), password(password), websocket(websocket) {}
