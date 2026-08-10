#ifndef ESP_DRONE_TRANSPORT_H
#define ESP_DRONE_TRANSPORT_H

#include <stdbool.h>

#include "crtp.h"

/** Initialize the transport selected by Kconfig. */
void transportInit(void);

/** Return true when the selected transport initialized as designed. */
bool transportTest(void);

/** Return the CRTP link implementation for the selected transport. */
struct crtpLinkOperations *transportGetLink(void);

/** Human-readable transport name for startup diagnostics. */
const char *transportGetName(void);

#endif
