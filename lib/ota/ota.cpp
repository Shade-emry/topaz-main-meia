/*
	SlimeVR Code is placed under the MIT license.

	This build deliberately has no OTA application partition. Keep the public
	interface as no-op stubs so shared startup code remains simple while the OTA
	library and updater implementation are not linked into the firmware.
*/

#include "ota.h"

void OTA::otaSetup(const char* const) {}
void OTA::otaUpdate() {}
