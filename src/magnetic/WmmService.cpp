#include "WmmService.h"

#if ENABLE_WMM && defined(ESP32)

#include <HTTPClient.h>
#include <LittleFS.h>
#include <NetworkClientSecure.h>
#include <WiFi.h>
#include <time.h>

#include <cmath>
#include <cstring>

#include "GlobalVars.h"
#include "WmmCertificates.h"
#include "serial/serialcommands.h"

#if __has_include("../secrets.local.h")
#include "../secrets.local.h"
#endif

#ifndef NOAA_GEOMAG_API_KEY
#define NOAA_GEOMAG_API_KEY ""
#endif

namespace SlimeVR::Magnetic {
namespace {
constexpr const char* SettingsPath = "/wmm_settings.bin";
constexpr const char* WmmHrCachePath = "/wmmhr_cache.bin";
constexpr const char* WmmCachePath = "/wmm_cache.bin";
constexpr const char* IpLocationUrl = "https://ipwho.is/";
constexpr const char* NoaaEndpoint
	= "https://www.ngdc.noaa.gov/geomag-web/calculators/calculateIgrfwmm";
}  // namespace

void WmmService::setup() {
	loadSettings();
	logger.info(
		"Mode=%s, model=%s, auto-location=%s",
		modeName(mode),
		modelName(model),
		autoLocation ? "on" : "off"
	);
}

void WmmService::resolveStartup() {
	if (mode == WmmMode::Off || !sensorManager.hasEnabledMagnetometer()) {
		logger.info("Startup lookup skipped (disabled or no enabled magnetometer)");
		sensorManager.clearMagneticReference();
		return;
	}

	logger.info("Waiting for WiFi before magnetic-reference initialization");
	waitForNetwork();
	synchronizeClock();

	if (autoLocation) {
		WmmLocation detected{};
		if (lookupIpLocation(detected)) {
			location = detected;
			saveSettings();
		} else if (location.valid) {
			logger.warn("IP lookup failed; using the last saved location");
		}
	}

	if (!location.valid) {
		logger.warn(
			"No location available; use WMM SET-LOCATION <lat> <lon> [alt-km]"
		);
		sensorManager.clearMagneticReference();
		return;
	}

	resolveReference();
	applyReference();
}

void WmmService::refresh() { resolveStartup(); }

void WmmService::setMode(WmmMode value) {
	mode = value;
	saveSettings();
	if (mode == WmmMode::Off) {
		sensorManager.clearMagneticReference();
	} else {
		applyReference();
	}
	logger.info("Mode set to %s", modeName(mode));
}

void WmmService::setModel(WmmModel value) {
	model = value;
	saveSettings();
	logger.info("Model selection set to %s", modelName(model));
}

void WmmService::setAutoLocation(bool enabled) {
	autoLocation = enabled;
	if (!enabled && location.valid) {
		location.manual = true;
	}
	saveSettings();
	logger.info("Automatic IP location %s", enabled ? "enabled" : "disabled");
}

bool WmmService::setManualLocation(
	double latitude,
	double longitude,
	float altitudeKm
) {
	if (!std::isfinite(latitude) || !std::isfinite(longitude)
		|| !std::isfinite(altitudeKm) || latitude < -90 || latitude > 90
		|| longitude < -180 || longitude > 180 || altitudeKm < -10
		|| altitudeKm > 10000) {
		logger.error("Invalid location");
		return false;
	}
	location = {
		.latitude = latitude,
		.longitude = longitude,
		.altitudeKm = altitudeKm,
		.valid = true,
		.manual = true,
	};
	autoLocation = false;
	saveSettings();
	printLocation();
	return true;
}

void WmmService::clearLocation() {
	location = {};
	saveSettings();
	logger.info("Saved location cleared");
}

void WmmService::clearCache() {
	LittleFS.remove(WmmHrCachePath);
	LittleFS.remove(WmmCachePath);
	reference = {};
	sensorManager.clearMagneticReference();
	logger.info("WMMHR and WMM caches cleared");
}

bool WmmService::waitForNetwork() {
	const uint32_t started = millis();
	while (!wifiNetwork.isConnected()
		   && millis() - started < NetworkTimeoutMillis) {
		networkManager.update();
		SerialCommands::update();
		delay(10);
	}
	if (!wifiNetwork.isConnected()) {
		logger.warn("WiFi wait timed out; trying saved magnetic references");
		return false;
	}
	logger.info("WiFi connected; resolving magnetic reference");
	return true;
}

bool WmmService::synchronizeClock() {
	if (!wifiNetwork.isConnected()) {
		return false;
	}
	configTime(0, 0, "pool.ntp.org", "time.nist.gov");
	const uint32_t started = millis();
	time_t now = time(nullptr);
	while (now < 1735689600 && millis() - started < TimeSyncTimeoutMillis) {
		delay(50);
		now = time(nullptr);
	}
	if (now < 1735689600) {
		logger.warn("NTP time synchronization failed");
		return false;
	}
	return true;
}

bool WmmService::lookupIpLocation(WmmLocation& result) {
	if (!wifiNetwork.isConnected()) {
		return false;
	}
	String body;
	if (!httpGet(IpLocationUrl, body)) {
		return false;
	}
	double latitude = 0;
	double longitude = 0;
	const char* latNames[]{"latitude", "lat"};
	const char* lonNames[]{"longitude", "lon"};
	if (!extractJsonNumber(body, latNames, 2, latitude)
		|| !extractJsonNumber(body, lonNames, 2, longitude)) {
		logger.warn("IP location response did not contain coordinates");
		return false;
	}
	result = {
		.latitude = latitude,
		.longitude = longitude,
		.altitudeKm = 0,
		.valid = true,
		.manual = false,
	};
	logger.info("IP location acquired: %.4f, %.4f", latitude, longitude);
	return true;
}

bool WmmService::requestReference(WmmModel requestedModel, WmmReference& result) {
	if (!wifiNetwork.isConnected() || strlen(NOAA_GEOMAG_API_KEY) == 0) {
		if (strlen(NOAA_GEOMAG_API_KEY) == 0) {
			logger.warn("NOAA API key is not compiled into this build");
		}
		return false;
	}

	time_t now = time(nullptr);
	struct tm utc {};
	if (now < 1735689600 || gmtime_r(&now, &utc) == nullptr) {
		logger.warn("Cannot request NOAA data without a valid UTC date");
		return false;
	}

	const char* requestedName
		= requestedModel == WmmModel::WmmHr ? "WMMHR" : "WMM";
	String url = String(NoaaEndpoint) + "?lat1=" + String(location.latitude, 6)
			   + "&lon1=" + String(location.longitude, 6) + "&elevation="
			   + String(location.altitudeKm, 3)
			   + "&elevationUnits=K&coordinateSystem=M&model=" + requestedName
			   + "&startYear=" + String(utc.tm_year + 1900)
			   + "&startMonth=" + String(utc.tm_mon + 1)
			   + "&startDay=" + String(utc.tm_mday)
			   + "&resultFormat=json&key=" + NOAA_GEOMAG_API_KEY;

	String body;
	if (!httpGet(url, body)) {
		logger.warn("NOAA %s request failed", requestedName);
		return false;
	}

	double total = 0;
	double horizontal = 0;
	double north = 0;
	double east = 0;
	double vertical = 0;
	double inclination = 0;
	double declination = 0;
	const char* totalNames[]{
		"totalintensity",
		"totalIntensity",
		"total_intensity",
	};
	const char* horizontalNames[]{
		"horizontalintensity",
		"horizontalIntensity",
		"horizontal_intensity",
	};
	const char* northNames[]{"northcomponent", "northComponent", "x"};
	const char* eastNames[]{"eastcomponent", "eastComponent", "y"};
	const char* verticalNames[]{"verticalcomponent", "verticalComponent", "z"};
	const char* inclinationNames[]{"inclination", "dip"};
	const char* declinationNames[]{"declination"};
	if (!extractJsonNumber(body, totalNames, 3, total)
		|| !extractJsonNumber(body, inclinationNames, 2, inclination)
		|| !extractJsonNumber(body, declinationNames, 1, declination)) {
		logger.warn("NOAA %s response could not be parsed", requestedName);
		return false;
	}
	extractJsonNumber(body, horizontalNames, 3, horizontal);
	extractJsonNumber(body, northNames, 3, north);
	extractJsonNumber(body, eastNames, 3, east);
	extractJsonNumber(body, verticalNames, 3, vertical);

	// NOAA returns magnetic intensity in nT.
	result = {
		.totalIntensityMicroTesla = static_cast<float>(total / 1000.0),
		.horizontalIntensityMicroTesla = static_cast<float>(horizontal / 1000.0),
		.northMicroTesla = static_cast<float>(north / 1000.0),
		.eastMicroTesla = static_cast<float>(east / 1000.0),
		.verticalMicroTesla = static_cast<float>(vertical / 1000.0),
		.inclinationDegrees = static_cast<float>(inclination),
		.declinationDegrees = static_cast<float>(declination),
		.timestamp = static_cast<uint32_t>(now),
		.location = location,
		.source = requestedModel == WmmModel::WmmHr ? WmmSource::NoaaWmmHr
												  : WmmSource::NoaaWmm,
		.valid = std::isfinite(total) && total > 0
			  && std::isfinite(inclination),
	};
	if (!result.valid) {
		return false;
	}
	saveCache(requestedModel, result);
	logger.info(
		"NOAA %s: %.2f uT, inclination %.2f deg, declination %.2f deg",
		requestedName,
		result.totalIntensityMicroTesla,
		result.inclinationDegrees,
		result.declinationDegrees
	);
	return true;
}

bool WmmService::resolveReference() {
	WmmReference candidate{};
	if ((model == WmmModel::Auto || model == WmmModel::WmmHr) && ENABLE_WMMHR) {
		if (requestReference(WmmModel::WmmHr, candidate)) {
			reference = candidate;
			return true;
		}
		if (loadCache(WmmModel::WmmHr, candidate)) {
			candidate.source = WmmSource::CacheWmmHr;
			reference = candidate;
			return true;
		}
	}

	if ((model == WmmModel::Auto || model == WmmModel::Wmm)
		&& ENABLE_WMM_STANDARD_FALLBACK) {
		if (requestReference(WmmModel::Wmm, candidate)) {
			reference = candidate;
			return true;
		}
		if (loadCache(WmmModel::Wmm, candidate)) {
			candidate.source = WmmSource::CacheWmm;
			reference = candidate;
			return true;
		}
	}

	reference = {};
	logger.warn("No WMM reference available; using ordinary magnetometer fusion");
	return false;
}

void WmmService::applyReference() {
	if (mode != WmmMode::On || !reference.valid) {
		sensorManager.clearMagneticReference();
		if (mode == WmmMode::Monitor && reference.valid) {
			logger.info("Monitor mode: reference is not influencing fusion");
		}
		return;
	}
	sensorManager.setMagneticReference(
		reference.totalIntensityMicroTesla,
		reference.inclinationDegrees * static_cast<float>(PI / 180.0)
	);
	logger.info("Reference applied to VQF as a soft norm/dip prior");
}

bool WmmService::loadSettings() {
	if (!LittleFS.exists(SettingsPath)) {
		return false;
	}
	File file = LittleFS.open(SettingsPath, "r");
	if (!file || file.size() != sizeof(PersistedSettings)) {
		return false;
	}
	PersistedSettings saved{};
	file.read(reinterpret_cast<uint8_t*>(&saved), sizeof(saved));
	file.close();
	const uint32_t expected = saved.checksum;
	saved.checksum = 0;
	if (saved.magic != SettingsMagic || saved.version != SettingsVersion
		|| expected != checksum(reinterpret_cast<const uint8_t*>(&saved), sizeof(saved))) {
		return false;
	}
	mode = static_cast<WmmMode>(saved.mode);
	model = static_cast<WmmModel>(saved.model);
	autoLocation = saved.autoLocation != 0;
	location = saved.location;
	return true;
}

bool WmmService::saveSettings() {
	PersistedSettings saved{};
	saved.mode = static_cast<uint8_t>(mode);
	saved.model = static_cast<uint8_t>(model);
	saved.autoLocation = autoLocation ? 1 : 0;
	saved.location = location;
	saved.checksum
		= checksum(reinterpret_cast<const uint8_t*>(&saved), sizeof(saved));
	File file = LittleFS.open(SettingsPath, "w");
	if (!file) {
		return false;
	}
	const bool ok = file.write(reinterpret_cast<const uint8_t*>(&saved), sizeof(saved))
				 == sizeof(saved);
	file.close();
	return ok;
}

bool WmmService::loadCache(WmmModel requestedModel, WmmReference& result) {
	const char* path
		= requestedModel == WmmModel::WmmHr ? WmmHrCachePath : WmmCachePath;
	if (!LittleFS.exists(path)) {
		return false;
	}
	File file = LittleFS.open(path, "r");
	if (!file || file.size() != sizeof(PersistedCache)) {
		return false;
	}
	PersistedCache saved{};
	file.read(reinterpret_cast<uint8_t*>(&saved), sizeof(saved));
	file.close();
	const uint32_t expected = saved.checksum;
	saved.checksum = 0;
	if (saved.magic != CacheMagic || saved.version != CacheVersion
		|| saved.model != static_cast<uint8_t>(requestedModel)
		|| expected != checksum(reinterpret_cast<const uint8_t*>(&saved), sizeof(saved))
		|| !saved.reference.valid) {
		return false;
	}
	result = saved.reference;
	logger.info("Loaded cached %s reference", modelName(requestedModel));
	return true;
}

bool WmmService::saveCache(WmmModel requestedModel, const WmmReference& value) {
	PersistedCache saved{};
	saved.model = static_cast<uint8_t>(requestedModel);
	saved.reference = value;
	saved.checksum
		= checksum(reinterpret_cast<const uint8_t*>(&saved), sizeof(saved));
	const char* path
		= requestedModel == WmmModel::WmmHr ? WmmHrCachePath : WmmCachePath;
	File file = LittleFS.open(path, "w");
	if (!file) {
		return false;
	}
	const bool ok = file.write(reinterpret_cast<const uint8_t*>(&saved), sizeof(saved))
				 == sizeof(saved);
	file.close();
	return ok;
}

bool WmmService::extractJsonNumber(
	const String& body,
	const char* const* names,
	size_t nameCount,
	double& value
) {
	for (size_t i = 0; i < nameCount; i++) {
		String needle = String("\"") + names[i] + "\"";
		int position = body.indexOf(needle);
		if (position < 0) {
			continue;
		}
		position = body.indexOf(':', position + needle.length());
		if (position < 0) {
			continue;
		}
		position++;
		while (position < static_cast<int>(body.length())
			   && (body[position] == ' ' || body[position] == '"')) {
			position++;
		}
		char* end = nullptr;
		value = strtod(body.c_str() + position, &end);
		if (end != body.c_str() + position && std::isfinite(value)) {
			return true;
		}
	}
	return false;
}

bool WmmService::httpGet(const String& url, String& response) {
	NetworkClientSecure client;
	client.setCACert(
		url.startsWith(IpLocationUrl) ? Certificates::IpLocationRoot
									: Certificates::NoaaRoot
	);
	HTTPClient http;
	http.setConnectTimeout(8000);
	http.setTimeout(10000);
	if (!http.begin(client, url)) {
		return false;
	}
	http.addHeader("User-Agent", "SlimeVR-Tracker-ESP/WMM");
	const int status = http.GET();
	if (status < 200 || status >= 300) {
		http.end();
		return false;
	}
	response = http.getString();
	http.end();
	return response.length() > 0;
}

uint32_t WmmService::checksum(const uint8_t* data, size_t size) {
	uint32_t hash = 2166136261u;
	for (size_t i = 0; i < size; i++) {
		hash ^= data[i];
		hash *= 16777619u;
	}
	return hash;
}

const char* WmmService::modeName(WmmMode value) {
	switch (value) {
		case WmmMode::Off:
			return "OFF";
		case WmmMode::Monitor:
			return "MONITOR";
		case WmmMode::On:
			return "ON";
	}
	return "UNKNOWN";
}

const char* WmmService::modelName(WmmModel value) {
	switch (value) {
		case WmmModel::Auto:
			return "AUTO";
		case WmmModel::WmmHr:
			return "WMMHR";
		case WmmModel::Wmm:
			return "WMM";
	}
	return "UNKNOWN";
}

const char* WmmService::sourceName(WmmSource value) {
	switch (value) {
		case WmmSource::None:
			return "NONE";
		case WmmSource::NoaaWmmHr:
			return "NOAA WMMHR";
		case WmmSource::CacheWmmHr:
			return "CACHED WMMHR";
		case WmmSource::NoaaWmm:
			return "NOAA WMM";
		case WmmSource::CacheWmm:
			return "CACHED WMM";
	}
	return "UNKNOWN";
}

void WmmService::printStatus() const {
	logger.info(
		"Mode=%s model=%s source=%s valid=%s auto-location=%s",
		modeName(mode),
		modelName(model),
		sourceName(reference.source),
		reference.valid ? "yes" : "no",
		autoLocation ? "on" : "off"
	);
	if (reference.valid) {
		logger.info(
			"Field %.2f uT, inclination %.2f deg, declination %.2f deg",
			reference.totalIntensityMicroTesla,
			reference.inclinationDegrees,
			reference.declinationDegrees
		);
	}
}

void WmmService::printLocation() const {
	if (!location.valid) {
		logger.info("No saved location");
		return;
	}
	logger.info(
		"Location %.6f, %.6f, altitude %.3f km (%s)",
		location.latitude,
		location.longitude,
		location.altitudeKm,
		location.manual ? "manual" : "IP lookup"
	);
}

}  // namespace SlimeVR::Magnetic

#else

namespace SlimeVR::Magnetic {
void WmmService::setup() {}
void WmmService::resolveStartup() {}
void WmmService::refresh() {}
void WmmService::setMode(WmmMode value) { mode = value; }
void WmmService::setModel(WmmModel value) { model = value; }
void WmmService::setAutoLocation(bool enabled) { autoLocation = enabled; }
bool WmmService::setManualLocation(double, double, float) { return false; }
void WmmService::clearLocation() {}
void WmmService::clearCache() {}
void WmmService::printStatus() const {}
void WmmService::printLocation() const {}
}  // namespace SlimeVR::Magnetic

#endif
