#pragma once

#include <Arduino.h>

#include <cstdint>

#include "debug.h"
#include "logging/Logger.h"

namespace SlimeVR::Magnetic {

enum class WmmMode : uint8_t {
	Off = WMM_MODE_OFF,
	Monitor = WMM_MODE_MONITOR,
	On = WMM_MODE_ON,
};

enum class WmmModel : uint8_t {
	Auto,
	WmmHr,
	Wmm,
};

enum class WmmSource : uint8_t {
	None,
	NoaaWmmHr,
	CacheWmmHr,
	NoaaWmm,
	CacheWmm,
};

struct WmmLocation {
	double latitude = 0;
	double longitude = 0;
	float altitudeKm = 0;
	bool valid = false;
	bool manual = false;
};

struct WmmReference {
	float totalIntensityMicroTesla = 0;
	float horizontalIntensityMicroTesla = 0;
	float northMicroTesla = 0;
	float eastMicroTesla = 0;
	float verticalMicroTesla = 0;
	float inclinationDegrees = 0;
	float declinationDegrees = 0;
	uint32_t timestamp = 0;
	WmmLocation location{};
	WmmSource source = WmmSource::None;
	bool valid = false;
};

class WmmService {
public:
	void setup();
	void resolveStartup();
	void refresh();

	void setMode(WmmMode mode);
	void setModel(WmmModel model);
	void setAutoLocation(bool enabled);
	bool setManualLocation(double latitude, double longitude, float altitudeKm);
	void clearLocation();
	void clearCache();

	[[nodiscard]] WmmMode getMode() const { return mode; }
	[[nodiscard]] WmmModel getModel() const { return model; }
	[[nodiscard]] bool getAutoLocation() const { return autoLocation; }
	[[nodiscard]] const WmmLocation& getLocation() const { return location; }
	[[nodiscard]] const WmmReference& getReference() const { return reference; }

	void printStatus() const;
	void printLocation() const;

private:
	static constexpr uint32_t SettingsMagic = 0x574D4D53;
	static constexpr uint16_t SettingsVersion = 1;
	static constexpr uint32_t CacheMagic = 0x574D4D43;
	static constexpr uint16_t CacheVersion = 1;
	static constexpr uint32_t NetworkTimeoutMillis = 30000;
	static constexpr uint32_t TimeSyncTimeoutMillis = 10000;

	struct PersistedSettings {
		uint32_t magic = SettingsMagic;
		uint16_t version = SettingsVersion;
		uint8_t mode = WMM_DEFAULT_MODE;
		uint8_t model = static_cast<uint8_t>(WmmModel::Auto);
		uint8_t autoLocation = 1;
		uint8_t reserved[3]{};
		WmmLocation location{};
		uint32_t checksum = 0;
	};

	struct PersistedCache {
		uint32_t magic = CacheMagic;
		uint16_t version = CacheVersion;
		uint8_t model = 0;
		uint8_t reserved = 0;
		WmmReference reference{};
		uint32_t checksum = 0;
	};

	bool waitForNetwork();
	bool synchronizeClock();
	bool lookupIpLocation(WmmLocation& result);
	bool requestReference(WmmModel model, WmmReference& result);
	bool resolveReference();
	void applyReference();

	bool loadSettings();
	bool saveSettings();
	bool loadCache(WmmModel model, WmmReference& result);
	bool saveCache(WmmModel model, const WmmReference& value);

	static bool extractJsonNumber(
		const String& body,
		const char* const* names,
		size_t nameCount,
		double& value
	);
	static bool httpGet(const String& url, String& response);
	static uint32_t checksum(const uint8_t* data, size_t size);
	static const char* modeName(WmmMode value);
	static const char* modelName(WmmModel value);
	static const char* sourceName(WmmSource value);

	WmmMode mode = static_cast<WmmMode>(WMM_DEFAULT_MODE);
	WmmModel model = WmmModel::Auto;
	bool autoLocation = true;
	WmmLocation location{};
	WmmReference reference{};
	Logging::Logger logger{"WMM"};
};

}  // namespace SlimeVR::Magnetic
