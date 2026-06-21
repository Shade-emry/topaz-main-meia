/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Gorbit99 & SlimeVR Contributors
	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:
	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.
	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

#include "../../../sensorinterface/RegisterInterface.h"
#include "callbacks.h"
#include "sensors/softfusion/magdriver.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses acceleration range at 32g
// and gyroscope range at 4000dps
// using high resolution mode
// Uses the internal clock
// Gyroscope ODR = 400Hz, accel ODR = 400Hz
// FIFO timestamps are enabled and used to derive actual sample intervals.

struct ICM45Base {
	static constexpr uint8_t Address = 0x68;

	static constexpr float GyrTs = 1.0 / 400.0;
	static constexpr float AccTs = 1.0 / 400.0;
	static constexpr float TempTs = 1.0 / 400.0;

	static constexpr float MagTs = 1.0 / 100;

	static constexpr float GyroSensitivity = 131.072f;
	static constexpr float AccelSensitivity = 16384.0f;

	static constexpr float TemperatureBias = 25.0f;
	static constexpr float TemperatureSensitivity = 128.0f;

	static constexpr float TemperatureZROChange = 20.0f;

	RegisterInterface& m_RegisterInterface;
	SlimeVR::Logging::Logger& m_Logger;
	ICM45Base(RegisterInterface& registerInterface, SlimeVR::Logging::Logger& logger)
		: m_RegisterInterface(registerInterface)
		, m_Logger(logger) {}

	struct BaseRegs {
		static constexpr uint8_t TempData = 0x0c;

		struct DeviceConfig {
			static constexpr uint8_t reg = 0x7f;
			static constexpr uint8_t valueSwReset = 0b10;
		};

		struct GyroConfig {
			static constexpr uint8_t reg = 0x1c;
			static constexpr uint8_t value
				= (0b0000 << 4) | 0b0111;  // 4000dps, ODR=400Hz
		};

		struct AccelConfig {
			static constexpr uint8_t reg = 0x1b;
			static constexpr uint8_t value
				= (0b000 << 4) | 0b0111;  // 32g, ODR=400Hz
		};

		struct FifoConfig0 {
			static constexpr uint8_t reg = 0x1d;
			static constexpr uint8_t value
				= (0b01 << 6) | (0b011110);  // stream to FIFO mode, FIFO depth
											 // Maximum usable depth; 0x1F is
											 // affected by the FIFO-depth erratum.
											 // This disables all APEX
											 // features, but we don't need them
		};

		struct FifoConfig3 {
			static constexpr uint8_t reg = 0x21;
			static constexpr uint8_t value = (0b1 << 0) | (0b1 << 1) | (0b1 << 2)
										   | (0b1 << 3);  // enable FIFO,
														  // enable accel,
														  // enable gyro,
														  // enable hires mode
		};

		struct FifoConfig4 {
			static constexpr uint8_t reg = 0x22;
			static constexpr uint8_t valueTimestampEnabled = (0b1 << 1);
		};

		struct TmstWomConfig {
			static constexpr uint8_t reg = 0x23;
			static constexpr uint8_t resolution16us = (0b1 << 5);
		};

		struct PwrMgmt0 {
			static constexpr uint8_t reg = 0x10;
			static constexpr uint8_t value
				= 0b11 | (0b11 << 2);  // accel in low noise mode, gyro in low noise
		};

		static constexpr uint8_t FifoCount = 0x12;
		static constexpr uint8_t FifoData = 0x14;

		// Indirect Register Access

		static constexpr uint32_t IRegWaitTimeMicros = 4;

		enum class Bank : uint8_t {
			IMemSram = 0x00,
			IPregBar = 0xa0,
			IPregSys1 = 0xa4,
			IPregSys2 = 0xa5,
			IPregTop1 = 0xa2,
		};

		static constexpr uint8_t IRegAddr = 0x7c;
		static constexpr uint8_t IRegData = 0x7e;

		// Mag Support

		struct IOCPadScenarioAuxOvrd {
			static constexpr uint8_t reg = 0x30;
			static constexpr uint8_t value = (0b1 << 4)  // Enable AUX1 override
										   | (0b01 << 2)  // Enable I2CM master
										   | (0b1 << 1)  // Enable AUX1 enable override
										   | (0b1 << 0);  // Enable AUX1
		};

		struct I2CMCommand0 {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x06;
		};

		struct I2CMDevProfile0 {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x0e;
		};

		struct I2CMDevProfile1 {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x0f;
		};

		struct I2CMWrData0 {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x33;
		};

		struct I2CMRdData0 {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x1b;
		};

		struct DmpExtSenOdrCfg {
			// TODO: todo
		};

		struct I2CMControl {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x16;
		};

		struct I2CMStatus {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x18;

			static constexpr uint8_t SDAErr = 0b1 << 5;
			static constexpr uint8_t SCLErr = 0b1 << 4;
			static constexpr uint8_t SRSTErr = 0b1 << 3;
			static constexpr uint8_t TimeoutErr = 0b1 << 2;
			static constexpr uint8_t Done = 0b1 << 1;
			static constexpr uint8_t Busy = 0b1 << 0;
			static constexpr uint8_t ErrorMask
				= SDAErr | SCLErr | SRSTErr | TimeoutErr;
		};

		struct SmcControl0 {
			static constexpr Bank bank = Bank::IPregTop1;
			static constexpr uint8_t reg = 0x58;
			static constexpr uint8_t timestampEnable = (0b1 << 0);
		};
	};

#pragma pack(push, 1)
	struct FifoEntryAligned {
		int16_t accel[3];
		int16_t gyro[3];
		uint16_t temp;
		uint16_t timestamp;
		uint8_t lsb[3];
	};
#pragma pack(pop)

	static constexpr size_t FullFifoEntrySize = sizeof(FifoEntryAligned) + 1;

	void softResetIMU() {
		m_RegisterInterface.writeReg(
			BaseRegs::DeviceConfig::reg,
			BaseRegs::DeviceConfig::valueSwReset
		);
		delay(35);
	}

	bool initializeBase() {
		// perform initialization step
		m_RegisterInterface.writeReg(
			BaseRegs::GyroConfig::reg,
			BaseRegs::GyroConfig::value
		);
		m_RegisterInterface.writeReg(
			BaseRegs::AccelConfig::reg,
			BaseRegs::AccelConfig::value
		);
		m_RegisterInterface.writeReg(
			BaseRegs::FifoConfig0::reg,
			BaseRegs::FifoConfig0::value
		);
		m_RegisterInterface.writeReg(
			BaseRegs::FifoConfig3::reg,
			BaseRegs::FifoConfig3::value
		);
		uint8_t fifoConfig4
			= m_RegisterInterface.readReg(BaseRegs::FifoConfig4::reg);
		fifoConfig4 |= BaseRegs::FifoConfig4::valueTimestampEnabled;
		m_RegisterInterface.writeReg(BaseRegs::FifoConfig4::reg, fifoConfig4);
		m_RegisterInterface.writeReg(
			BaseRegs::PwrMgmt0::reg,
			BaseRegs::PwrMgmt0::value
		);

		// Configure the timestamp counter before consuming FIFO frames.
		uint8_t tmstConfig = m_RegisterInterface.readReg(BaseRegs::TmstWomConfig::reg);
		tmstConfig &= ~BaseRegs::TmstWomConfig::resolution16us;
		m_RegisterInterface.writeReg(BaseRegs::TmstWomConfig::reg, tmstConfig);

		uint8_t smcControl = readBankRegister<typename BaseRegs::SmcControl0>();
		smcControl |= BaseRegs::SmcControl0::timestampEnable;
		writeBankRegister<typename BaseRegs::SmcControl0>(smcControl);

		m_RegisterInterface.writeReg(
			BaseRegs::IOCPadScenarioAuxOvrd::reg,
			BaseRegs::IOCPadScenarioAuxOvrd::value
		);

		read_buffer.resize(FullFifoEntrySize * MaxReadings);
		resetTimestamps();

		delay(1);

		return true;
	}

	static constexpr size_t MaxReadings = 8;
	// Allocate on heap so that it does not take up stack space, which can result in
	// stack overflow and panic
	std::vector<uint8_t> read_buffer;

	bool bulkRead(DriverCallbacks<int32_t>&& callbacks) {
		constexpr int16_t InvalidReading = -32768;

		// AN-000364 (2.2): read FIFO_COUNT twice and use the second value.
		m_RegisterInterface.readReg16(BaseRegs::FifoCount);
		size_t fifo_packets = m_RegisterInterface.readReg16(BaseRegs::FifoCount);

		if (fifo_packets <= 1) {
			pollAux(callbacks);
			return false;
		}

		// AN-000364
		// 2.16 FIFO EMPTY EVENT IN STREAMING MODE CAN CORRUPT FIFO DATA
		//
		// Description: When in FIFO streaming mode, a FIFO empty event
		// (caused by host reading the last byte of the last FIFO frame) can
		// cause FIFO data corruption in the first FIFO frame that arrives
		// after the FIFO empty condition. Once the issue is triggered, the
		// FIFO state is compromised and cannot recover. FIFO must be set in
		// bypass mode to flush out the wrong state
		//
		// When operating in FIFO streaming mode, if FIFO threshold
		// interrupt is triggered with M number of FIFO frames accumulated
		// in the FIFO buffer, the host should only read the first M-1
		// number of FIFO frames. This prevents the FIFO empty event, that
		// can cause FIFO data corruption, from happening.
		--fifo_packets;

		auto packets_to_read = std::min(fifo_packets, MaxReadings);

		size_t bytes_to_read = packets_to_read * FullFifoEntrySize;
		m_RegisterInterface
			.readBytes(BaseRegs::FifoData, bytes_to_read, read_buffer.data());

		for (auto i = 0u; i < bytes_to_read; i += FullFifoEntrySize) {
			uint8_t header = read_buffer[i];
			bool has_gyro = header & (1 << 5);
			bool has_accel = header & (1 << 6);
			bool has_timestamp = header & (1 << 3);

			const uint8_t* frame = &read_buffer[i + 1];
			FifoEntryAligned entry{
				.accel =
					{readLittleEndianInt16(frame + 0),
					 readLittleEndianInt16(frame + 2),
					 readLittleEndianInt16(frame + 4)},
				.gyro =
					{readLittleEndianInt16(frame + 6),
					 readLittleEndianInt16(frame + 8),
					 readLittleEndianInt16(frame + 10)},
				.temp = readLittleEndianUint16(frame + 12),
				.timestamp = readLittleEndianUint16(frame + 14),
				.lsb = {frame[16], frame[17], frame[18]},
			};

			// Process temperature first so this frame's gyro sample is corrected
			// using the matching temperature.
			if (static_cast<int16_t>(entry.temp) != InvalidReading) {
				callbacks.processTempSample(
					static_cast<int16_t>(entry.temp),
					getSampleDelta(
						entry.timestamp,
						TempTs,
						lastTempTimestamp,
						tempTimestampInitialized,
						has_timestamp
					)
				);
			}

			if (has_gyro && entry.gyro[0] != InvalidReading) {
				const int32_t gyroData[3]{
					decode20Bit(entry.gyro[0], entry.lsb[0] & 0x0f),
					decode20Bit(entry.gyro[1], entry.lsb[1] & 0x0f),
					decode20Bit(entry.gyro[2], entry.lsb[2] & 0x0f),
				};
				callbacks.processGyroSample(
					gyroData,
					getSampleDelta(
						entry.timestamp,
						GyrTs,
						lastGyroTimestamp,
						gyroTimestampInitialized,
						has_timestamp
					)
				);
			}

			if (has_accel && entry.accel[0] != InvalidReading) {
				const int32_t accelData[3]{
					decode20Bit(entry.accel[0], (entry.lsb[0] & 0xf0) >> 4),
					decode20Bit(entry.accel[1], (entry.lsb[1] & 0xf0) >> 4),
					decode20Bit(entry.accel[2], (entry.lsb[2] & 0xf0) >> 4),
				};
				callbacks.processAccelSample(
					accelData,
					getSampleDelta(
						entry.timestamp,
						AccTs,
						lastAccelTimestamp,
						accelTimestampInitialized,
						has_timestamp
					)
				);
			}
		}

		pollAux(callbacks);
		return fifo_packets > MaxReadings;
	}

	static int16_t readLittleEndianInt16(const uint8_t* data) {
		return static_cast<int16_t>(readLittleEndianUint16(data));
	}

	static uint16_t readLittleEndianUint16(const uint8_t* data) {
		return static_cast<uint16_t>(data[0])
			 | (static_cast<uint16_t>(data[1]) << 8);
	}

	static int32_t decode20Bit(int16_t high16, uint8_t low4) {
		uint32_t raw = (static_cast<uint32_t>(static_cast<uint16_t>(high16)) << 4)
					 | (low4 & 0x0f);
		if (raw & 0x80000) {
			raw |= 0xfff00000;
		}
		return static_cast<int32_t>(raw);
	}

	static float getSampleDelta(
		uint16_t timestamp,
		float nominalDelta,
		uint16_t& lastTimestamp,
		bool& initialized,
		bool timestampValid
	) {
		if (!timestampValid) {
			initialized = false;
			return nominalDelta;
		}

		if (!initialized) {
			lastTimestamp = timestamp;
			initialized = true;
			return nominalDelta;
		}

		const uint16_t ticks = timestamp - lastTimestamp;
		lastTimestamp = timestamp;
		const float measuredDelta = static_cast<float>(ticks) * TimestampTickSeconds;

		if (measuredDelta < nominalDelta * 0.25f
			|| measuredDelta > nominalDelta * 8.0f) {
			return nominalDelta;
		}
		return measuredDelta;
	}

	void resetTimestamps() {
		gyroTimestampInitialized = false;
		accelTimestampInitialized = false;
		tempTimestampInitialized = false;
	}

	template <typename Reg>
	uint8_t readBankRegister() {
		uint8_t buffer;
		readBankRegister<Reg>(&buffer, sizeof(buffer));
		return buffer;
	}

	template <typename Reg, typename T>
	void readBankRegister(T* buffer, size_t length) {
		uint8_t data[] = {
			static_cast<uint8_t>(Reg::bank),
			Reg::reg,
		};

		auto* bufferBytes = reinterpret_cast<uint8_t*>(buffer);
		m_RegisterInterface.writeBytes(BaseRegs::IRegAddr, sizeof(data), data);
		delayMicroseconds(BaseRegs::IRegWaitTimeMicros);
		for (size_t i = 0; i < length * sizeof(T); i++) {
			bufferBytes[i] = m_RegisterInterface.readReg(BaseRegs::IRegData);
			delayMicroseconds(BaseRegs::IRegWaitTimeMicros);
		}
	}

	template <typename Reg>
	void writeBankRegister() {
		writeBankRegister<Reg>(&Reg::value, sizeof(Reg::value));
	}

	template <typename Reg, typename T>
	void writeBankRegister(T* buffer, size_t length) {
		auto* bufferBytes = reinterpret_cast<uint8_t*>(buffer);

		uint8_t data[] = {
			static_cast<uint8_t>(Reg::bank),
			Reg::reg,
			bufferBytes[0],
		};

		m_RegisterInterface.writeBytes(BaseRegs::IRegAddr, sizeof(data), data);
		delayMicroseconds(BaseRegs::IRegWaitTimeMicros);
		for (size_t i = 1; i < length * sizeof(T); i++) {
			m_RegisterInterface.writeReg(BaseRegs::IRegData, bufferBytes[i]);
			delayMicroseconds(BaseRegs::IRegWaitTimeMicros);
		}
	}

	template <typename Reg>
	void writeBankRegister(uint8_t value) {
		writeBankRegister<Reg>(&value, sizeof(value));
	}

	void setAuxId(uint8_t deviceId) {
		// I2CM_DEV_PROFILE stores a 7-bit address.
		writeBankRegister<typename BaseRegs::I2CMDevProfile1>(deviceId & 0x7f);
	}

	uint8_t readAux(uint8_t address) {
		uint8_t value = 0xff;
		readAuxBytes(address, &value, sizeof(value));
		return value;
	}

	bool readAuxBytes(uint8_t address, uint8_t* values, size_t length) {
		if (values == nullptr || length == 0 || length > 0x0f) {
			return false;
		}

		writeBankRegister<typename BaseRegs::I2CMDevProfile0>(address);

		writeBankRegister<typename BaseRegs::I2CMCommand0>(
			(0b1 << 7)  // Last transaction
			| (0b0 << 6)  // Channel 0
			| (0b01 << 4)  // Read with register
			| static_cast<uint8_t>(length)
		);
		if (!executeAuxTransaction("read", address)) {
			return false;
		}

		readBankRegister<typename BaseRegs::I2CMRdData0>(values, length);
		return true;
	}

	bool writeAux(uint8_t address, uint8_t value) {
		uint8_t writeData[]{address, value};
		writeBankRegister<typename BaseRegs::I2CMWrData0>(
			writeData,
			sizeof(writeData)
		);
		writeBankRegister<typename BaseRegs::I2CMCommand0>(
			(0b1 << 7)  // Last transaction
			| (0b0 << 6)  // Channel 0
			| (0b00 << 4)  // Write
			| (0b0010 << 0)  // Register address plus one data byte
		);
		return executeAuxTransaction("write", address);
	}

	void startAuxPolling(
		uint8_t dataReg,
		MagDataWidth dataWidth,
		float samplePeriod
	) {
		auxDataReg = dataReg;
		auxDataLength = dataWidth == MagDataWidth::NineByte ? 9 : 6;
		auxSamplePeriod = samplePeriod > 0 ? samplePeriod : MagTs;
		auxPollingEnabled = true;
		magTimestampInitialized = false;
	}

	void stopAuxPolling() {
		auxPollingEnabled = false;
		magTimestampInitialized = false;
	}

private:
	bool executeAuxTransaction(const char* operation, uint8_t address) {
		uint8_t control = readBankRegister<typename BaseRegs::I2CMControl>();
		control &= ~((0b1 << 6) | (0b1 << 3));  // No restart, fast mode
		control |= 0b1;  // Start transaction
		writeBankRegister<typename BaseRegs::I2CMControl>(control);

		const uint32_t start = micros();
		uint8_t status;
		do {
			status = readBankRegister<typename BaseRegs::I2CMStatus>();
			if (micros() - start > AuxTransactionTimeoutMicros) {
				m_Logger.error(
					"Aux %s at register 0x%02x timed out",
					operation,
					address
				);
				return false;
			}
		} while (status & BaseRegs::I2CMStatus::Busy);

		if ((status & BaseRegs::I2CMStatus::ErrorMask)
			|| !(status & BaseRegs::I2CMStatus::Done)) {
			m_Logger.error(
				"Aux %s at register 0x%02x returned status 0x%02x",
				operation,
				address,
				status
			);
			return false;
		}
		return true;
	}

	void pollAux(DriverCallbacks<int32_t>& callbacks) {
		if (!auxPollingEnabled || !callbacks.processMagSample) {
			return;
		}

		const uint32_t now = micros();
		const uint32_t elapsedMicros = now - lastMagTimestampMicros;
		const uint32_t targetMicros
			= static_cast<uint32_t>(auxSamplePeriod * 1e6f);
		if (magTimestampInitialized && elapsedMicros < targetMicros) {
			return;
		}

		uint8_t data[9]{};
		if (!readAuxBytes(auxDataReg, data, auxDataLength)) {
			return;
		}

		float delta = auxSamplePeriod;
		if (magTimestampInitialized) {
			delta = static_cast<float>(elapsedMicros) * 1e-6f;
		}
		lastMagTimestampMicros = now;
		magTimestampInitialized = true;

		const int32_t magData[3]{
			readLittleEndianInt16(data + 0),
			readLittleEndianInt16(data + 2),
			readLittleEndianInt16(data + 4),
		};
		callbacks.processMagSample(magData, delta);
	}

	static constexpr float TimestampTickSeconds = 1e-6f;
	static constexpr uint32_t AuxTransactionTimeoutMicros = 5000;
	uint16_t lastGyroTimestamp = 0;
	uint16_t lastAccelTimestamp = 0;
	uint16_t lastTempTimestamp = 0;
	bool gyroTimestampInitialized = false;
	bool accelTimestampInitialized = false;
	bool tempTimestampInitialized = false;
	bool auxPollingEnabled = false;
	uint8_t auxDataReg = 0;
	size_t auxDataLength = 0;
	float auxSamplePeriod = MagTs;
	uint32_t lastMagTimestampMicros = 0;
	bool magTimestampInitialized = false;
};

};  // namespace SlimeVR::Sensors::SoftFusion::Drivers
