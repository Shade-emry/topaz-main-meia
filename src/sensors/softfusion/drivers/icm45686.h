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

#include "icm45base.h"
#include "vqf.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses acceleration range at 32g
// and gyroscope range at 4000dps
// using high resolution mode
// Uses the internal clock
// Gyroscope ODR = 400Hz, accel ODR = 400Hz
// FIFO timestamps are consumed by ICM45Base and passed through to VQF.

struct ICM45686 : public ICM45Base {
	static constexpr auto Name = "ICM-45686";
	static constexpr auto Type = SensorTypeID::ICM45686;

	static constexpr VQFParams SensorVQFParams{
		.tauMag = 30.0f,
		.magMinUndisturbedTime = 5.0f,
		.magRejectPersistent = ENABLE_VR_MAG_REJECTION,
		.magMaxCorrectionRate = 0.20f,
	};

	ICM45686(RegisterInterface& registerInterface, SlimeVR::Logging::Logger& logger)
		: ICM45Base{registerInterface, logger} {}

	struct Regs {
		struct WhoAmI {
			static constexpr uint8_t reg = 0x72;
			static constexpr uint8_t value = 0xe9;
		};

		struct Pin9Config {
			static constexpr uint8_t reg = 0x31;
			static constexpr uint8_t mask = 0b00000111;
			static constexpr uint8_t value = 0b00000110;  // pin 9 to clkin
		};

		struct RtcConfig {
			static constexpr uint8_t reg = 0x26;
			static constexpr uint8_t enable = 0b00100000;
		};

		struct SifsI3CStcConfig {
			static constexpr BaseRegs::Bank bank = BaseRegs::Bank::IPregTop1;
			static constexpr uint8_t reg = 0x68;
			static constexpr uint8_t modeMask = (0b1 << 2);
		};

		struct AccelSourceControl {
			static constexpr BaseRegs::Bank bank = BaseRegs::Bank::IPregSys2;
			static constexpr uint8_t reg = 0x7b;
			static constexpr uint8_t mask = 0b00000011;
			static constexpr uint8_t firOnly = 0b00000001;
			static constexpr uint8_t interpolatorAndFir = 0b00000010;
		};

		struct GyroSourceControl {
			static constexpr BaseRegs::Bank bank = BaseRegs::Bank::IPregSys1;
			static constexpr uint8_t reg = 0xa6;
			static constexpr uint8_t mask = 0b01100000;
			static constexpr uint8_t firOnly = 0b00100000;
			static constexpr uint8_t interpolatorAndFir = 0b01000000;
		};

		struct GyroUiLpfBandwidth {
			static constexpr BaseRegs::Bank bank = BaseRegs::Bank::IPregSys1;
			static constexpr uint8_t reg = 0xac;
			static constexpr uint8_t mask = 0b00000111;
			static constexpr uint8_t odrDiv4 = 0b00000001;
		};

		struct AccelUiLpfBandwidth {
			static constexpr BaseRegs::Bank bank = BaseRegs::Bank::IPregSys2;
			static constexpr uint8_t reg = 0x83;
			static constexpr uint8_t mask = 0b00000111;
			static constexpr uint8_t odrDiv4 = 0b00000001;
		};
	};

	bool initialize() {
		ICM45Base::softResetIMU();
#if IMU_USE_EXTERNAL_CLOCK
		uint8_t pinConfig = m_RegisterInterface.readReg(Regs::Pin9Config::reg);
		pinConfig = (pinConfig & ~Regs::Pin9Config::mask) | Regs::Pin9Config::value;
		m_RegisterInterface.writeReg(Regs::Pin9Config::reg, pinConfig);

		uint8_t i3cStc = readBankRegister<typename Regs::SifsI3CStcConfig>();
		i3cStc &= ~Regs::SifsI3CStcConfig::modeMask;
		writeBankRegister<typename Regs::SifsI3CStcConfig>(i3cStc);

		uint8_t accelSource = readBankRegister<typename Regs::AccelSourceControl>();
		accelSource = (accelSource & ~Regs::AccelSourceControl::mask)
					| Regs::AccelSourceControl::interpolatorAndFir;
		writeBankRegister<typename Regs::AccelSourceControl>(accelSource);

		uint8_t gyroSource = readBankRegister<typename Regs::GyroSourceControl>();
		gyroSource = (gyroSource & ~Regs::GyroSourceControl::mask)
				   | Regs::GyroSourceControl::interpolatorAndFir;
		writeBankRegister<typename Regs::GyroSourceControl>(gyroSource);

		uint8_t rtcConfig = m_RegisterInterface.readReg(Regs::RtcConfig::reg);
		rtcConfig |= Regs::RtcConfig::enable;
		m_RegisterInterface.writeReg(Regs::RtcConfig::reg, rtcConfig);
#else
		uint8_t accelSource = readBankRegister<typename Regs::AccelSourceControl>();
		accelSource = (accelSource & ~Regs::AccelSourceControl::mask)
					| Regs::AccelSourceControl::firOnly;
		writeBankRegister<typename Regs::AccelSourceControl>(accelSource);

		uint8_t gyroSource = readBankRegister<typename Regs::GyroSourceControl>();
		gyroSource = (gyroSource & ~Regs::GyroSourceControl::mask)
				   | Regs::GyroSourceControl::firOnly;
		writeBankRegister<typename Regs::GyroSourceControl>(gyroSource);
#endif

		uint8_t gyroLpf = readBankRegister<typename Regs::GyroUiLpfBandwidth>();
		gyroLpf = (gyroLpf & ~Regs::GyroUiLpfBandwidth::mask)
				| Regs::GyroUiLpfBandwidth::odrDiv4;
		writeBankRegister<typename Regs::GyroUiLpfBandwidth>(gyroLpf);

		uint8_t accelLpf = readBankRegister<typename Regs::AccelUiLpfBandwidth>();
		accelLpf = (accelLpf & ~Regs::AccelUiLpfBandwidth::mask)
				 | Regs::AccelUiLpfBandwidth::odrDiv4;
		writeBankRegister<typename Regs::AccelUiLpfBandwidth>(accelLpf);

		return ICM45Base::initializeBase();
	}
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
