#pragma once

#include "azureKinectDK.h"
#include "k4arecord/record.h"

namespace kinectCloud {
	// wrapper for azure kinect recording
	// must outlive given device
	// as of november 2019, this can record synchronized frames only while k4arecord.exe cannot
	class azureKinectRecord {
		k4a_record_t _record = nullptr;

		azureKinectDK* _dev;
	public:
		inline void writeHeader() {
			if (K4A_RESULT_SUCCEEDED != k4a_record_write_header(_record)) {
				throw std::runtime_error("failed to write header to record stream");
			}
		}

		inline void recordFrame() {
			_dev->captureFrame();

			if (K4A_RESULT_SUCCEEDED != k4a_record_write_capture(_record, _dev->getCurrCapture())) {
				throw std::runtime_error("failed to write capture to record stream");
			}
		}

		// open recording with device
		inline azureKinectRecord(azureKinectDK* dev, std::string const& path) : _dev(dev) {
			if (K4A_RESULT_SUCCEEDED != k4a_record_create(path.c_str(), dev->getDevice(), dev->getConfig(), &_record)) {
				_record = nullptr;
				throw std::runtime_error("failed to open recording");
			}
		}

		// copy constructor removed
		inline azureKinectRecord(azureKinectRecord const& other) = delete;

		// copy assignment removed
		inline azureKinectRecord& operator=(azureKinectRecord const& other) = delete;

		// move constructor (needed for use in std::vector)
		inline azureKinectRecord(azureKinectRecord&& other) noexcept {
			move(other);
		}

		// move assignment (needed for use in std::vector)
		inline azureKinectRecord& operator=(azureKinectRecord&& other) noexcept {
			move(other);
		}

		// destructor
		inline ~azureKinectRecord() {
			if (_record) {
				k4a_record_close(_record);
			}
		}

	private:

		// copy values from other to this, then clear values from other
		inline void move(azureKinectRecord& other) {
			_record = other._record;
			_dev = other._dev;

			other._record = nullptr;
			other._dev = nullptr;
		}
	};
}