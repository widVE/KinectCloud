#pragma once

#include "kinectUtil.h"

namespace kinectCloud {
	// wrapper for azure kinect
	class azureKinectDK {
		k4a_device_t _device = nullptr;
		k4a_transformation_t _transform = nullptr;
		k4a_capture_t _capture = nullptr;

		k4a_calibration_t _cali;
		std::string _serial;
	public:

		// call kinect api to retrieve serial number of this device
		inline std::string getSerialNum() {
			if (_serial.empty()) {
				size_t resSize = 0;
				if (K4A_BUFFER_RESULT_FAILED == k4a_device_get_serialnum(_device, nullptr, &resSize)) {
					throw std::runtime_error("failed to get device serila number");
				}
				char*data = new char[resSize + 1];
				if (K4A_BUFFER_RESULT_FAILED == k4a_device_get_serialnum(_device, data, &resSize)) {
					throw std::runtime_error("failed to get device serila number");
				}
				std::string res = data;
				delete[] data;
				_serial = res;
			}
			return _serial;
		}

		// set exposure time in nanoseconds, 0 => automatic exposure settings
		inline void setExposure(int nanos) {
			if (K4A_RESULT_SUCCEEDED != k4a_device_set_color_control(
				_device,
				K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
				nanos ? K4A_COLOR_CONTROL_MODE_MANUAL : K4A_COLOR_CONTROL_MODE_AUTO,
				nanos
			)) {
				throw std::runtime_error("failed to set color control");
			}
		}

		// set white balance in kelvin, value must be divisible by 10, 0 => automatic
		inline void setWhiteBalance(int kelvin) {
			if (K4A_RESULT_SUCCEEDED != k4a_device_set_color_control(
				_device,
				K4A_COLOR_CONTROL_WHITEBALANCE,
				kelvin ? K4A_COLOR_CONTROL_MODE_MANUAL : K4A_COLOR_CONTROL_MODE_AUTO,
				kelvin
			)) {
				throw std::runtime_error("failed to set color control");
			}
		}

		// transform current frame into point cloud and save it
		inline void saveCurrentPointCloud(std::string const& filePath) {
			k4a_image_t depthImage = k4a_capture_get_depth_image(_capture);
			k4a_image_t colorImage = k4a_capture_get_color_image(_capture);
			k4a_image_t transformedDepthImage = nullptr;
			k4a_image_t xyzImage = nullptr;

			uint32_t resWidth = k4a_image_get_width_pixels(colorImage);
			uint32_t resHeight = k4a_image_get_height_pixels(colorImage);
			uint32_t colorStride = resWidth * sizeof(uint8_t) * 4;
			uint32_t xyzStride = resWidth * sizeof(int16_t) * 3;
			uint32_t transformedDepthStride = resWidth * sizeof(uint16_t);

			if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, resWidth, resHeight, transformedDepthStride, &transformedDepthImage)) {
				k4a_image_release(depthImage);
				k4a_image_release(colorImage);
				throw std::runtime_error("failed to create mapped color image");
			}

			if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera(
				_transform,
				depthImage,
				transformedDepthImage
			)) {
				k4a_image_release(depthImage);
				k4a_image_release(colorImage);
				k4a_image_release(transformedDepthImage);
				throw std::runtime_error("failed to transform depth image to color image space");
			}

			k4a_image_release(depthImage);

			if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, resWidth, resHeight, xyzStride, &xyzImage)) {
				k4a_image_release(colorImage);
				k4a_image_release(transformedDepthImage);
				throw std::runtime_error("failed to create point cloud image");
			}

			if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(
				_transform,
				transformedDepthImage,
				K4A_CALIBRATION_TYPE_COLOR,
				xyzImage
			)) {
				k4a_image_release(colorImage);
				k4a_image_release(xyzImage);
				k4a_image_release(transformedDepthImage);
				throw std::runtime_error("failed to transform mapped depth to point cloud image");
			}

			k4a_image_release(transformedDepthImage);

			savePointCloud(glm::uvec2(resWidth, resHeight), xyzImage, colorImage, filePath);

			k4a_image_release(colorImage);
			k4a_image_release(xyzImage);
		}

		// get the next frame and keep it in memory until next frame is retrieved
		inline void captureFrame() {
			if (_capture != nullptr) {
				k4a_capture_release(_capture);
				_capture = nullptr;
			}

			if (k4a_device_get_capture(_device, &_capture, K4A_WAIT_INFINITE) != K4A_WAIT_RESULT_SUCCEEDED) {
				throw std::runtime_error("capture failed");
			}
		}

		// start cameras
		inline void start(k4a_wired_sync_mode_t auth, k4a_color_resolution_t colorRes, k4a_depth_mode_t depthMode) {
			k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
			config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
			config.color_resolution = colorRes;
			config.depth_mode = depthMode;
			config.camera_fps = K4A_FRAMES_PER_SECOND_5;
			config.synchronized_images_only = true;
			config.wired_sync_mode = auth;

			if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(_device, &config)) {
				throw std::runtime_error("failed to start cameras");
			}

			if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(_device, config.depth_mode, config.color_resolution, &_cali)) {
				throw std::runtime_error("failed to get device calibration");
			}

			_transform = k4a_transformation_create(&_cali);
		}

		// open deviec with given serial number
		inline azureKinectDK(std::string const& serialNum) {
			uint32_t deviceCount = k4a_device_get_installed_count();
			for (uint32_t i = 0; i < deviceCount; i++) {
				if (K4A_RESULT_SUCCEEDED == k4a_device_open(i, &_device)) {
					if (getSerialNum() != serialNum) {
						k4a_device_close(_device);
						_device = nullptr;
					}
					else {
						return;
					}
				}
			}
			_device = nullptr;
			throw std::runtime_error("Can't open device with given serial #");
		}

		// open device with given index
		inline azureKinectDK(uint32_t deviceIndex) {
			if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIndex, &_device)) {
				_device = nullptr;
				throw std::runtime_error("Can't open device");
			}
		}

		// copy constructor removed
		inline azureKinectDK(azureKinectDK const& other) = delete;

		// copy assignment removed
		inline azureKinectDK& operator=(azureKinectDK const& other) = delete;

		// move constructor (needed for use in std::vector)
		inline azureKinectDK(azureKinectDK &&other) noexcept {
			move(other);
		}

		// move assignment (needed for use in std::vector)
		inline azureKinectDK& operator=(azureKinectDK &&other) noexcept {
			move(other);
		}

		// destructor
		inline ~azureKinectDK() {
			if (_transform) {
				k4a_transformation_destroy(_transform);
			}
			if (_capture) {
				k4a_capture_release(_capture);
			}
			if (_device) {
				k4a_device_stop_cameras(_device);
				k4a_device_close(_device);
			}
		}

		// get number of plugged in devices
		static inline uint32_t getNumDevices() {
			return k4a_device_get_installed_count();
		}

	private:

		// copy values from other to this, then clear values from other
		inline void move(azureKinectDK &other) {
			_device = other._device;
			_cali = other._cali;
			_transform = other._transform;
			_capture = other._capture;
			_serial = other._serial;

			other._device = nullptr;
			other._capture = nullptr;
			other._transform = nullptr;
			other._serial.clear();
		}
	};
}