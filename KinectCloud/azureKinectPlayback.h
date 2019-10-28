#pragma once

#include "kinectUtil.h"
#include "k4arecord/playback.h"

namespace kinectCloud {
	// wrapper for azure kinect
	class azureKinectPlayback {
		k4a_playback_t _playback = nullptr;
		k4a_transformation_t _transform = nullptr;
		k4a_capture_t _capture = nullptr;

		k4a_calibration_t _cali;
		bool _eof = false;
	public:

		inline int framerate() {
			k4a_record_configuration_t conf;
			k4a_playback_get_record_configuration(_playback, &conf);
			switch (conf.camera_fps)
			{
			case K4A_FRAMES_PER_SECOND_5:
				return 5;
				break;
			case K4A_FRAMES_PER_SECOND_15:
				return 15;
				break;
			case K4A_FRAMES_PER_SECOND_30:
				return 30;
				break;
			default:
				return 0;
				break;
			}
		}

		// transform current frame into point cloud and save it
		// may not save anything, if the image is not synchronized
		// returns false if capture is not valid, otherwise true
		inline bool saveCurrentPointCloud(std::string const& filePath) {
			k4a_image_t depthImage = k4a_capture_get_depth_image(_capture);
			k4a_image_t colorImage = k4a_capture_get_color_image(_capture);
			if (_capture == nullptr || depthImage == nullptr || colorImage == nullptr) return false;

			auto a = k4a_image_get_stride_bytes(colorImage);
			auto b = k4a_image_get_size(colorImage);
			auto c = k4a_image_get_format(colorImage);
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

			return true;
		}

		// jump to frame with t >= time and load capture
		inline void seekTime(int64_t usec) {
			k4a_playback_seek_timestamp(_playback, usec, K4A_PLAYBACK_SEEK_BEGIN);
			nextCapture();
		}

		// increment stream to next frame and keep it in memory until next frame is retrieved
		// if eof is reached capture is invalidated
		// returns false if eof reached, true otherwise
		inline bool nextCapture() {
			if (_capture != nullptr) {
				k4a_capture_release(_capture);
				_capture = nullptr;
			}

			k4a_stream_result_t res = k4a_playback_get_next_capture(_playback, &_capture);
			_eof = (res == K4A_STREAM_RESULT_EOF);
			if (res == K4A_STREAM_RESULT_FAILED) {
				throw std::runtime_error("next capture failed");
				_capture = nullptr;
			}

			return !_eof;
		}

		inline bool eof() const {
			return _eof;
		}

		// open playback with given serial number
		inline azureKinectPlayback(std::string const& path) {
			if (K4A_RESULT_SUCCEEDED != k4a_playback_open(path.c_str(), &_playback)) {
				_playback = nullptr;
				throw std::runtime_error("failed to open playback");
			}
			if (K4A_RESULT_SUCCEEDED != k4a_playback_set_color_conversion(_playback, K4A_IMAGE_FORMAT_COLOR_BGRA32)) {
				throw std::runtime_error("failed to set playback color conversion");
			}
			if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(_playback, &_cali)) {
				throw std::runtime_error("failed to retrieve playback calibration");
			}
			_transform = k4a_transformation_create(&_cali);
		}

		// copy constructor removed
		inline azureKinectPlayback(azureKinectPlayback const& other) = delete;

		// copy assignment removed
		inline azureKinectPlayback& operator=(azureKinectPlayback const& other) = delete;

		// move constructor (needed for use in std::vector)
		inline azureKinectPlayback(azureKinectPlayback&& other) noexcept {
			move(other);
		}

		// move assignment (needed for use in std::vector)
		inline azureKinectPlayback& operator=(azureKinectPlayback&& other) noexcept {
			move(other);
		}

		// destructor
		inline ~azureKinectPlayback() {
			if (_transform) {
				k4a_transformation_destroy(_transform);
			}
			if (_capture) {
				k4a_capture_release(_capture);
			}
			if (_playback) {
				k4a_playback_close(_playback);
			}
		}

	private:

		// copy values from other to this, then clear values from other
		inline void move(azureKinectPlayback& other) {
			_playback = other._playback;
			_cali = other._cali;
			_transform = other._transform;
			_capture = other._capture;
			_eof = other._eof;

			other._playback = nullptr;
			other._capture = nullptr;
			other._transform = nullptr;
			_eof = false;
		}
	};
}