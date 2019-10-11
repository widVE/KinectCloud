#pragma once

#include <map>
#include <k4a/k4a.h>
#include <nlohmann/json.hpp>

#include "util.h"

namespace kinectCloud {
	using json = nlohmann::json;

	// parse depth mode from string (not case sensitive)
	k4a_depth_mode_t depthModeFromString(std::string mode) {
		mode = stringToUppercase(mode);
		std::map<std::string, k4a_depth_mode_t> modes;
		modes["NFOV_2X2BINNED"] = K4A_DEPTH_MODE_NFOV_2X2BINNED;
		modes["NFOV_UNBINNED"] = K4A_DEPTH_MODE_NFOV_UNBINNED;
		modes["WFOV_2X2BINNED"] = K4A_DEPTH_MODE_WFOV_2X2BINNED;
		modes["WFOV_UNBINNED"] = K4A_DEPTH_MODE_WFOV_UNBINNED;
		if (modes.find(mode) != modes.end()) return modes[mode];
		return K4A_DEPTH_MODE_OFF;
	}

	// parse color resolution from string (not case sensitive)
	k4a_color_resolution_t colorResolutionFromString(std::string mode) {
		mode = stringToUppercase(mode);
		std::map<std::string, k4a_color_resolution_t> modes;
		modes["K4A_COLOR_RESOLUTION_720P"] = K4A_COLOR_RESOLUTION_720P;
		modes["K4A_COLOR_RESOLUTION_1080P"] = K4A_COLOR_RESOLUTION_1080P;
		modes["K4A_COLOR_RESOLUTION_1440P"] = K4A_COLOR_RESOLUTION_1440P;
		modes["K4A_COLOR_RESOLUTION_1536P"] = K4A_COLOR_RESOLUTION_1536P;
		modes["K4A_COLOR_RESOLUTION_2160P"] = K4A_COLOR_RESOLUTION_2160P;
		modes["K4A_COLOR_RESOLUTION_3072P"] = K4A_COLOR_RESOLUTION_3072P;
		modes["COLOR_RESOLUTION_720P"] = K4A_COLOR_RESOLUTION_720P;
		modes["COLOR_RESOLUTION_1080P"] = K4A_COLOR_RESOLUTION_1080P;
		modes["COLOR_RESOLUTION_1440P"] = K4A_COLOR_RESOLUTION_1440P;
		modes["COLOR_RESOLUTION_1536P"] = K4A_COLOR_RESOLUTION_1536P;
		modes["COLOR_RESOLUTION_2160P"] = K4A_COLOR_RESOLUTION_2160P;
		modes["COLOR_RESOLUTION_3072P"] = K4A_COLOR_RESOLUTION_3072P;
		modes["720P"] = K4A_COLOR_RESOLUTION_720P;
		modes["1080P"] = K4A_COLOR_RESOLUTION_1080P;
		modes["1440P"] = K4A_COLOR_RESOLUTION_1440P;
		modes["1536P"] = K4A_COLOR_RESOLUTION_1536P;
		modes["2160P"] = K4A_COLOR_RESOLUTION_2160P;
		modes["3072P"] = K4A_COLOR_RESOLUTION_3072P;
		if (modes.find(mode) != modes.end()) return modes[mode];
		return K4A_COLOR_RESOLUTION_OFF;
	}

	// get vec2 of resolution from depth mode
	glm::uvec2 depthResFromMode(k4a_depth_mode_t mode) {
		std::map<k4a_depth_mode_t, glm::uvec2> reses;
		reses[K4A_DEPTH_MODE_NFOV_2X2BINNED] = glm::uvec2(320, 288);
		reses[K4A_DEPTH_MODE_NFOV_UNBINNED] = glm::uvec2(640, 576);
		reses[K4A_DEPTH_MODE_WFOV_2X2BINNED] = glm::uvec2(512, 512);
		reses[K4A_DEPTH_MODE_WFOV_UNBINNED] = glm::uvec2(1024, 1024);
		if (reses.find(mode) != reses.end()) return reses[mode];
		return glm::uvec2(0, 0);
	}

	// get color resolution from height of color frame
	k4a_color_resolution_t colorResolutionFromHeight(uint32_t height) {
		std::map<uint32_t, k4a_color_resolution_t> modes;
		modes[720] = K4A_COLOR_RESOLUTION_720P;
		modes[1080] = K4A_COLOR_RESOLUTION_1080P;
		modes[1440] = K4A_COLOR_RESOLUTION_1440P;
		modes[1536] = K4A_COLOR_RESOLUTION_1536P;
		modes[2160] = K4A_COLOR_RESOLUTION_2160P;
		modes[3072] = K4A_COLOR_RESOLUTION_3072P;
		if (modes.find(height) != modes.end()) return modes[height];
		return K4A_COLOR_RESOLUTION_OFF;
	}

	// get vec2 of color resolution from k4a type
	glm::uvec2 colorResFromColorK4a(k4a_color_resolution_t res) {
		std::map<uint32_t, glm::uvec2> reses;
		reses[K4A_COLOR_RESOLUTION_720P] = glm::uvec2(1280, 720);
		reses[K4A_COLOR_RESOLUTION_1080P] = glm::uvec2(1920, 1080);
		reses[K4A_COLOR_RESOLUTION_1440P] = glm::uvec2(2560, 1440);
		reses[K4A_COLOR_RESOLUTION_1536P] = glm::uvec2(2048, 1536);
		reses[K4A_COLOR_RESOLUTION_2160P] = glm::uvec2(3840, 2160);
		reses[K4A_COLOR_RESOLUTION_3072P] = glm::uvec2(4096, 3072);
		if (reses.find(res) != reses.end()) return reses[res];
		return glm::uvec2(0, 0);
	}

	// save existing xyz image as pts file
	void savePointCloud(glm::uvec2 size, k4a_image_t xyzImg, k4a_image_t colorImg, std::string const& loc) {
		uint32_t resWidth = size.x, resHeight = size.y;
		uint32_t colorStride = resWidth * sizeof(uint8_t) * 4;
		uint32_t xyzStride = resWidth * sizeof(int16_t) * 3;

		std::vector<char> bytes;
		bytes.resize(resWidth * resHeight * (7 * 3 + 4 * 3));
		char* current = bytes.data();

		uint8_t* rawData = k4a_image_get_buffer(xyzImg);
		uint8_t* colorData = k4a_image_get_buffer(colorImg);

		for (int y = 0; y < resHeight; y++) {
			for (int x = 0; x < resWidth; x++) {
				int16_t px = *(int16_t*)(rawData + y * xyzStride + x * 6 + 0);
				int16_t py = *(int16_t*)(rawData + y * xyzStride + x * 6 + 2);
				int16_t pz = *(int16_t*)(rawData + y * xyzStride + x * 6 + 4);
				uint8_t pr = *(colorData + y * colorStride + x * 4 + 0);
				uint8_t pg = *(colorData + y * colorStride + x * 4 + 1);
				uint8_t pb = *(colorData + y * colorStride + x * 4 + 2);
				uint8_t pa = *(colorData + y * colorStride + x * 4 + 3);
				if (px != 0 || py != 0 || pz != 0) {
					current = fastCopyInt16Str(current, px);
					*current = ' '; current++;
					current = fastCopyInt16Str(current, py);
					*current = ' '; current++;
					current = fastCopyInt16Str(current, pz);
					*current = ' '; current++;
					current = fastCopyInt16Str(current, pr);
					*current = ' '; current++;
					current = fastCopyInt16Str(current, pg);
					*current = ' '; current++;
					current = fastCopyInt16Str(current, pb);
					*current = ' '; current++;
					current = fastCopyInt16Str(current, pa);
					*current = '\n'; current++;
				}
			}
		}
		FILE* fout = fopen(loc.c_str(), "wb");
		fwrite(bytes.data(), 1, (current - bytes.data()), fout);
		fclose(fout);
	}

#if KINECTCLOUD_EXPERIMENTAL
	struct deviceConfig {
		bool valid;
		uint32_t colorHeight;
		std::string depthMode;
		glm::uvec2 colorRes;
		glm::uvec2 depthRes;
		k4a_calibration_t cali;
		k4a_transformation_t trans;
	};

	// extract json file from 
	deviceConfig getConfig(std::string ffmpeg_path, std::string ffprobe_path, std::string videoPath) {
		deviceConfig res;
		res.valid = true;
		{
			std::string tmpFileLoc = R"(.\tmpinfo.txt)";
			std::string invoke = ffprobe_path; // FFMPEG_DIR + std::string("ffprobe.exe");
			invoke = invoke + " -hide_banner -v quiet -print_format json -show_streams -i " + videoPath + " > " + tmpFileLoc;
			exec(invoke.c_str());
			if (waitUntilFileExists(tmpFileLoc, std::chrono::milliseconds(10 * 1000))) {
				try {
					std::string info = readEntireFile(tmpFileLoc);
					json jinfo = json::parse(info);
					res.colorHeight = jinfo["streams"][0]["height"].get<uint32_t>();
					res.depthMode = jinfo["streams"][1]["tags"]["K4A_DEPTH_MODE"].get<std::string>();
				}
				catch (...) {
					std::cout << "invalid video file\n";
					res.valid = false;
				}
				remove(tmpFileLoc.c_str());
			}
			else {
				std::cout << "extract file info failed\n";
				res.valid = false;
			}
		}
		k4a_color_resolution_t	colorResolution;
		k4a_depth_mode_t		depthMode;
		if (res.valid) {
			colorResolution = colorResolutionFromHeight(res.colorHeight);
			depthMode = depthModeFromString(res.depthMode);
			res.colorRes = colorResFromColorK4a(colorResolution);
			res.depthRes = depthResFromMode(depthMode);
		}
		if (colorResolution == K4A_COLOR_RESOLUTION_OFF && depthMode == K4A_DEPTH_MODE_OFF) {
			std::cout << "video does not have valid color or depth data\n";
			res.valid = false;
		}
		if (res.valid) {
			std::string tmpFileLoc = R"(.\tmpcali.json)";
			std::string invoke = ffmpeg_path;
			invoke = invoke + " -hide_banner -dump_attachment:4 " + tmpFileLoc + " -i " + videoPath;
			exec(invoke.c_str());
			if (waitUntilFileExists(tmpFileLoc, std::chrono::milliseconds(10 * 1000))) {
				std::string info = readEntireFile(tmpFileLoc);
				if (k4a_calibration_get_from_raw(info.data(), info.size() + 1, depthMode, colorResolution, &res.cali) != K4A_RESULT_SUCCEEDED) {
					std::cout << "invalid calibration file\n";
					res.valid = false;
				}
				remove(tmpFileLoc.c_str());
			}
			else {
				std::cout << "extract file info failed\n";
				res.valid = false;
			}
			res.trans = k4a_transformation_create(&res.cali);
		}

		return res;
	}

	// load color image, load depth image, then convert into point cloud and save as pts file
	// colorLoc = input color path
	// depthLoc = input depth path
	// expLoc   = output pts path
	// conf     = config from the device which produced the color and depth images
	void transformImage(std::string colorLoc, std::string depthLoc, std::string expLoc, deviceConfig const& conf) {
		auto colorBytes = loadBitmap(colorLoc);
		if (colorBytes.size() != (conf.colorRes.x * conf.colorRes.y * 4)) colorBytes = addAComponentToRGBColor(colorBytes); // it may be necessary to convert RGB to RBGA
		auto depthBytes = loadBitmap(depthLoc);

		k4a_image_t colorImg = nullptr;
		k4a_image_t depthImg = nullptr;
		k4a_image_t transformedDepthImg = nullptr;
		k4a_image_t xyzImg = nullptr;

		try {
			if (K4A_RESULT_SUCCEEDED != k4a_image_create_from_buffer(
				K4A_IMAGE_FORMAT_COLOR_BGRA32,
				conf.colorRes.x,
				conf.colorRes.y,
				conf.colorRes.x * sizeof(uint8_t) * 4,
				colorBytes.data(),
				colorBytes.size() * sizeof(uint8_t),
				nullptr,
				nullptr,
				&colorImg))
			{
				throw std::runtime_error("failed to create color image from data");
			}

			if (K4A_RESULT_SUCCEEDED != k4a_image_create_from_buffer(
				K4A_IMAGE_FORMAT_DEPTH16,
				conf.depthRes.x,
				conf.depthRes.y,
				conf.depthRes.x * sizeof(int16_t),
				depthBytes.data(),
				depthBytes.size() * sizeof(uint8_t),
				nullptr,
				nullptr,
				&depthImg))
			{
				throw std::runtime_error("failed to create depth image from data");
			}

			if (K4A_RESULT_SUCCEEDED != k4a_image_create(
				K4A_IMAGE_FORMAT_DEPTH16,
				conf.colorRes.x,
				conf.colorRes.y,
				conf.colorRes.x * sizeof(int16_t),
				&transformedDepthImg))
			{
				throw std::runtime_error("failed to create transformed depth image");
			}

			if (K4A_RESULT_SUCCEEDED != k4a_image_create(
				K4A_IMAGE_FORMAT_CUSTOM,
				conf.colorRes.x,
				conf.colorRes.y,
				conf.colorRes.x * sizeof(int16_t) * 3,
				&xyzImg))
			{
				throw std::runtime_error("failed to create xyz image");
			}

			if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera(
				conf.trans,
				depthImg,
				transformedDepthImg
			))
			{
				throw std::runtime_error("failed to transform depth image to color image space");
			}

			if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(
				conf.trans,
				transformedDepthImg,
				K4A_CALIBRATION_TYPE_COLOR,
				xyzImg
			)) {
				throw std::runtime_error("failed to transform mapped depth to point cloud image");
			}

			savePointCloud(conf.colorRes, xyzImg, colorImg, R"(C:\Users\bwysonggrass\Desktop\test.pts)");
		}
		catch (std::runtime_error const& er) {
			std::cout << er.what() << "\n";
		}

		if (colorImg) k4a_image_release(colorImg);
		if (depthImg) k4a_image_release(depthImg);
		if (transformedDepthImg) k4a_image_release(transformedDepthImg);
		if (xyzImg) k4a_image_release(xyzImg);
	}

	// add blank alpha component to vector of rgb pixels
	std::vector<uint8_t> addAComponentToRGBColor(std::vector<uint8_t> const& rgbColorBytes) {
		if (rgbColorBytes.size() % 3 != 0) throw std::runtime_error("color image bytes improperly sized");
		std::vector<uint8_t> res;
		res.resize(rgbColorBytes.size() / 3 * 4);

		size_t resi = 0;
		for (size_t i = 0; i < rgbColorBytes.size(); i += 3) {
			res[resi + 0] = rgbColorBytes[i + 0];
			res[resi + 1] = rgbColorBytes[i + 1];
			res[resi + 2] = rgbColorBytes[i + 2];
			res[resi + 3] = 0;
			resi += 4;
		}

		return res;
	}
#endif
}