#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <thread>

#include <glm/ext.hpp>
#include <lodepng.h>

namespace kinectCloud {

	// convert all uppercase chars into lowercase chars
	std::string stringToUppercase(std::string const& str) {
		std::string res;
		res.reserve(str.size());
		std::locale loc;
		for (int i = 0; i < str.length(); i++) {
			res.push_back(std::toupper(str[i], loc));
		}
		return res;
	}

	// copy file to string and return it
	std::string readEntireFile(std::string const& path) {
		std::ifstream ifs(path);
		return std::string(
			std::istreambuf_iterator<char>(ifs),
			std::istreambuf_iterator<char>()
		);
	}

	std::vector<uint8_t> readEntireFileBinary(std::string const& path) {
		std::ifstream ifs(path, std::ios::binary);
		return std::vector<uint8_t>(
			std::istreambuf_iterator<char>(ifs),
			std::istreambuf_iterator<char>()
		);
	}

	// split string based on symbol
	std::vector<std::string> split(const std::string& str, const std::string& delim) {
		std::vector<std::string> tokens;
		size_t prev = 0, pos = 0;
		do
		{
			pos = str.find(delim, prev);
			if (pos == std::string::npos) pos = str.length();
			std::string token = str.substr(prev, pos - prev);
			if (!token.empty()) tokens.push_back(token);
			prev = pos + delim.length();
		} while (pos < str.length() && prev < str.length());
		return tokens;
	}

	// if string contains symbol
	bool contains(std::string const& str, std::string const& sym) {
		return str.find(sym) != std::string::npos;
	}

	glm::mat4 readMat4(std::string path) {
		glm::mat4 res;
		std::ifstream ifs(path);
		for (int i = 0; i < 4; i++) {
			glm::vec4 vec;
			ifs >> vec.x >> vec.y >> vec.z >> vec.w;
			res[i] = vec;
		}
		return res;
	}

	// in base, replace %s with serial and %f with frameNum
	// only the first instance of %s, %f, is used
	// substitution only occurs if %s or %f are found
	std::string formatFilePath(std::string base, std::string serial, std::string frameNum) {
		size_t sf = base.find("%s");
		if (sf != std::string::npos) {
			base = base.substr(0, sf) + serial + base.substr(sf + 2);
		}

		size_t ff = base.find("%f");
		if (ff != std::string::npos) {
			base = base.substr(0, ff) + frameNum + base.substr(ff + 2);
		}

		return base;
	}

	char*	data;
	char*	begins[65536];
	int		sizes[65536];

	// create list of all int16s
	void initFastString() {
		data = (char*)malloc(500 * 1024 * 1024);
		begins[0] = data;
		for (int i = 0; i <= ((int)std::numeric_limits<int16_t>::max() - (int)std::numeric_limits<int16_t>::min()); ++i) {
			sprintf(begins[i], "%hd", i + (int)std::numeric_limits<int16_t>::min());
			sizes[i] = strlen(begins[i]);
			if (i != ((int)std::numeric_limits<int16_t>::max() - (int)std::numeric_limits<int16_t>::min())) begins[i + 1] = begins[i] + sizes[i];
		}
	}
	
	// similar to sprintf(start, "%d", val)
	// but may be slightly faster due to caching all int16s
	char* fastCopyInt16Str(char* start, int16_t val) {
		memcpy(start, begins[val - (int)std::numeric_limits<int16_t>::min()], sizes[val - (int)std::numeric_limits<int16_t>::min()]);
		return start + sizes[val - (int)std::numeric_limits<int16_t>::min()];
	}

#ifdef KINECTCLOUD_EXPERIMENTAL
	// execute command synchronously
	std::string exec(const char* cmd) {
		char buffer[128];
		std::string result = "";
#ifdef __linux__ 
		FILE* pipe = popen(cmd, "r");
#elif _WIN32
		FILE* pipe = _popen(cmd, "r");
#else
		FILE* pipe = NULL;
#endif
		if (!pipe) throw std::runtime_error("popen() failed!");
		try {
			while (fgets(buffer, sizeof buffer, pipe) != NULL) {
				result += buffer;
			}
		}
		catch (...) {
			_pclose(pipe);
			throw;
		}
		_pclose(pipe);
		return result;
	}

	// get list of all files in folder
	std::vector<std::string> filesInFolder(std::string folder) {
		if (!std::filesystem::is_directory(folder)) return { };
		std::vector<std::string> res;
		for (const auto& entry : std::filesystem::directory_iterator(folder)) {
			res.push_back(entry.path().string());
		}
		return res;
	}

	//wait until file exists, or until timeout occurs
	//returns true if file exists
	//return false if timeout occurred
	bool waitUntilFileExists(std::string const& file, std::chrono::milliseconds maxWait) {
		std::cout << "waiting on " << file << "\n";
		auto start = std::chrono::high_resolution_clock::now();
		while (!std::filesystem::exists(file)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(250));
			if (std::chrono::high_resolution_clock::now() - start > maxWait) {
				std::cout << "wait timed out\n";
				return false;
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(250));
		std::cout << "wait succeeded\n";
		return true;
	}

	// load bitmap file
	std::vector<uint8_t> loadBitmap(std::string const& file) {
		int i;
		FILE* f = fopen(file.c_str(), "rb");
		unsigned char info[54];
		fread(info, sizeof(unsigned char), 54, f); // read the 54-byte header

		// extract image height and width from header
		int width =		*(int*)&info[18];
		int height =	*(int*)&info[22];

		int size = 3 * width * height;
		std::vector<uint8_t> res;
		res.resize(size);
		fread(res.data(), sizeof(unsigned char), size, f); // read the rest of the data at once
		fclose(f);

		for (i = 0; i < size; i += 3)
		{
			unsigned char tmp = res.data()[i];
			res.data()[i] = res.data()[i + 2];
			res.data()[i + 2] = tmp;
		}
		return res;
	}

	std::vector<uint8_t> loadPng(std::string const& file, uint32_t& width, uint32_t& height, int& componentSize, bool& hasAlpha, bool& isRGB) {
		std::vector<uint8_t> input = readEntireFileBinary(file);
		std::vector<uint8_t> res;
		lodepng::State s;
		s.decoder.color_convert = false;
		lodepng::decode(res, width, height, s, input);
		componentSize = s.info_png.color.bitdepth;
		hasAlpha =
			(s.info_raw.colortype == LCT_GREY_ALPHA || s.info_raw.colortype == LCT_RGBA);
		isRGB =
			(s.info_raw.colortype == LCT_RGB || s.info_raw.colortype == LCT_RGBA);
		return res;
	}

	int numFramesInMKV(std::string const& ffprobePath, std::string const& mkvPath) {
		auto res = exec((ffprobePath + R"( -v error -count_frames -select_streams v:0 -show_entries stream=nb_read_frames -of default=nokey=1:noprint_wrappers=1 )" + mkvPath).c_str());
		return std::stoi(res);
	}
#endif
}
