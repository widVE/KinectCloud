#include <iostream>
#include <memory>
#include <stdexcept>
#include <cstdio>
#include <array>
#include <stdio.h>
#include <set>

#include <k4a/k4a.h>

#include "util.h"
#include "cloud.h"
#include "azureKinectDK.h"

//constexpr auto FFMPEG_DIR = R"(C:\Users\bwysonggrass\Desktop\ffmpeg-20190826-0821bc4-win64-static\bin\)";

#define VALID_RESOLUTIONS "{ 720P, 1080P, 1440P, 1536P, 2160P, 3072P }"
#define VALID_DEPTHS "{ NFOV_2X2BINNED, NFOV_UNBINNED, WFOV_2X2BINNED, WFOV_UNBINNED }"

namespace kinectCloud {
	constexpr k4a_wired_sync_mode_t defaultSyncMode = K4A_WIRED_SYNC_MODE_STANDALONE;
	constexpr k4a_color_resolution_t defaultColorRes = K4A_COLOR_RESOLUTION_3072P;
	constexpr k4a_depth_mode_t defaultDepthMode = K4A_DEPTH_MODE_WFOV_UNBINNED;
	
	bool isSerialNum(std::string const& serial) {
		for (int i = 0; i < serial.length(); i++) {
			if (serial[i] < '0' || serial[i] > '9') return false;
		}
		return true;
	}

	struct deviceInitInfo {
		k4a_wired_sync_mode_t syncMode = defaultSyncMode;
		k4a_color_resolution_t colorRes = defaultColorRes;
		k4a_depth_mode_t depthMode = defaultDepthMode;
	};

	// parameter stuff
	std::string mode = "-n";
	std::string inPath, outPath = "%s_%f.pts";
	std::vector<std::string> specifiedSerialNums;
	std::map<std::string, deviceInitInfo> deviceInfo;
	std::set<std::string> otherOptions;
	int waitMillis = 0;
	int consecutiveCount = 1;
	int colorExposure = 0; // in nanoseconds
	int colorWhiteBalance = 0; // in kelvin
	bool verbose = false;
	k4a_color_resolution_t allResolution = K4A_COLOR_RESOLUTION_OFF;
	k4a_depth_mode_t allDepth = K4A_DEPTH_MODE_OFF;

	// runtime stuff
	std::vector<azureKinectDK> devices;

	// load and start all devices specified by the input parameters, if no specifications open default device only
	void startDevices() {
		if (otherOptions.find("-da") != otherOptions.end()) {
			uint32_t count = azureKinectDK::getNumDevices();
			for (int i = 0; i < count; i++) {
				devices.emplace_back(i);
			}
		} else if (otherOptions.find("-dn") == otherOptions.end()) {
			if (!specifiedSerialNums.empty()) {
				for (std::string const& serial : specifiedSerialNums) {
					devices.emplace_back(serial);
				}
			} else {
				devices.emplace_back(0);
			}
		} else {
			return;
		}
		if (verbose) {
			std::cout << "Opened device(s)";
			for (auto &dev : devices) {
				std::cout << " " << dev.getSerialNum();
			}
			std::cout << "\n";
		}
		for (auto &dev : devices) {
			std::string serial = dev.getSerialNum();
			if (deviceInfo.find(serial) != deviceInfo.end()) {
				auto info = deviceInfo[serial];
				dev.start(info.syncMode, info.colorRes, info.depthMode);
			} else {
				dev.start(defaultSyncMode, (allResolution != K4A_COLOR_RESOLUTION_OFF) ? allResolution : defaultColorRes, defaultDepthMode);
			}
		}
		if (colorExposure) {
			for (auto &dev : devices) {
				dev.setExposure(colorExposure);
			}
		}
		if (colorWhiteBalance) {
			for (auto &dev : devices) {
				dev.setWhiteBalance(colorWhiteBalance);
			}
		}
	}

	// currently not implemented
	// this will extract frames from a video
	int extractMode() {
		return 0;
	}

	// capture point cloud from kinect
	int captureMode() {
		if (waitMillis != 0) std::this_thread::sleep_for(std::chrono::milliseconds(waitMillis));

		int frameNum = 0;
		while (true) {
			for (auto & device : devices) {
				device.captureFrame();
			}
			for (auto & device : devices) {
				device.saveCurrentPointCloud(formatFilePath(outPath, device.getSerialNum(), frameNum));
			}
			std::cout << "frame " << frameNum << "done\n";
			frameNum++;
			if (frameNum >= consecutiveCount) break;
		}

		return 0;
	}

	// parse args
	int main(int argc, char** argv) {
		bool badParams = argc == 1;

		initFastString();

		std::vector<std::string> alerts;

		for (int i = 1; i < argc; i++) {
			if (argv[i] == std::string("-e")) { // extract pointcloud from video
				mode = "-e";
			} else if(argv[i] == std::string("-s")) { // capture and save pointcloud
				mode = "-s";
			} else if(argv[i] == std::string("-i")) { // input path
				if (++i != argc) {
					inPath = argv[i];
				} else badParams = true;
			} else if(argv[i] == std::string("-o")) { // output path (default %s_%f.pts)
				if (++i != argc) {
					outPath = argv[i];
				} else badParams = true;
			} else if(argv[i] == std::string("-w")) { // wait after startup
				if (++i != argc) {
					waitMillis = std::atoi(argv[i]);
				} else {
					alerts.push_back("Error: -w must be followed by integer");
					badParams = true;
				}
				if (waitMillis < 0) waitMillis = 0;
			} else if(argv[i] == std::string("-c")) { // consecutive frames
				if (++i != argc) {
					consecutiveCount = std::atoi(argv[i]);
				} else {
					alerts.push_back("Error: -c must be followed by integer");
					badParams = true;
				}
				if (consecutiveCount < 1) consecutiveCount = 1;
			} else if(argv[i] == std::string("-ds")) { // device serial number to open
				if (++i != argc) {
					auto str = std::string(argv[i]);
					if(!isSerialNum(argv[i])) alerts.push_back("Warning: -ds serial number should probably just be digits, instead is " + std::string(argv[i]));
					specifiedSerialNums.push_back(argv[i]);
					deviceInfo[argv[i]].syncMode = defaultSyncMode;
				} else {
					alerts.push_back("Error: -ds must be followed by serial number");
					badParams = true;
				}
			} else if(argv[i] == std::string("-dt")) { // topology / authority mode
				std::string serialNum;
				if (++i != argc) {
					if (!isSerialNum(argv[i])) alerts.push_back("Warning: -dt serial number should probably just be digits, instead is " + std::string(argv[i]));
					serialNum = argv[i];
				} else {
					alerts.push_back("Error: -dt must be followed by integer");
					badParams = true;
				}
				if (++i != argc) {
					if (argv[i] == std::string("a")) {
						deviceInfo[serialNum].syncMode = K4A_WIRED_SYNC_MODE_STANDALONE;
					} else if (argv[i] == std::string("m")) {
						deviceInfo[serialNum].syncMode = K4A_WIRED_SYNC_MODE_MASTER;
					} else if (argv[i] == std::string("s")) {
						deviceInfo[serialNum].syncMode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
					} else {
						alerts.push_back("Error: -dt must be followed by serial number and a, m, or s");
						badParams = true;
					}
				} else {
					alerts.push_back("Error: -dt must be followed by serial number and a, m, or s");
					badParams = true;
				}
			} else if(argv[i] == std::string("-dr")) { // color resolution
				std::string serialNum;
				if (++i != argc) {
					if (!isSerialNum(argv[i])) alerts.push_back("Warning: -dr serial number should probably just be digits, instead is " + std::string(argv[i]));
					serialNum = argv[i];
					
					if (++i != argc) {
						k4a_color_resolution_t res = colorResolutionFromString(argv[i]);
						if (res != K4A_COLOR_RESOLUTION_OFF) {
							deviceInfo[serialNum].colorRes = res;
						} else {
							alerts.push_back("Error: -dr resolution must be one of " VALID_RESOLUTIONS);
							badParams = true;
						}
					} else {
						alerts.push_back("Error: -dr must be followed by serial number and resolution");
						badParams = true;
					}
				} else {
					alerts.push_back("Error: -dr must be followed by serial number and resolution");
					badParams = true;
				}
			} else if(argv[i] == std::string("-dra")) { // color resolution all
				if (++i != argc) {
					k4a_color_resolution_t res = colorResolutionFromString(argv[i]);
					if (res != K4A_COLOR_RESOLUTION_OFF) {
						allResolution = res;
					} else {
						alerts.push_back("Error: -dra resolution must be one of " VALID_RESOLUTIONS);
						badParams = true;
					}
				} else {
					alerts.push_back("Error: -dra must be followed by resolution");
					badParams = true;
				}
			} else if(argv[i] == std::string("-dm")) { // depth mode
				std::string serialNum;
				if (++i != argc) {
					if (!isSerialNum(argv[i])) alerts.push_back("Warning: -dm serial number should probably just be digits, instead is " + std::string(argv[i]));
					serialNum = argv[i];
					
					if (++i != argc) {
						k4a_depth_mode_t mode = depthModeFromString(argv[i]);
						if (mode != K4A_DEPTH_MODE_OFF) {
							deviceInfo[serialNum].depthMode = mode;
						} else {
							alerts.push_back("Error: -dm mode must be one of " VALID_DEPTHS);
							badParams = true;
						}
					} else {
						alerts.push_back("Error: -dm must be followed by serial number and depth mode");
						badParams = true;
					}
				} else {
					alerts.push_back("Error: -dm must be followed by serial number and depth mode");
					badParams = true;
				}
			} else if(argv[i] == std::string("-dma")) { // depth mode all
				if (++i != argc) {
					k4a_depth_mode_t mode = depthModeFromString(argv[i]);
					if (mode != K4A_DEPTH_MODE_OFF) {
						allDepth = mode;
					} else {
						alerts.push_back("Error: -dma mode must be one of " VALID_DEPTHS);
						badParams = true;
					}
				} else {
					alerts.push_back("Error: -dma must be followed by depth mode");
					badParams = true;
				}
			} else if(argv[i] == std::string("-ce")) { // color exposure
				if (++i != argc) {
					colorExposure = std::atoi(argv[i]);
				} else badParams = true;
			} else if(argv[i] == std::string("-cw")) { // color white balance
				if (++i != argc) {
					colorWhiteBalance = std::atoi(argv[i]);
				} else badParams = true;
			} else if(argv[i] == std::string("-v")) {
				verbose = true;
			} else {
				otherOptions.insert(argv[i]);
			}
		}

		if (badParams) {
			for (int i = 0; i < alerts.size(); i++) {
				std::cout << alerts[i] << "\n";
			}
			if (true) {
				std::cout << "options:\n";
				//std::cout << " -e              | extract colored frames into pts files\n";
				std::cout << " -s              | capture a colored pointcloud for devices (by default first device only)\n";
				//std::cout << " -i file         | specify input file (if applicable)\n";
				std::cout << " -o path         | specify output locations (if applicable)\n";
				std::cout << "                 | %s -> serial number, %f -> frame number\n";
				std::cout << " -w int          | wait a number of milliseconds after device startup (if applicable)\n";
				std::cout << " -c int          | store n consecutive frames (if applicable)\n";
				std::cout << " -ds ser         | device operations apply device with given serial number\n";
				std::cout << " -da             | device operations apply to all devices\n";
				std::cout << " -dn             | if no other devices are specified, do not perform any device operations\n";
				std::cout << " -dt ser {a,m,s} | declare device topology, stAndalone, Master or Subordinate\n";
				std::cout << " -dr ser {res}   | declare resolution to use for specific device\n";
				std::cout << " -dra {res}      | declare resolution to use for all devices\n";
				std::cout << "   color resolutions: " VALID_RESOLUTIONS ", default is 3072P\n";
				std::cout << " -ce int         | color camera exposure time in nanoseconds for all devices\n";
				std::cout << " -cw int         | color camera white balance in kelvin for all devices (must be % by 10)\n";
				std::cout << " -v              | verbose output\n";
			}
			return 0;
		}
	
		if (mode == "-e") {
			extractMode();
		} else if(mode == "-s") {
			startDevices();
			captureMode();
		}

		return 0;

		/*auto conf = getConfig(FFMPEG_DIR + std::string("ffmpeg.exe"), FFMPEG_DIR + std::string("ffprobe.exe"), R"(C:\Users\bwysonggrass\Desktop\test.mkv)");

		transformImage(R"(C:\Users\bwysonggrass\Desktop\color0001.bmp)", R"(C:\Users\bwysonggrass\Desktop\depth0001.bmp)", R"(C:\Users\bwysonggrass\Desktop\out0001.bmp)", conf);

		if (false) {
			if (!filesInFolder(".\\processing").empty()) {
				std::cout << "processing folder needs to be empty\n";
				return 1;
			}

			for (auto const& p : filesInFolder("./")) {
				if (contains(p, "REGISTRATION_MATRIX")) {
					remove(p.c_str());
				}
			}

			merge("C:\\Program Files\\CloudCompare\\CloudCompare.exe", "2048x1536_0.pts", "2048x1536_1.pts", ".\\processing\\tmp1.bin");
			waitUntilFileExists(".\\processing\\tmp1.bin", std::chrono::milliseconds(300 * 1000));

			for (int i = 2; i < 100; i++) {
				std::string registrationMatLoc;
				for (auto const& p : filesInFolder("./")) {
					if (contains(p, "REGISTRATION_MATRIX")) {
						registrationMatLoc = p;
						break;
					}
				}

				std::string basePath = ".\\processing\\tmp" + std::to_string(i - 1) + ".bin";
				std::string addPath = "2048x1536_" + std::to_string(i) + ".pts";
				std::string outPath = ".\\processing\\tmp" + std::to_string(i) + ".bin";

				merge("C:\\Program Files\\CloudCompare\\CloudCompare.exe", basePath, addPath, outPath, registrationMatLoc);
				waitUntilFileExists(outPath, std::chrono::milliseconds(300 * 1000));

				std::this_thread::sleep_for(std::chrono::milliseconds(250));

				remove(registrationMatLoc.c_str());

				std::this_thread::sleep_for(std::chrono::milliseconds(250));
			}
		} else {
			auto r = loadBitmap("C:\\Users\\bwysonggrass\\Desktop\\ffmpeg-20190826-0821bc4-win64-static\\bin\\frame0001.bmp");

		
		}

		return 0;*/
	}
}

int main(int argc, char **argv) {
	try {
		return kinectCloud::main(argc, argv);
	}
	catch (std::runtime_error const& er) {
		std::cout << "Runtime Exception: " << er.what() << "\n";
	}
}