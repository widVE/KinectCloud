#pragma once

#include <vector>
#include <mutex>
#include <thread>
#include <chrono>

#include "azureKinectDK.h"
#include "k4arecord/record.h"
#include "httplib.h"

namespace kinectCloud {
	// host an HTTP server which returns a blob representing the point cloud
	class azureKinectServer {
		httplib::Server *_server;

		azureKinectDK* _dev;

		struct frame {
			void* data;
			uint64_t dataSize;
			int frameNum;
		};

		std::vector<frame> frames;

		int curFrame = 0;

		int maxFrames;

		std::mutex framesMut;

		std::atomic_bool shouldClose;
	public:
		// open recording with device
		inline azureKinectServer(azureKinectDK* dev, int cacheFrames) : _dev(dev) {
			maxFrames = cacheFrames;

			std::thread t([this]() {
				_server = new httplib::Server();

				_server->Get("/status", [this](httplib::Request const& req, httplib::Response& res) {
					json j = {
						{"cache frames", maxFrames},
					};
					res.set_content(j.dump(4), "application/json");
				});

				_server->Get("/frames", [this](httplib::Request const& req, httplib::Response& res) {
					std::string str;

					{
						std::lock_guard<std::mutex> lock(framesMut);
						for (int i = 0; i < frames.size(); i++) {
							str += std::to_string(frames[i].frameNum) + "\n";
						}
					}

					res.set_content(str, "text/plain");
				});

				_server->Get(R"(/frame/(\d+))", [this](httplib::Request const& req, httplib::Response& res) {
					auto m = req.matches[1].str();
	
					int frameNum = atoi(m.c_str());
					std::lock_guard<std::mutex> lock(framesMut);

					int index = -1;
					for (int i = 0; i < frames.size(); i++) if (frames[i].frameNum == frameNum) { index = i; break; }
					if (index == -1) return;

					res.set_content((char*)frames[index].data, frames[index].dataSize, "application/octet-stream");
				});

				_server->Get("/frame/latest", [this](httplib::Request const& req, httplib::Response& res) {
					std::lock_guard<std::mutex> lock(framesMut);

					int index = frames.empty() ? -1 : (frames.size() - 1);
					if (index == -1) return;

					res.set_content((char*)frames[index].data, frames[index].dataSize, "application/octet-stream");
				});

				_server->Get("/close", [this](httplib::Request const& req, httplib::Response& res) {
					shouldClose = true;
					_server->stop();
				});

				_server->listen("localhost", 5687);
			});

			shouldClose = false;
			while (!shouldClose) {
				_dev->captureFrame();
				k4a_capture_t cap = _dev->getCurrCapture();
				if (cap) {
					uint8_t* rawMem = new uint8_t[1024 * 1024 * 128];
					((uint64_t*)rawMem)[0] = _dev->saveCurrentPointCloudRaw(rawMem + sizeof(uint64_t));
					
					frame f;
					f.data = rawMem;
					f.dataSize = ((uint64_t*)rawMem)[0] * 9 + 8;
					f.frameNum = curFrame;
					curFrame++;

					{
						std::lock_guard<std::mutex> lock(framesMut);

						if (frames.size() >= maxFrames) {
							delete[] frames.begin()->data;
							frames.erase(frames.begin());
						}

						frames.push_back(f);
					}
				}
			}

			t.join();
		}

		// copy constructor removed
		inline azureKinectServer(azureKinectServer const& other) = delete;

		// copy assignment removed
		inline azureKinectServer& operator=(azureKinectServer const& other) = delete;

		// move constructor (needed for use in std::vector)
		inline azureKinectServer(azureKinectServer&& other) noexcept {
			move(other);
		}

		// move assignment (needed for use in std::vector)
		inline azureKinectServer& operator=(azureKinectServer&& other) noexcept {
			move(other);
		}

		// destructor
		inline ~azureKinectServer() {
			if (_server) delete _server;
		}

	private:

		// copy values from other to this, then clear values from other
		inline void move(azureKinectServer& other) {
			throw std::runtime_error("move not supported for azureKinectServer");
			_server = other._server;
			_dev = other._dev;

			other._server = nullptr;
			other._dev = nullptr;
		}
	};
}