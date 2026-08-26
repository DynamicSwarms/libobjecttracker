#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "libobjecttracker/object_tracker.h"
#include "libobjecttracker/conversions.hpp"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <type_traits>

#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_2D.h>

// point cloud log format:
// infinite repetitions of:
// timestamp (milliseconds) : uint32
// cloud size               : uint32
// [x y z, x y z, ... ]     : float32

#define markermax 60*4 

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using ICP = pcl::IterativeClosestPoint<Point, Point>;

namespace libobjecttracker {

	class PointCloudLogger
	{
	public:
		PointCloudLogger(std::string file_path) : file(file_path, std::ios::binary | std::ios::out)
		{
		}

		void log(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
		{
			auto stamp = std::chrono::steady_clock::now();
			if (start == (decltype(start)())) {
				start = stamp;
			}
			auto millis = std::chrono::duration_cast<std::chrono::milliseconds>
				(stamp - start).count();
			log(millis, cloud);
		}

		void log(uint32_t millis, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
		{
			write<uint32_t>(file, millis);
			write<uint32_t>(file, cloud->size());
			for (pcl::PointXYZ const &p : *cloud) {
				static_assert(std::is_same<decltype(p.x), float>::value, "expected float");
				write(file, p.x);
				write(file, p.y);
				write(file, p.z);
			}
		}

		void flush()
		{
			file.flush();
		}

	protected:
		template <typename T>
		void write(std::ofstream &s, T const &t)
		{
			s.write((char const *)&t, sizeof(T));
		}
		std::ofstream file;
			std::chrono::steady_clock::time_point start;
	};

	class PointCloudPlayer
	{
	public:
		void load(std::string path)
		{
			std::ifstream s(path, std::ios::binary | std::ios::in);
			if (!s) {
				throw std::runtime_error("PointCloudPlayer: bad file path.");
			}
			while (s) {
				uint32_t millis = read<uint32_t>(s);
				// TODO cleaner loop?
				if (!s) {
					break;
				}
				timestamps.push_back(millis);

				uint32_t size = read<uint32_t>(s);
				clouds.emplace_back(new pcl::PointCloud<pcl::PointXYZ>());
				clouds.back()->resize(size);
				for (uint32_t i = 0; i < size; ++i) {
					float x = read<float>(s);
					float y = read<float>(s);
					float z = read<float>(s);
					(*clouds.back())[i] = pcl::PointXYZ(x, y, z);
				}
			}
		}

		void play(libobjecttracker::ObjectTracker &tracker) const
		{
			uint32_t previous_millis = 0;
			for (size_t i = 0; i < clouds.size(); ++i) {
				std::cout << "\n  " << i << "  ------------------------------\n";
				const auto dt = std::chrono::milliseconds(
					timestamps[i] - previous_millis);
				tracker.update(dt, clouds[i]);
				previous_millis = timestamps[i];
			}
		}

	protected:
		template <typename T>
		T read(std::ifstream &s)
		{
			T t;
			s.read((char *)&t, sizeof(t));
			return t;
		}
		std::vector<uint32_t> timestamps;
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
	};

	class PointCloudDebugger : public PointCloudPlayer
	{

	public:
		PointCloudDebugger(std::string file_path) : writepath(file_path) {}

		void convert(libobjecttracker::ObjectTracker & tracker, std::vector<MarkerConfiguration> & config)
		{
			//play points
			uint32_t previous_millis = 0;
			for (size_t i = 0; i < clouds.size(); ++i) {
				std::cout << "\n  " << i << "  ------------------------------\n";
				const auto dt = std::chrono::milliseconds(
					timestamps[i] - previous_millis);
				tracker.update(dt, clouds[i]);
				previous_millis = timestamps[i];
				//continue;
				//make another output cloud
				matches.emplace_back(new pcl::PointCloud<pcl::PointXYZ>());
				matches.back()->reserve(markermax);
				//read updated objects
				const auto objects = tracker.objects();
				int a = 0;
				for (const auto* object : objects) {
					++a;
					std::cout << "Object vector size: " << objects.size() << "\n";
					std::cout << "Object " << a << " processing\n";
					//debugging stuff
					Cloud::Ptr &objMarkers = config[object->m_marker_configuration_idx];
					size_t const objNpts = objMarkers->size();
					for (size_t j = 0; j < objNpts; ++j) { //for each marker
						auto p = object->transformation() * to_eigen((*objMarkers)[j]); //get real position
						matches.back()->push_back(to_pcl(p));
					}
				}
			}
			std::cout << "Writing converted file\n";
			std::ofstream s(writepath, std::ios::binary | std::ios::out);
			for (size_t i = 0; i < matches.size(); ++i) {
				write(s, timestamps[i]);
				write(s, (uint32_t)matches[i]->size());
				for (pcl::PointXYZ const &p : *(matches[i])) {
					static_assert(std::is_same<decltype(p.x), float>::value, "expected float");
					write(s, p.x);
					write(s, p.y);
					write(s, p.z);
				}
			}
		}

	private:
		template <typename T>
		void write(std::ofstream &s, T const &t)
		{
			s.write((char const *)&t, sizeof(T));
		}
		std::string writepath;
		std::string path;
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> matches;
	};

} //namespace libobjecttracker
