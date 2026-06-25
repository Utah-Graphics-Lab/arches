#pragma once

// Shared command-line configuration for Arches and Native Kernel Executables (trax).
// Both parse same --key=value argumets and derive the same scene/camera setup
// Host-only do not include in the RISC-V kernel builds

#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

#include "rtm/rtm.hpp"

namespace Arches {

const static std::vector<std::string> arch_names = {"TRaX", "STRaTA", "STRaTA-RT", "Dual-Streaming", "RIC"};

struct SceneConfig
{
	std::string name;
	rtm::vec3 cam_pos;
	rtm::vec3 cam_target;
	float focal_length;
};

const static std::vector<SceneConfig> scene_configs =
{
	{"cornell-box", rtm::vec3(0, 0.8, 1.8), rtm::vec3(0, 0.8, 0), 12.0f},
	{"sibenik", rtm::vec3(3.0, -13.0, 0.0), rtm::vec3(0, -12.0, 0), 12.0f},
	{"crytek-sponza", rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f), 12.0f},
	{"bistro-interior", rtm::vec3(-0.813307f, 2.0811f, -1.28115f), rtm::vec3(0.186693f, 2.0811f, -1.28115f), 24.0f},
	{"intel-sponza", rtm::vec3(-900.6f, 150.8f, 120.74f), rtm::vec3(79.7f, 14.0f, -17.4f), 12.0f},
	{"sponza", rtm::vec3(0.0f, 2.0f, 0.0f), rtm::vec3(90.0f, 0.0f, -1.0f), 12.0f},
	{"san-miguel", rtm::vec3(7.448, 1.014, 12.357), rtm::vec3(8.056, 1.04, 11.563), 12.0f},
	{"hairball", rtm::vec3(0, 0, 10), rtm::vec3(0, 0, 0), 24.0f},
	{"bistro", rtm::vec3(-8.0, 2.0, 2.0), rtm::vec3(0.0f, 1.0f, -1.0f), 12.0f},
};

class SimulationConfig
{
public:
	struct Param
	{
		enum Type { INT, FLOAT, STRING } type;
		union { int i; float f; };
		std::string s;

		Param() {}
	};

	rtm::Camera camera;

private:
	std::map<std::string, Param> _params;

	static std::string default_dataset_dir()
	{
#ifdef ARCHES_DEFAULT_DATASET_DIR
		return ARCHES_DEFAULT_DATASET_DIR;
#else
		return "datasets";
#endif
	}

public:
	// Defaults common to every Arches executable. 
    // Application-specific params (e.g. the simulator's config-dir/kernel-path) are 
    // registered by the caller with set_param() before parse().
	SimulationConfig()
	{
		set_param("logging-interval", 10000);
		set_param("arch-name", "TRaX");
		set_param("max-rays", 128);
		set_param("dataset-dir", default_dataset_dir());
		set_param("scene-name", "sponza");
		set_param("framebuffer-width", 1024);
		set_param("framebuffer-height", 1024);
		set_param("pregen-rays", 0);
		set_param("pregen-bounce", 1);
		set_param("bvh-preset", 0);
		set_param("bvh-merging", 0);
	}

	// Parse --key=value args, resolve arch-id/scene-id, build the camera, and print.
	void parse(int argc, char* argv[])
	{
		for(int i = 1; i < argc; ++i)
		{
			std::string arg(argv[i]);
			size_t split_pos = arg.find('=');
			if(arg.rfind("--", 0) != 0 || split_pos == std::string::npos)
			{
				printf("Error: malformed argument '%s' (expected --key=value)\n", arg.c_str());
				exit(1);
			}
			parse_param(arg.substr(2, split_pos - 2), arg.substr(split_pos + 1));
		}

		set_param("arch-id", index_of(arch_names, get_string("arch-name"), "arch-name"));

		int scene_id = -1;
		for(int i = 0; i < (int)scene_configs.size(); ++i)
			if(scene_configs[i].name == get_string("scene-name"))
				scene_id = i;
		if(scene_id == -1)
		{
			printf("Error: unknown scene-name '%s'\n", get_string("scene-name").c_str());
			exit(1);
		}
		set_param("scene-id", scene_id);

		camera = rtm::Camera(get_int("framebuffer-width"), get_int("framebuffer-height"),
			scene_configs[scene_id].focal_length, scene_configs[scene_id].cam_pos, scene_configs[scene_id].cam_target);

		print();
	}

	int get_int(const std::string& key) const { return find(key, Param::INT)->second.i; }
	float get_float(const std::string& key) const { return find(key, Param::FLOAT)->second.f; }
	std::string get_string(const std::string& key) const { return find(key, Param::STRING)->second.s; }

	void set_param(const std::string& key, int value) { _params[key].type = Param::INT; _params[key].i = value; }
	void set_param(const std::string& key, float value) { _params[key].type = Param::FLOAT; _params[key].f = value; }
	void set_param(const std::string& key, const std::string& value) { _params[key].type = Param::STRING; _params[key].s = value; }

	void parse_param(const std::string& key, const std::string& str)
	{
		auto a = _params.find(key);
		if(a == _params.end())
		{
			printf("Error: unknown parameter '--%s'\n", key.c_str());
			exit(1);
		}

		try
		{
			if(a->second.type == Param::INT)        a->second.i = std::stoi(str);
			else if(a->second.type == Param::FLOAT) a->second.f = std::stof(str);
			else                                    a->second.s = str;
		}
		catch(const std::exception&)
		{
			printf("Error: invalid value '%s' for parameter '--%s'\n", str.c_str(), key.c_str());
			exit(1);
		}
	}

	void print() const
	{
		printf("Simulation Parameters\n");
		for(const auto& a : _params)
		{
			if(a.second.type == Param::INT)        printf("%s: %d\n", a.first.c_str(), a.second.i);
			else if(a.second.type == Param::FLOAT) printf("%s: %f\n", a.first.c_str(), a.second.f);
			else                                   printf("%s: %s\n", a.first.c_str(), a.second.s.c_str());
		}
		printf("\n");
	}

private:
	std::map<std::string, Param>::const_iterator find(const std::string& key, Param::Type type) const
	{
		auto a = _params.find(key);
		if(a == _params.end() || a->second.type != type)
		{
			printf("Error: missing or wrong-typed parameter '%s'\n", key.c_str());
			exit(1);
		}
		return a;
	}

	static int index_of(const std::vector<std::string>& names, const std::string& value, const char* what)
	{
		for(int i = 0; i < (int)names.size(); ++i)
			if(names[i] == value)
				return i;
		printf("Error: unknown %s '%s'\n", what, value.c_str());
		exit(1);
	}
};

}
