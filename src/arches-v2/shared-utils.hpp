#pragma once
#include "stdafx.hpp"

#include <filesystem>

#include "simulator/simulator.hpp"

#include "units/unit-dram.hpp"
#include "units/unit-cache.hpp"
#include "units/unit-crossbar.hpp"
#include "units/unit-buffer.hpp"
#include "units/unit-atomic-reg-file.hpp"
#include "units/unit-tile-scheduler.hpp"
#include "units/unit-sfu.hpp"
#include "units/unit-tp.hpp"

#include "util/elf.hpp"
#include "isa/riscv.hpp"
#include "rtm/rtm.hpp"
#include "sim-config.hpp"

#if defined BUILD_PLATFORM_WINDOWS
	#include <Windows.h>
	#define MAX_FILENAME_LENGTH MAX_PATH
#elif defined BUILD_PLATFORM_LINUX
	#include <linux/limits.h>
	#define MAX_FILENAME_LENGTH FILENAME_MAX
#endif

char full_exe_name[MAX_FILENAME_LENGTH];
namespace Arches {

void set_full_exe_name(const char *name) {
	strcpy(full_exe_name, name);
}

static std::filesystem::path get_executable_path()
{
	std::filesystem::path executable_path(full_exe_name);
	if(executable_path.empty())
		return std::filesystem::current_path();

	if(executable_path.is_relative())
		executable_path = std::filesystem::absolute(executable_path);

	return executable_path.lexically_normal();
}

static std::filesystem::path get_executable_folder()
{
	std::filesystem::path executable_path = get_executable_path();
	if(std::filesystem::is_directory(executable_path))
		return executable_path;

	return executable_path.parent_path();
}

static std::string normalize_path_string(const std::filesystem::path& path)
{
	return path.lexically_normal().generic_string();
}

static std::string ensure_trailing_slash(std::string path)
{
	for(char& c : path)
	{
		if(c == '\\')
			c = '/';
	}

	while(path.size() > 1 && path.back() == '/')
		path.pop_back();

	if(!path.empty())
		path += "/";

	return path;
}

// CMake stages config-files/ and kernels/ next to the executable, so prefer those, otherwise use the source-tree paths baked in at configure time.
std::string get_default_config_dir()
{
	std::filesystem::path runtime_config_dir = get_executable_folder() / "config-files";
	if(std::filesystem::exists(runtime_config_dir))
		return normalize_path_string(runtime_config_dir);

	return normalize_path_string(std::filesystem::path(ARCHES_DEFAULT_CONFIG_DIR));
}

std::string get_default_kernel_path()
{
	std::filesystem::path runtime_kernel_path = get_executable_folder() / "kernels" / "trax" / "kernel";
	if(std::filesystem::exists(runtime_kernel_path))
		return normalize_path_string(runtime_kernel_path);

	return normalize_path_string(std::filesystem::path(ARCHES_DEFAULT_KERNEL_PATH));
}

std::string join_path(std::string dir, const std::string& name)
{
	return ensure_trailing_slash(dir) + name;
}

template <typename T>
static T* write_array(Units::UnitMainMemoryBase* main_memory, size_t alignment, const T* data, size_t size, paddr_t& heap_address)
{
	paddr_t array_address = align_to(alignment, heap_address);
	heap_address = array_address + size * sizeof(T);
	main_memory->direct_write(data, size * sizeof(T), array_address);
	return reinterpret_cast<T*>(array_address);
}

template <typename T>
static T* write_vector(Units::UnitMainMemoryBase* main_memory, size_t alignment, const std::vector<T>& v, paddr_t& heap_address)
{
	return write_array(main_memory, alignment, v.data(), v.size(), heap_address);
}

template <typename T>
static T* write_array(uint8_t* main_memory, size_t alignment, const T* data, size_t size, paddr_t& heap_address)
{
	paddr_t array_address = align_to(alignment, heap_address);
	heap_address = array_address + size * sizeof(T);
	memcpy(main_memory + array_address, data, size * sizeof(T));
	return reinterpret_cast<T*>(array_address);
}

template <typename T>
static T* write_vector(uint8_t* main_memory, size_t alignment, const std::vector<T>& v, paddr_t& heap_address)
{
	return write_array(main_memory, alignment, v.data(), v.size(), heap_address);
}

template <typename T>
static T* write_array(Units::UnitMainMemoryBase** drams,  const Units::UnitCrossbar& xbar, size_t alignment, const T* data, size_t size, paddr_t& heap_address)
{
	paddr_t array_address = align_to(alignment, heap_address);
	heap_address = array_address + size * sizeof(T);
	for(uint i = 0; i < size * sizeof(T); ++i)
		drams[xbar.get_partition(array_address + i)]->direct_write((uint8_t*)data + i, 1, xbar.strip_partition_bits(array_address + i));
	return (T*)(array_address);
}

template <typename T>
static T* write_vector(Units::UnitMainMemoryBase** drams, const Units::UnitCrossbar& xbar, size_t alignment, const std::vector<T>& v, paddr_t& heap_address)
{
	return write_array(drams, xbar, alignment, v.data(), v.size(), heap_address);
}

template <class T, class L>
inline static L delta_log(L& master_log, std::vector<T*> units)
{
	L delta_log;
	for(uint i = 0; i < units.size(); ++i)
	{
		delta_log.accumulate(units[i]->log);
		units[i]->log.reset();
	}
	master_log.accumulate(delta_log);
	return delta_log;
}

template <class T, class L>
inline static L delta_log(L& master_log, T& unit)
{
	L delta_log = unit.log;
	master_log.accumulate(unit.log);
	unit.log.reset();
	return delta_log;
}

void print_header(std::string string, uint header_length = 80)
{
	uint spacers = string.length() < header_length ? header_length - string.length() : 0;
	printf("\n");
	for(uint i = 0; i < spacers / 2; ++i)
		printf("-");
	printf("%s", string.c_str());
	for(uint i = 0; i < (spacers + 1) / 2; ++i)
		printf("-");
	printf("\n");
}


}
