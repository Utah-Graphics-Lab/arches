#pragma once
#include "stdafx.hpp"

#include <condition_variable>

namespace Arches {

// Streams simulation data to an external visualizer (e.g. RusticArches) over TCP.
//
// Wire format, all integers little-endian:
//   [u32 length][u8 type][payload (length - 1 bytes)]
// Types:
//   HELLO (1) JSON: static configuration, sent once before the simulation starts
//   STATS (2) JSON: raw counters for one logging interval
//   FRAME (3) binary: u64 cycle, u32 width, u32 height, width * height RGBA8 pixels (bottom row first)
//   DONE  (4) JSON: end of run summary
//
// If no visualizer is listening, the stream disables itself and every call is a no-op.
// Sending happens on a background thread so the simulator never blocks on the socket.
// When the visualizer falls behind, FRAME messages are dropped before anything else.
class LiveStream
{
public:
	enum class MessageType : uint8_t
	{
		HELLO = 1,
		STATS = 2,
		FRAME = 3,
		DONE = 4,
	};

	LiveStream(const std::string& host, uint16_t port);
	~LiveStream();

	LiveStream(const LiveStream&) = delete;
	LiveStream& operator=(const LiveStream&) = delete;

	bool connected() const { return _connected; }

	void send(MessageType type, const std::string& payload);
	void send(MessageType type, const uint8_t* payload, size_t size);
	void send_frame(uint64_t cycle, uint32_t width, uint32_t height, const uint8_t* rgba);

	// Blocks until everything queued has been sent, then closes the connection.
	void finish();

private:
	static constexpr size_t MAX_QUEUED_BYTES = 64ull << 20;

	std::atomic_bool _connected{false};
	uint64_t _socket{~0ull};

	std::thread _worker;
	std::mutex _mutex;
	std::condition_variable _cv;
	std::deque<std::vector<uint8_t>> _queue;
	size_t _queued_bytes{0};
	bool _stopping{false};

	void _worker_loop();
	bool _send_all(const uint8_t* data, size_t size);
	void _close();
};

// Describes the simulated architecture for the visualizer's block diagram: groups of identical
// units and the links between them. Record each group right where its units are constructed so
// the diagram always matches what was actually built.
class Topology
{
public:
	typedef std::vector<std::pair<std::string, std::string>> Props;

	// `count` is the total number of units. With a `parent`, they are split evenly among the
	// parent's units. `stats` names the per-unit counters in STATS ("l1d", "tp", ...), if any.
	void group(const std::string& id, const std::string& label, uint count, const Props& props = {}, const std::string& parent = "", const std::string& stats = "");

	// How units of `from` connect to units of `to`:
	//   "one"   unit i to unit i
	//   "group" consecutive blocks of from-units share one to-unit (block size = from count / to count)
	//   "all"   every unit to every unit (a crossbar)
	//   "local" units within the same parent unit
	// `traffic` names the per-unit counters whose bytes_read measure the data crossing the link.
	void link(const std::string& from, const std::string& to, const std::string& map, const std::string& traffic = "");

	std::string json() const;

	static std::string bytes(uint64_t size);

private:
	std::vector<std::string> _groups, _links;
};

// Minimal JSON object writer, enough for flat and nested objects of numbers, strings and arrays.
class JsonWriter
{
public:
	JsonWriter& begin_object(const char* key = nullptr) { _key(key); _out += '{'; _first = true; return *this; }
	JsonWriter& end_object() { _out += '}'; _first = false; return *this; }
	JsonWriter& begin_array(const char* key = nullptr) { _key(key); _out += '['; _first = true; return *this; }
	JsonWriter& end_array() { _out += ']'; _first = false; return *this; }

	JsonWriter& value(const char* key, uint64_t v) { _key(key); _out += std::to_string(v); return *this; }
	JsonWriter& value(const char* key, int64_t v) { _key(key); _out += std::to_string(v); return *this; }
	JsonWriter& value(const char* key, uint32_t v) { return value(key, (uint64_t)v); }
	JsonWriter& value(const char* key, int32_t v) { return value(key, (int64_t)v); }
	JsonWriter& value(const char* key, double v);
	JsonWriter& value(const char* key, const std::string& v);
	JsonWriter& value(const char* key, const char* v) { return value(key, std::string(v)); }

	JsonWriter& counters(const char* key, const uint64_t* v, size_t n)
	{
		begin_array(key);
		for(size_t i = 0; i < n; ++i) value(nullptr, v[i]);
		return end_array();
	}

	JsonWriter& counters(const char* key, const std::vector<uint64_t>& v) { return counters(key, v.data(), v.size()); }

	// Inserts an already serialized JSON value.
	JsonWriter& raw(const char* key, const std::string& json) { _key(key); _out += json; return *this; }

	const std::string& str() const { return _out; }

private:
	std::string _out;
	bool _first{true};

	void _key(const char* key);
};

}
