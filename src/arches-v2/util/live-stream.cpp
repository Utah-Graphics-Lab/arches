// Winsock must be included before anything pulls in windows.h (TBB does, via stdafx.hpp)
#ifdef _WIN32
	#ifndef WIN32_LEAN_AND_MEAN
		#define WIN32_LEAN_AND_MEAN
	#endif
	#ifndef NOMINMAX
		#define NOMINMAX
	#endif
	#include <winsock2.h>
	#include <ws2tcpip.h>
	typedef SOCKET socket_t;
	#define ARCHES_INVALID_SOCKET INVALID_SOCKET
	#define ARCHES_CLOSE_SOCKET closesocket
#else
	#include <arpa/inet.h>
	#include <fcntl.h>
	#include <netinet/in.h>
	#include <netinet/tcp.h>
	#include <sys/select.h>
	#include <sys/socket.h>
	#include <unistd.h>
	typedef int socket_t;
	#define ARCHES_INVALID_SOCKET (-1)
	#define ARCHES_CLOSE_SOCKET ::close
#endif

#include "live-stream.hpp"

#include <cmath>
#include <cstdio>

namespace Arches {

static void set_blocking(socket_t s, bool blocking)
{
#ifdef BUILD_PLATFORM_WINDOWS
	u_long mode = blocking ? 0 : 1;
	ioctlsocket(s, FIONBIO, &mode);
#else
	int flags = fcntl(s, F_GETFL, 0);
	fcntl(s, F_SETFL, blocking ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK));
#endif
}

LiveStream::LiveStream(const std::string& host, uint16_t port)
{
	if(port == 0) return;

#ifdef BUILD_PLATFORM_WINDOWS
	WSADATA wsa_data;
	if(WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) return;
#endif

	socket_t s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if(s == ARCHES_INVALID_SOCKET) return;

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	if(inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1)
	{
		printf("Live stream: invalid host '%s'\n", host.c_str());
		ARCHES_CLOSE_SOCKET(s);
		return;
	}

	// Non-blocking connect with a short timeout. A refused connection on Windows
	// otherwise takes ~2 seconds because of SYN retries.
	set_blocking(s, false);
	connect(s, (sockaddr*)&addr, sizeof(addr));

	fd_set write_set, error_set;
	FD_ZERO(&write_set); FD_SET(s, &write_set);
	FD_ZERO(&error_set); FD_SET(s, &error_set);
	timeval timeout{0, 250000};
	bool ok = select((int)s + 1, nullptr, &write_set, &error_set, &timeout) > 0 && FD_ISSET(s, &write_set) && !FD_ISSET(s, &error_set);
	if(ok)
	{
		int error = 0;
		socklen_t len = sizeof(error);
		ok = getsockopt(s, SOL_SOCKET, SO_ERROR, (char*)&error, &len) == 0 && error == 0;
	}

	if(!ok)
	{
		printf("Live stream: no visualizer at %s:%u, continuing without it\n", host.c_str(), port);
		ARCHES_CLOSE_SOCKET(s);
		return;
	}

	set_blocking(s, true);
	int no_delay = 1;
	setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char*)&no_delay, sizeof(no_delay));

	printf("Live stream: connected to %s:%u\n", host.c_str(), port);
	_socket = (uint64_t)s;
	_connected = true;
	_worker = std::thread(&LiveStream::_worker_loop, this);
}

LiveStream::~LiveStream()
{
	finish();
}

void LiveStream::send(MessageType type, const std::string& payload)
{
	send(type, (const uint8_t*)payload.data(), payload.size());
}

void LiveStream::send(MessageType type, const uint8_t* payload, size_t size)
{
	if(!_connected) return;

	std::vector<uint8_t> message(5 + size);
	uint32_t length = (uint32_t)(size + 1);
	std::memcpy(message.data(), &length, 4);
	message[4] = (uint8_t)type;
	if(size) std::memcpy(message.data() + 5, payload, size);

	{
		std::lock_guard<std::mutex> lock(_mutex);
		if(type == MessageType::FRAME && _queued_bytes + message.size() > MAX_QUEUED_BYTES)
			return;

		_queued_bytes += message.size();
		_queue.push_back(std::move(message));
	}
	_cv.notify_one();
}

void LiveStream::send_frame(uint64_t cycle, uint32_t width, uint32_t height, const uint8_t* rgba)
{
	if(!_connected) return;

	size_t pixel_bytes = (size_t)width * height * 4;
	std::vector<uint8_t> payload(16 + pixel_bytes);
	std::memcpy(payload.data() + 0, &cycle, 8);
	std::memcpy(payload.data() + 8, &width, 4);
	std::memcpy(payload.data() + 12, &height, 4);
	std::memcpy(payload.data() + 16, rgba, pixel_bytes);
	send(MessageType::FRAME, payload.data(), payload.size());
}

void LiveStream::finish()
{
	{
		std::lock_guard<std::mutex> lock(_mutex);
		_stopping = true;
	}
	_cv.notify_one();

	if(_worker.joinable())
		_worker.join();

	_close();
}

void LiveStream::_worker_loop()
{
	while(true)
	{
		std::vector<uint8_t> message;
		{
			std::unique_lock<std::mutex> lock(_mutex);
			_cv.wait(lock, [&] { return _stopping || !_queue.empty(); });
			if(_queue.empty()) return; //stopping and drained

			message = std::move(_queue.front());
			_queue.pop_front();
			_queued_bytes -= message.size();
		}

		if(!_send_all(message.data(), message.size()))
		{
			printf("Live stream: visualizer disconnected\n");
			_connected = false;
			std::lock_guard<std::mutex> lock(_mutex);
			_queue.clear();
			_queued_bytes = 0;
			return;
		}
	}
}

bool LiveStream::_send_all(const uint8_t* data, size_t size)
{
	socket_t s = (socket_t)_socket;
	while(size > 0)
	{
		int chunk = (int)std::min<size_t>(size, 1 << 20);
		int sent = ::send(s, (const char*)data, chunk, 0);
		if(sent <= 0) return false;
		data += sent;
		size -= sent;
	}
	return true;
}

void LiveStream::_close()
{
	if(_socket != ~0ull)
	{
		ARCHES_CLOSE_SOCKET((socket_t)_socket);
		_socket = ~0ull;
#ifdef BUILD_PLATFORM_WINDOWS
		WSACleanup();
#endif
	}
	_connected = false;
}

void Topology::group(const std::string& id, const std::string& label, uint count, const Props& props, const std::string& parent, const std::string& stats)
{
	JsonWriter json;
	json.begin_object()
		.value("id", id)
		.value("label", label)
		.value("count", count)
		.value("parent", parent)
		.value("stats", stats)
		.begin_array("props");
	for(auto& prop : props)
		json.begin_array().value(nullptr, prop.first).value(nullptr, prop.second).end_array();
	json.end_array().end_object();
	_groups.push_back(json.str());
}

void Topology::link(const std::string& from, const std::string& to, const std::string& map, const std::string& traffic)
{
	JsonWriter json;
	json.begin_object()
		.value("from", from)
		.value("to", to)
		.value("map", map)
		.value("traffic", traffic)
		.end_object();
	_links.push_back(json.str());
}

std::string Topology::json() const
{
	std::string out = "{\"groups\":[";
	for(size_t i = 0; i < _groups.size(); ++i)
		out += (i ? "," : "") + _groups[i];
	out += "],\"links\":[";
	for(size_t i = 0; i < _links.size(); ++i)
		out += (i ? "," : "") + _links[i];
	return out + "]}";
}

std::string Topology::bytes(uint64_t size)
{
	const char* units[] = {"B", "KB", "MB", "GB", "TB"};
	uint unit = 0;
	while(size >= 1024 && size % 1024 == 0 && unit < 4)
	{
		size /= 1024;
		++unit;
	}
	return std::to_string(size) + " " + units[unit];
}

void JsonWriter::_key(const char* key)
{
	if(!_first) _out += ',';
	_first = false;
	if(key)
	{
		_out += '"';
		_out += key;
		_out += "\":";
	}
}

JsonWriter& JsonWriter::value(const char* key, double v)
{
	_key(key);
	if(!std::isfinite(v))
	{
		_out += "null";
	}
	else
	{
		char buffer[32];
		snprintf(buffer, sizeof(buffer), "%.9g", v);
		_out += buffer;
	}
	return *this;
}

JsonWriter& JsonWriter::value(const char* key, const std::string& v)
{
	_key(key);
	_out += '"';
	for(char c : v)
	{
		switch(c)
		{
		case '"':  _out += "\\\""; break;
		case '\\': _out += "\\\\"; break;
		case '\n': _out += "\\n"; break;
		case '\r': _out += "\\r"; break;
		case '\t': _out += "\\t"; break;
		default:
			if((unsigned char)c < 0x20)
			{
				char buffer[8];
				snprintf(buffer, sizeof(buffer), "\\u%04x", c);
				_out += buffer;
			}
			else _out += c;
		}
	}
	_out += '"';
	return *this;
}

}
