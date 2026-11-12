#define PI 3.14159265358f

#if defined __x86_64__ || defined _M_X64
#define __x86
#endif

#if defined _WIN16 || defined WIN32 || defined _WIN32 || defined WIN64 || defined _WIN64 || defined __WIN32__ || defined __TOS_WIN__ || defined __WINDOWS__
	#define BUILD_PLATFORM_WINDOWS
#elif defined __linux__
	#define BUILD_PLATFORM_LINUX
#endif

//To add breakpoints for debugging at runtime
#if defined BUILD_PLATFORM_WINDOWS
	#define add_breakpoint() __debugbreak()
#elif defined BUILD_PLATFORM_LINUX
	#include <signal.h>
	#define add_breakpoint() raise(SIGINT)
#endif