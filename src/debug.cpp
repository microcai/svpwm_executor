
#include "debug.hpp"
#include "SEGGER_RTT.h"
#include <cstdarg>
#include <cstdio>
#include <stdio.h>

void debug_print(const char* fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt); // Initialize
    char buf[128];
    auto len = vsnprintf(buf, 128, fmt, arg_list);
    
    SEGGER_RTT_Write(0, buf, len);
    va_end(arg_list);              // Cleanup
}
