//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef COMMON_DELOG_H_
#define COMMON_DELOG_H_
#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"

#ifdef _WIN32
#define __FILENAME__ (strrchr(__FILE__, '\\') ? (strrchr(__FILE__, '\\') + 1):__FILE__)
#else
#define __FILENAME__ (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1):__FILE__)
#endif

#ifndef suffix
#define suffix(msg)  std::string(msg).append("  [")\
        .append(__FILENAME__).append("] [").append(__func__)\
        .append("] [").append(std::to_string(__LINE__))\
        .append("]").c_str()
#endif

#ifndef SPDLOG_TRACE_ON
#define SPDLOG_TRACE_ON
#endif

#ifndef SPDLOG_DEBUG_ON
#define SPDLOG_DEBUG_ON
#endif

extern std::shared_ptr<spdlog::logger> delogger;
extern bool is_save_log_file;

void InitDelog();
void UninitDelog();

#define DELOG_TRACE(msg, ...) delogger->trace(suffix(msg), ##__VA_ARGS__)
#define DELOG_DEBUG(msg, ...) delogger->debug(suffix(msg), ##__VA_ARGS__)
#define DELOG_INFO(msg, ...)  delogger->info(suffix(msg), ##__VA_ARGS__)
#define DELOG_WARN(msg, ...)  delogger->warn(suffix(msg), ##__VA_ARGS__)
#define DELOG_ERROR(msg, ...) delogger->error(suffix(msg), ##__VA_ARGS__)
#define DELOG_FATAL(msg, ...) delogger->critical(suffix(msg), ##__VA_ARGS__)

#endif  // COMMON_DELOG_H_
