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

#include "delog.h"

std::shared_ptr<spdlog::logger> delogger = NULL;
bool save_delog_file = false;

void InitDelog() {

  if (spdlog::get("console") != nullptr) {
    delogger = spdlog::get("console");
    return;
  }

  std::vector<spdlog::sink_ptr> sinkList;
  auto consoleSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  consoleSink->set_level(spdlog::level::debug);
  sinkList.push_back(consoleSink);

  if (save_delog_file) {
    auto rotateSink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("delog.txt", 1024 * 1024 * 5, 4);
    rotateSink->set_level(spdlog::level::debug);
    sinkList.push_back(rotateSink);
  }

  delogger = std::make_shared<spdlog::logger>("console", begin(sinkList), end(sinkList));
  spdlog::register_logger(delogger);
  delogger->set_level(spdlog::level::debug);
  delogger->flush_on(spdlog::level::debug);
}

void UninitDeLog() {
  spdlog::drop_all();
}