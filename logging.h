//
// Created by arnoud on 23-2-19.
//

#ifndef MARKER_VISION_LOGGING_H
#define MARKER_VISION_LOGGING_H


#include "spdlog/spdlog.h"


extern std::shared_ptr<spdlog::logger> vpp_logger;
extern std::shared_ptr<spdlog::logger> pp_logger;
extern std::shared_ptr<spdlog::logger> v_logger;




//spdlog::logger combined_logger; // = std::make_shared<spdlog::logger>("V_PP_c", begin(sinks), end(sinks));
//spdlog::logger vpp_logger; // = std::make_shared<spdlog::logger>("V_PP", vpp_sink);






#endif //MARKER_VISION_LOGGING_H
