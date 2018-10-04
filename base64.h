#ifndef MARKER_VISION_BASE64_H
#define MARKER_VISION_BASE64_H
#include <string>

std::string base64_encode(unsigned char const* , unsigned int len);
std::string base64_decode(std::string const& s);

#endif //MARKER_VISION_BASE64_H
