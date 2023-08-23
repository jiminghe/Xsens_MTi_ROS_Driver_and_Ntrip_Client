//  Copyright (c) 2003-2023 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  
// MIT License
//
// Copyright (c) 2021 Yuming Meng
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "ntrip_util.h"

#include <math.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <string>
#include <fstream>
#include <memory>


namespace libntrip {

namespace {

double DegreeConvertToDDMM(double const& degree) {
  int deg = static_cast<int>(floor(degree));
  double minute = degree - deg*1.0;
  return (deg*1.0 + minute*60.0/100.0);
}

}  // namespace

//
// Ntrip util.
//

int BccCheckSumCompareForGGA(const char *src) {
  int sum = 0;
  int num = 0;
  sscanf(src, "%*[^*]*%x", &num);
  for (int i = 1; src[i] != '*'; ++i) {
    sum ^= src[i];
  }
  return sum - num;
}

std::string kBase64CodeTable =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

inline
int base64_index(char in) {
  return kBase64CodeTable.find(in);
}

int Base64Encode(std::string const& raw, std::string* out) {
  int len = raw.size();
  for (int i = 0; i < len; i += 3) {
    out->push_back(kBase64CodeTable[(raw[i]&0xFC)>>2]);
    if (i+1 >= len) {
      out->push_back(kBase64CodeTable[(raw[i]&0x03)<<4]);
      break;
    }
    out->push_back(kBase64CodeTable[(raw[i]&0x03)<<4|(raw[i+1]&0xF0)>>4]);
    if (i+2 >= len) {
      out->push_back(kBase64CodeTable[(raw[i+1]&0x0F)<<2]);
      break;
    } else {
      out->push_back(kBase64CodeTable[(raw[i+1]&0x0F)<<2|(raw[i+2]&0xC0)>>6]);
    }
    out->push_back(kBase64CodeTable[raw[i+2]&0x3F]);
  }
  len = out->size();
  if (len % 4 != 0) {
    out->append(std::string(4-len%4, '='));
  }
  return 0;
}

int Base64Decode(std::string const& raw, std::string* out) {
  if (out == nullptr) return -1;
  int len = raw.size();
  if ((len == 0) || (len%4 != 0)) return -1;
  out->clear();
  for (int i = 0; i < len; i += 4) {
    out->push_back(((base64_index(raw[i])&0x3F)<<2) |
        ((base64_index(raw[i+1])&0x3F)>>4));
    if (raw[i+2] == '=') {
      out->push_back(((base64_index(raw[i+1])&0x0F)<<4));
      break;
    }
    out->push_back(((base64_index(raw[i+1])&0x0F)<<4) |
        ((base64_index(raw[i+2])&0x3F)>>2));
    if (raw[i+3] == '=') {
      out->push_back(((base64_index(raw[i+2])&0x03)<<6));
      break;
    }
    out->push_back(((base64_index(raw[i+2])&0x03)<<6) |
        (base64_index(raw[i+3])&0x3F));
  }
  return 0;
}

// int GGAFrameGenerate(double latitude, double longitude,
//     double altitude, std::string* gga_out) {
//   if (gga_out == nullptr) return -1;
//   char src[256] = {0};
//   time_t t = time(nullptr);
//   struct tm *tt = localtime(&t);
//   double timestamp[3];
//   timestamp[0] = tt->tm_hour >= 8 ? tt->tm_hour - 8 : tt->tm_hour + 24 - 8;
//   timestamp[1] = tt->tm_min;
//   timestamp[2] = tt->tm_sec;
//   char *ptr = src;
//   ptr += snprintf(ptr, sizeof(src)+src-ptr,
//       "$GPGGA,%02.0f%02.0f%05.2f,%012.7f,%s,%013.7f,%s,2,"
//       "10,1.2,%.4f,M,-2.860,M,,0000",
//       timestamp[0], timestamp[1], timestamp[2],
//       fabs(DegreeConvertToDDMM(latitude))*100.0,
//       latitude > 0.0 ? "N" : "S",
//       fabs(DegreeConvertToDDMM(longitude))*100.0,
//       longitude > 0.0 ? "E" : "W",
//       altitude);
//   uint8_t checksum = 0;
//   for (char *q = src + 1; q <= ptr; q++) {
//     checksum ^= *q; // check sum.
//   }
//   ptr += snprintf(ptr, sizeof(src)+src-ptr, "*%02X%c%c", checksum, 0x0D, 0x0A);
//   *gga_out = std::string(src, ptr-src);
//   return BccCheckSumCompareForGGA(gga_out->c_str());
// }




int generateGGA(const XsRawGnssPvtData& pvtData, std::string* gga_out) {
  if (gga_out == nullptr) return -1;
  char src[256] = {0};
  char *ptr = src;
  // add the Time (hhmmss.ss format), Latitude (ddmm.mmmmmmmm format), Latitude direction (N/S), 
  // Longitude (dddmm.mmmmmmmm format),Longitude direction (E/W),GPS quality indicator, Number of satellites,Horizontal dilution of precision
  // Altitude above mean sea level (meters) from pvtData, members of pvtData inlcude: m_hour, m_min, m_sec, m_nano, m_lat, m_lon, m_fixType, m_numSv, m_hdop, m_hMsl
  ptr += snprintf(ptr, sizeof(src)+src-ptr,
      "$GPGGA,%02d%02d%05.2f,%012.7f,%s,%013.7f,%s,%01d,"
      "%02d,%.1f,%.2f,M,0.000,M,,0000",
      pvtData.m_hour, pvtData.m_min, pvtData.m_sec+pvtData.m_nano*1e-9,
      fabs(DegreeConvertToDDMM(pvtData.m_lat* 1e-7))*100.0,
      pvtData.m_lat > 0.0 ? "N" : "S",
      fabs(DegreeConvertToDDMM(pvtData.m_lon* 1e-7))*100.0,
      pvtData.m_lon > 0.0 ? "E" : "W",
      pvtData.m_fixType,
      pvtData.m_numSv,
      static_cast<double>(pvtData.m_hdop/100.0),
      static_cast<double>(pvtData.m_hMsl)/1000.0); // cast to double and convert to meters

  uint8_t checksum = 0;
  for (char *q = src + 1; q <= ptr; q++) {
    checksum ^= *q; // check sum.
  }
  ptr += snprintf(ptr, sizeof(src)+src-ptr, "*%02X%c%c", checksum, 0x0D, 0x0A);
  *gga_out = std::string(src, ptr-src);
  return BccCheckSumCompareForGGA(gga_out->c_str());
}






}  // namespace libntrip
