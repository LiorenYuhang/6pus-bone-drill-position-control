#include <gtest/gtest.h>
#include "h30_imu/h30_protocol.hpp"
#include <cstdint>
#include <vector>
namespace {
void i32(std::vector<uint8_t> &v, int32_t x) { const uint32_t u=static_cast<uint32_t>(x); for (int n=0;n<4;++n) v.push_back(static_cast<uint8_t>(u>>(8*n))); }
void u32(std::vector<uint8_t> &v, uint32_t x) { for (int n=0;n<4;++n) v.push_back(static_cast<uint8_t>(x>>(8*n))); }
std::vector<uint8_t> frame(uint16_t tid = 0x1234, bool timestamps = false) {
  std::vector<uint8_t> payload{0x10, 0x0c}; i32(payload, 1000000); i32(payload, -2000000); i32(payload, 9806650);
  payload.push_back(0x20); payload.push_back(0x0c); i32(payload, 180000000); i32(payload, 0); i32(payload, -90000000);
  if (timestamps) {
    payload.push_back(0x40); payload.push_back(0x0c);
    i32(payload, 1000000); i32(payload, -2000000); i32(payload, 3000000);
    payload.push_back(0x41); payload.push_back(0x10);
    i32(payload, 900000); i32(payload, 0); i32(payload, 0); i32(payload, 435890);
    payload.push_back(0x01); payload.push_back(0x02); payload.push_back(0xc4); payload.push_back(0x09);
    payload.push_back(0x51); payload.push_back(0x04); u32(payload, 123456789);
    payload.push_back(0x52); payload.push_back(0x04); u32(payload, 123457001);
  }
  std::vector<uint8_t> f{0x59,0x53,static_cast<uint8_t>(tid),static_cast<uint8_t>(tid >> 8),static_cast<uint8_t>(payload.size())};
  f.insert(f.end(),payload.begin(),payload.end());
  uint8_t a=0,b=0; for(std::size_t n=2;n<f.size();++n){a=static_cast<uint8_t>(a+f[n]);b=static_cast<uint8_t>(b+a);} f.push_back(a);f.push_back(b);return f;
}
}
TEST(H30Protocol, HandlesSplitFrameAndUnits) {
  h30_imu::ProtocolParser p; auto f=frame(); EXPECT_TRUE(p.feed(f.data(),3).empty()); auto out=p.feed(f.data()+3,f.size()-3);
  ASSERT_EQ(out.size(),1u); EXPECT_EQ(out[0].tid,0x1234); EXPECT_DOUBLE_EQ(*out[0].ax,1.0); EXPECT_DOUBLE_EQ(*out[0].ay,-2.0); EXPECT_DOUBLE_EQ(*out[0].gx,180.0);
}
TEST(H30Protocol, RejectsBadChecksumAndResynchronizes) {
  h30_imu::ProtocolParser p; auto bad=frame(); bad.back()^=1; auto good=frame(); bad.insert(bad.end(),good.begin(),good.end()); auto out=p.feed(bad.data(),bad.size()); EXPECT_EQ(out.size(),1u); EXPECT_EQ(p.stats().checksum_errors,1u);
}
TEST(H30Protocol, ParsesDeviceTimestampsAndWrapsTid) {
  h30_imu::ProtocolParser p;
  auto first = frame(0xffff, true);
  auto second = frame(0, true);
  first.insert(first.end(), second.begin(), second.end());
  auto out = p.feed(first.data(), first.size());
  ASSERT_EQ(out.size(), 2u);
  ASSERT_TRUE(out[0].sample_timestamp_us.has_value());
  ASSERT_TRUE(out[0].data_ready_timestamp_us.has_value());
  EXPECT_EQ(*out[0].sample_timestamp_us, 123456789u);
  EXPECT_EQ(*out[0].data_ready_timestamp_us, 123457001u);
  const auto stats = p.stats();
  EXPECT_EQ(stats.valid_frames, 2u);
  EXPECT_EQ(stats.tid_dropped_frames, 0u);
  EXPECT_EQ(stats.tid_wraps, 1u);
}
TEST(H30Protocol, CountsTidGap) {
  h30_imu::ProtocolParser p;
  auto first = frame(10);
  auto second = frame(12);
  first.insert(first.end(), second.begin(), second.end());
  ASSERT_EQ(p.feed(first.data(), first.size()).size(), 2u);
  EXPECT_EQ(p.stats().tid_dropped_frames, 1u);
}

TEST(H30Protocol, HandlesH30TidWrapAt60000) {
  h30_imu::ProtocolParser p;
  auto first = frame(60000);
  auto second = frame(1);
  first.insert(first.end(), second.begin(), second.end());
  ASSERT_EQ(p.feed(first.data(), first.size()).size(), 2u);
  EXPECT_EQ(p.stats().tid_dropped_frames, 0u);
  EXPECT_EQ(p.stats().tid_wraps, 1u);
}
