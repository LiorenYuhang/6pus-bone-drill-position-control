#include <gtest/gtest.h>
#include "h30_imu/h30_protocol.hpp"
#include <cstdint>
#include <vector>
namespace {
void i32(std::vector<uint8_t> &v, int32_t x) { const uint32_t u=static_cast<uint32_t>(x); for (int n=0;n<4;++n) v.push_back(static_cast<uint8_t>(u>>(8*n))); }
void u32(std::vector<uint8_t> &v, uint32_t x) { for (int n=0;n<4;++n) v.push_back(static_cast<uint8_t>(x>>(8*n))); }
std::vector<uint8_t> frame() {
  std::vector<uint8_t> payload{0x10, 0x0c}; i32(payload, 1000000); i32(payload, -2000000); i32(payload, 9806650);
  payload.push_back(0x20); payload.push_back(0x0c); i32(payload, 180000000); i32(payload, 0); i32(payload, -90000000);
  payload.push_back(0x51); payload.push_back(0x04); u32(payload, 123456789);
  payload.push_back(0x52); payload.push_back(0x04); u32(payload, 123457001);
  std::vector<uint8_t> f{0x59,0x53,0x34,0x12,static_cast<uint8_t>(payload.size())}; f.insert(f.end(),payload.begin(),payload.end());
  uint8_t a=0,b=0; for(std::size_t n=2;n<f.size();++n){a=static_cast<uint8_t>(a+f[n]);b=static_cast<uint8_t>(b+a);} f.push_back(a);f.push_back(b);return f;
}
void set_tid(std::vector<uint8_t> &f, uint16_t tid) {
  f[2] = static_cast<uint8_t>(tid & 0xffu);
  f[3] = static_cast<uint8_t>((tid >> 8) & 0xffu);
  uint8_t a=0,b=0; for(std::size_t n=2;n+2<f.size();++n){a=static_cast<uint8_t>(a+f[n]);b=static_cast<uint8_t>(b+a);}
  f[f.size() - 2] = a;
  f[f.size() - 1] = b;
}

void set_sample_timestamp(std::vector<uint8_t> &f, uint32_t sample_us) {
  const std::size_t payload_size = f[4];
  std::size_t pos = 5;
  const std::size_t end = pos + payload_size;
  while (pos + 2 <= end) {
    const uint8_t id = f[pos];
    const std::size_t len = f[pos + 1];
    pos += 2;
    if (id == 0x51 && len == 4) {
      for (int n = 0; n < 4; ++n) f[pos + n] = static_cast<uint8_t>(sample_us >> (8 * n));
      break;
    }
    pos += len;
  }
  uint8_t a=0,b=0; for(std::size_t n=2;n+2<f.size();++n){a=static_cast<uint8_t>(a+f[n]);b=static_cast<uint8_t>(b+a);}
  f[f.size() - 2] = a;
  f[f.size() - 1] = b;
}
}
TEST(H30Protocol, HandlesSplitFrameAndUnits) {
  h30_imu::ProtocolParser p; auto f=frame(); EXPECT_TRUE(p.feed(f.data(),3).empty()); auto out=p.feed(f.data()+3,f.size()-3);
  ASSERT_EQ(out.size(),1u); EXPECT_EQ(out[0].tid,0x1234); EXPECT_DOUBLE_EQ(*out[0].ax,1.0); EXPECT_DOUBLE_EQ(*out[0].ay,-2.0); EXPECT_DOUBLE_EQ(*out[0].gx,180.0);
  ASSERT_TRUE(out[0].sample_timestamp_us.has_value());
  ASSERT_TRUE(out[0].data_ready_timestamp_us.has_value());
  EXPECT_EQ(*out[0].sample_timestamp_us, 123456789u);
  EXPECT_EQ(*out[0].data_ready_timestamp_us, 123457001u);
  EXPECT_LT(*out[0].sample_timestamp_us, *out[0].data_ready_timestamp_us);
  const auto stats = p.stats();
  EXPECT_EQ(stats.valid_frames, 1u);
  EXPECT_EQ(stats.checksum_errors, 0u);
  EXPECT_EQ(stats.malformed_tlv_errors, 0u);
  EXPECT_EQ(stats.tid_dropped_frames, 0u);
  EXPECT_EQ(stats.true_tid_missing_frames, 0u);
  EXPECT_EQ(stats.sample_timestamp_count, 1u);
  EXPECT_EQ(stats.sample_timestamp_missing_samples, 0u);
  EXPECT_EQ(stats.data_ready_timestamp_count, 1u);
  EXPECT_EQ(stats.tid_duplicates, 0u);
  EXPECT_EQ(stats.tid_out_of_order, 0u);
  EXPECT_TRUE(stats.have_last_tid);
  EXPECT_EQ(stats.last_tid, 0x1234);
}
TEST(H30Protocol, RejectsBadChecksumAndResynchronizes) {
  h30_imu::ProtocolParser p; auto bad=frame(); bad.back()^=1; auto good=frame(); bad.insert(bad.end(),good.begin(),good.end()); auto out=p.feed(bad.data(),bad.size()); EXPECT_EQ(out.size(),1u); EXPECT_EQ(p.checksum_errors(),1u);
  const auto stats = p.stats();
  EXPECT_EQ(stats.valid_frames, 1u);
  EXPECT_EQ(stats.checksum_errors, 1u);
}

TEST(H30Protocol, CountsTidDropsDuplicatesAndOutOfOrder) {
  h30_imu::ProtocolParser p;
  auto f1 = frame();
  set_tid(f1, 10u);
  auto f2 = frame();
  set_tid(f2, 13u);
  auto f3 = frame();
  set_tid(f3, 13u);
  auto f4 = frame();
  set_tid(f4, 12u);

  std::vector<uint8_t> bytes;
  bytes.insert(bytes.end(), f1.begin(), f1.end());
  bytes.insert(bytes.end(), f2.begin(), f2.end());
  bytes.insert(bytes.end(), f3.begin(), f3.end());
  bytes.insert(bytes.end(), f4.begin(), f4.end());

  const auto out = p.feed(bytes.data(), bytes.size());
  ASSERT_EQ(out.size(), 4u);
  const auto stats = p.stats();
  EXPECT_EQ(stats.valid_frames, 4u);
  EXPECT_EQ(stats.tid_dropped_frames, 2u);
  EXPECT_EQ(stats.true_tid_missing_frames, 2u);
  EXPECT_EQ(stats.tid_duplicates, 1u);
  EXPECT_EQ(stats.tid_out_of_order, 1u);
}

TEST(H30Protocol, CountsH30TidWrap) {
  h30_imu::ProtocolParser p;
  auto before = frame();
  set_tid(before, 60000u);
  auto after = frame();
  set_tid(after, 1u);

  std::vector<uint8_t> bytes;
  bytes.insert(bytes.end(), before.begin(), before.end());
  bytes.insert(bytes.end(), after.begin(), after.end());

  const auto out = p.feed(bytes.data(), bytes.size());
  ASSERT_EQ(out.size(), 2u);
  const auto stats = p.stats();
  EXPECT_EQ(stats.valid_frames, 2u);
  EXPECT_EQ(stats.tid_wraps, 1u);
  EXPECT_EQ(stats.tid_dropped_frames, 0u);
}


TEST(H30Protocol, H30TidWrapWithOneLostWrapFrameCountsOneMissingNotHugeDrop) {
  h30_imu::ProtocolParser p;
  auto before = frame();
  set_tid(before, 59999u);
  auto after = frame();
  set_tid(after, 1u);

  std::vector<uint8_t> bytes;
  bytes.insert(bytes.end(), before.begin(), before.end());
  bytes.insert(bytes.end(), after.begin(), after.end());

  const auto out = p.feed(bytes.data(), bytes.size());
  ASSERT_EQ(out.size(), 2u);
  const auto stats = p.stats();
  EXPECT_EQ(stats.tid_wraps, 1u);
  EXPECT_EQ(stats.true_tid_missing_frames, 1u);
  EXPECT_EQ(stats.tid_dropped_frames, 1u);
  EXPECT_EQ(stats.tid_resets, 0u);
}

TEST(H30Protocol, CountsSampleTimestampContinuityAndResetSeparately) {
  h30_imu::ProtocolParser p;
  auto f1 = frame();
  set_tid(f1, 100u);
  set_sample_timestamp(f1, 100000u);
  auto f2 = frame();
  set_tid(f2, 101u);
  set_sample_timestamp(f2, 110000u);
  auto f3 = frame();
  set_tid(f3, 102u);
  set_sample_timestamp(f3, 109000u);

  std::vector<uint8_t> bytes;
  bytes.insert(bytes.end(), f1.begin(), f1.end());
  bytes.insert(bytes.end(), f2.begin(), f2.end());
  bytes.insert(bytes.end(), f3.begin(), f3.end());

  const auto out = p.feed(bytes.data(), bytes.size());
  ASSERT_EQ(out.size(), 3u);
  const auto stats = p.stats();
  EXPECT_EQ(stats.true_tid_missing_frames, 0u);
  EXPECT_EQ(stats.sample_timestamp_count, 3u);
  EXPECT_EQ(stats.sample_timestamp_missing_samples, 1u);
  EXPECT_EQ(stats.sample_timestamp_resets, 1u);
  EXPECT_EQ(stats.sample_timestamp_out_of_order, 1u);
}
