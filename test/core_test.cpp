// SPDX-License-Identifier: MIT
#include <cassert>
#include <vector>

#include "leptrino_force_torque_ros_driver/protocol.hpp"
#include "leptrino_force_torque_ros_driver/publication.hpp"

using namespace leptrino_force_torque_ros_driver;

int main()
{
  const uint8_t stop[] = {4, 255, 0x33, 0};
  std::array<uint8_t, max_wire> wire;
  const auto size = encode(stop, sizeof(stop), wire);
  const uint8_t expected[] = {16, 2, 4, 255, 0x33, 0, 16, 3, 0xcb};
  assert(size == sizeof(expected) && std::memcmp(wire.data(), expected, size) == 0);

  std::vector<uint8_t> body(20, 0);
  body[0] = 20;
  body[1] = 255;
  body[2] = 0x32;
  body[4] = 16;
  body[19] = 0xff;  // v1.31 reserved byte is unspecified.
  const auto length = encode(body.data(), body.size(), wire);
  for (std::size_t split = 0; split <= length; ++split) {
    Parser parser;
    int count = 0;
    const auto cb =
      [&](Event event, const uint8_t * data, std::size_t n, const ReceiveStamp & stamp) {
        assert(event == Event::frame && n == body.size());
        assert(std::memcmp(data, body.data(), n) == 0);
        assert(stamp.ros_nanoseconds == (split == length ? 1 : 2));
        ++count;
      };

    ReceiveStamp first;
    ReceiveStamp last;
    first.ros_nanoseconds = 1;
    last.ros_nanoseconds = 2;
    parser.feed(wire.data(), split, cb, first);
    parser.feed(wire.data() + split, length - split, cb, last);
    assert(count == 1);
  }

  Parser parser;
  int frames = 0;
  int naks = 0;
  const auto cb = [&](Event event, const uint8_t *, std::size_t, const ReceiveStamp &) {
    if (event == Event::frame) {
      ++frames;
    } else {
      ++naks;
    }
  };

  auto damaged = wire;
  damaged[length - 1] ^= 1;
  parser.feed(damaged.data(), length, cb);

  std::vector<uint8_t> oversized(300, 1);
  oversized[0] = 16;
  oversized[1] = 2;
  parser.feed(oversized.data(), oversized.size(), cb);

  for (int i = 0; i < 2; ++i) {
    parser.feed(wire.data(), length, cb);
  }

  const uint8_t nak[] = {16, 0x15};
  parser.feed(nak, sizeof(nak), cb);
  assert(frames == 2 && naks == 1);
  assert(validate(body.data(), body.size(), 0x32) == Response::measurement);
  body[0] = 19;
  assert(validate(body.data(), body.size(), 0x32) == Response::invalid);

  std::vector<uint8_t> info{38, 255, 0x2a, 0};
  const std::string fields = "PFS055YA251U6S  2606008 40091200  ";
  info.insert(info.end(), fields.begin(), fields.end());
  ProductInfo product;
  assert(decodeProduct(info.data(), info.size(), product));
  assert(
    product.protocol == Protocol::v131 && product.serial == "2606008" &&
    product.output_rate == "1200");

  info[20] = 'x';
  assert(!decodeProduct(info.data(), info.size(), product));

  info[20] = '2';

  info.resize(32);
  info[0] = 32;
  assert(decodeProduct(info.data(), info.size(), product));
  assert(product.protocol == Protocol::v113 && product.output_rate.empty());

  uint8_t limits[28] = {28, 255, 0x2b, 0};
  for (int i = 0; i < 6; ++i) {
    limits[4 + 4 * i + 2] = 0x7a;
    limits[4 + 4 * i + 3] = 0x43;
  }  // 250, little endian
  std::array<double, 6> factors;
  assert(conversionFactors(limits, sizeof(limits), factors) && factors[0] == .025);

  // Literal wire data checks field offsets and signed six-axis conversion independently.
  const uint8_t measurement[] = {20, 255, 0x32, 0, 0xff, 0xff, 2, 0, 3, 0,
                                 4,  0,   5,    0, 0,    0x80, 0, 0, 4, 0xff};
  const auto wrench = decodeWrench(measurement, factors);
  const std::array<double, 6> expected_wrench{{-.025, .05, .075, .1, .125, -819.2}};
  for (std::size_t i = 0; i < wrench.size(); ++i) {
    assert(std::abs(wrench[i] - expected_wrench[i]) < 1e-12);
  }
  assert(statusBits(measurement) == 4);

  const auto saved = factors;
  for (uint32_t invalid : {0u, 0xbf800000u, 0x7f800000u, 0x7fc00000u}) {
    for (int i = 0; i < 4; ++i) {
      limits[24 + i] = (invalid >> (8 * i)) & 255;
    }
    assert(!conversionFactors(limits, sizeof(limits), factors) && factors == saved);
  }

  uint8_t filter[] = {8, 255, 0xb6, 0, 3, 0, 0, 0};
  assert(validFilter(filter, sizeof(filter)));
  filter[5] = 1;
  assert(!validFilter(filter, sizeof(filter)));

  PublicationSchedule schedule(500);
  int selected = 0;
  for (int i = 0; i < 12000; ++i) {
    const auto t = Deadline{} + std::chrono::nanoseconds(int64_t(i) * 1000000000 / 1200);
    selected += schedule.select(t);
  }
  assert(selected == 5000);

  PublicationSchedule burst(500);
  selected = 0;
  for (int i = 0; i < 3; ++i) {
    const auto t = Deadline{} + std::chrono::milliseconds(i * 10);
    for (int j = 0; j < 12; ++j) {
      selected += burst.select(t);
    }
  }
  assert(selected == 3);

  PublicationSchedule all(0);
  assert(all.select(Deadline{}) && all.select(Deadline{}));
  assert(!validRate(-1) && !validRate(std::numeric_limits<double>::infinity()));

  StatusThrottle throttle;
  assert(throttle.warnings(1, Deadline{}) == 1 && throttle.warnings(1, Deadline{}) == 0);
  assert(throttle.warnings(2, Deadline{}) == 2);
  assert(throttle.warnings(3, Deadline{} + std::chrono::milliseconds(4999)) == 0);
  assert(throttle.warnings(3, Deadline{} + std::chrono::seconds(5)) == 3);
}
