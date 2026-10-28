// Tests sans matériel : le port série est remplacé par un pipe.
// Trames de référence calculées à part (Python, binascii.crc_hqx).
#include <gtest/gtest.h>

#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <vector>

#include "wesc/wesc.h"

namespace
{

class Pipe : public ::testing::Test
{
protected:
  void SetUp() override {ASSERT_EQ(pipe(fds_), 0);}
  void TearDown() override
  {
    close(fds_[0]);
    if (fds_[1] >= 0) {
      close(fds_[1]);
    }
  }

  int rd() const {return fds_[0];}
  int wr() const {return fds_[1];}
  void close_writer() {close(fds_[1]); fds_[1] = -1;}

  std::vector<uint8_t> drain()
  {
    uint8_t buf[600];
    ssize_t n = read(fds_[0], buf, sizeof(buf));
    return n > 0 ? std::vector<uint8_t>(buf, buf + n) : std::vector<uint8_t>{};
  }

  void feed(const std::vector<uint8_t> & bytes)
  {
    ASSERT_EQ(write(fds_[1], bytes.data(), bytes.size()), static_cast<ssize_t>(bytes.size()));
  }

  int fds_[2] = {-1, -1};
};

using Bytes = std::vector<uint8_t>;

// Trame courte autour d'une charge utile, CRC calculé par la bibliothèque
Bytes frame(const Bytes & payload)
{
  uint16_t crc = wesc_crc16(payload.data(), payload.size());
  Bytes out = {0x02, static_cast<uint8_t>(payload.size())};
  out.insert(out.end(), payload.begin(), payload.end());
  out.insert(out.end(), {static_cast<uint8_t>(crc >> 8), static_cast<uint8_t>(crc), 0x03});
  return out;
}

void put16(Bytes & b, int16_t v)
{
  b.push_back(static_cast<uint8_t>(static_cast<uint16_t>(v) >> 8));
  b.push_back(static_cast<uint8_t>(v));
}

void put32(Bytes & b, int32_t v)
{
  for (int shift = 24; shift >= 0; shift -= 8) {
    b.push_back(static_cast<uint8_t>(static_cast<uint32_t>(v) >> shift));
  }
}

// Réponse GET_VALUES complète (73 octets), valeurs connues
Bytes values_payload(bool full = true)
{
  Bytes p = {WESC_COMM_GET_VALUES};
  put16(p, 254);        // temp_fet 25,4 °C
  put16(p, -12);        // temp_motor -1,2 °C
  put32(p, 150);        // avg_motor_current 1,5 A
  put32(p, -25);        // avg_input_current -0,25 A
  put32(p, 10);         // avg_id 0,1 A
  put32(p, 20);         // avg_iq 0,2 A
  put16(p, -60);        // duty -0,06
  put32(p, -4614);      // rpm
  put16(p, 157);        // v_in 15,7 V
  put32(p, 12345);      // amp_hours 1,2345
  put32(p, 1);          // amp_hours_charged
  put32(p, 20000);      // watt_hours 2
  put32(p, 0);          // watt_hours_charged
  put32(p, -1000);      // tachometer
  put32(p, 5000);       // tachometer_abs
  p.push_back(0);       // fault
  put32(p, 90000000);   // pid_pos_now 90°
  p.push_back(7);       // controller id
  if (full) {
    put16(p, 301);      // temp_mos1
    put16(p, 302);
    put16(p, 303);
    put32(p, 1500);     // avg_vd 1,5 V
    put32(p, -500);     // avg_vq -0,5 V
  }
  return p;
}

TEST(Crc, KnownVector)
{
  const uint8_t data[] = "123456789";
  EXPECT_EQ(wesc_crc16(data, 9), 0x31C3);  // CRC16/XMODEM
  EXPECT_EQ(wesc_crc16(data, 0), 0x0000);
}

TEST_F(Pipe, EncodeAlive)
{
  ASSERT_EQ(wesc_send_alive(wr()), 0);
  EXPECT_EQ(drain(), (Bytes{0x02, 0x01, 0x1e, 0xf3, 0xff, 0x03}));
}

TEST_F(Pipe, EncodeGetValues)
{
  ASSERT_EQ(wesc_request_values(wr()), 0);
  EXPECT_EQ(drain(), (Bytes{0x02, 0x01, 0x04, 0x40, 0x84, 0x03}));
}

TEST_F(Pipe, EncodeDuty)
{
  ASSERT_EQ(wesc_set_duty(wr(), 0.06f), 0);
  EXPECT_EQ(drain(), (Bytes{0x02, 0x05, 0x05, 0x00, 0x00, 0x17, 0x70, 0xc7, 0x24, 0x03}));
  ASSERT_EQ(wesc_set_duty(wr(), -0.06f), 0);
  EXPECT_EQ(drain(), (Bytes{0x02, 0x05, 0x05, 0xff, 0xff, 0xe8, 0x90, 0xbd, 0x35, 0x03}));
}

TEST_F(Pipe, EncodeDutyClamped)
{
  ASSERT_EQ(wesc_set_duty(wr(), 5.0f), 0);
  Bytes out = drain();
  ASSERT_EQ(out.size(), 10u);
  EXPECT_EQ(Bytes(out.begin() + 3, out.begin() + 7), (Bytes{0x00, 0x01, 0x86, 0xa0}));  // 100000
}

TEST_F(Pipe, EncodeRpm)
{
  ASSERT_EQ(wesc_set_rpm(wr(), -4614), 0);
  EXPECT_EQ(drain(), (Bytes{0x02, 0x05, 0x08, 0xff, 0xff, 0xed, 0xfa, 0xae, 0x56, 0x03}));
}

TEST_F(Pipe, EncodeServo)
{
  ASSERT_EQ(wesc_set_servo(wr(), 0.5f), 0);
  EXPECT_EQ(drain(), (Bytes{0x02, 0x03, 0x0c, 0x01, 0xf4, 0xe9, 0xcb, 0x03}));
}

TEST_F(Pipe, EncodeCurrentAndBrake)
{
  ASSERT_EQ(wesc_set_current(wr(), 2.5f), 0);
  EXPECT_EQ(drain(), (Bytes{0x02, 0x05, 0x06, 0x00, 0x00, 0x09, 0xc4, 0xee, 0xd5, 0x03}));
  ASSERT_EQ(wesc_set_brake(wr(), 10.0f), 0);
  EXPECT_EQ(drain(), (Bytes{0x02, 0x05, 0x07, 0x00, 0x00, 0x27, 0x10, 0xea, 0x94, 0x03}));
}

TEST_F(Pipe, ReadValues)
{
  feed(frame(values_payload()));
  wesc_values_t v;
  ASSERT_EQ(wesc_read_values(rd(), &v, 100), 0);
  EXPECT_FLOAT_EQ(v.temp_fet, 25.4f);
  EXPECT_FLOAT_EQ(v.temp_motor, -1.2f);
  EXPECT_FLOAT_EQ(v.avg_motor_current, 1.5f);
  EXPECT_FLOAT_EQ(v.avg_input_current, -0.25f);
  EXPECT_FLOAT_EQ(v.duty_cycle_now, -0.06f);
  EXPECT_EQ(v.rpm, -4614);
  EXPECT_FLOAT_EQ(v.v_in, 15.7f);
  EXPECT_FLOAT_EQ(v.amp_hours, 1.2345f);
  EXPECT_FLOAT_EQ(v.watt_hours, 2.0f);
  EXPECT_EQ(v.tachometer, -1000);
  EXPECT_EQ(v.tachometer_abs, 5000);
  EXPECT_FLOAT_EQ(v.pid_pos_now, 90.0f);
  EXPECT_EQ(v.app_controller_id, 7);
  EXPECT_FLOAT_EQ(v.temp_mos1, 30.1f);
  EXPECT_FLOAT_EQ(v.temp_mos3, 30.3f);
  EXPECT_FLOAT_EQ(v.avg_vd, 1.5f);
  EXPECT_FLOAT_EQ(v.avg_vq, -0.5f);
}

TEST_F(Pipe, ReadValuesShortFirmware)
{
  feed(frame(values_payload(false)));
  wesc_values_t v;
  ASSERT_EQ(wesc_read_values(rd(), &v, 100), 0);
  EXPECT_FLOAT_EQ(v.v_in, 15.7f);
  EXPECT_EQ(v.temp_mos1, 0.0f);
}

TEST_F(Pipe, ResyncAfterGarbage)
{
  Bytes data = {0xff, 0x00, 0x42};
  Bytes f = frame(values_payload());
  data.insert(data.end(), f.begin(), f.end());
  feed(data);
  wesc_values_t v;
  ASSERT_EQ(wesc_read_values(rd(), &v, 100), 0);
  EXPECT_EQ(v.rpm, -4614);
}

TEST_F(Pipe, SkipsOtherPackets)
{
  Bytes data = frame({WESC_COMM_FW_VERSION, 6, 0});
  Bytes f = frame(values_payload());
  data.insert(data.end(), f.begin(), f.end());
  feed(data);
  wesc_values_t v;
  ASSERT_EQ(wesc_read_values(rd(), &v, 100), 0);
  EXPECT_FLOAT_EQ(v.v_in, 15.7f);
}

TEST_F(Pipe, LongFrame)
{
  Bytes payload(300, 0xab);
  payload[0] = WESC_COMM_FW_VERSION;
  ASSERT_EQ(wesc_write_payload(wr(), payload.data(), payload.size()), 306);
  uint8_t buf[WESC_MAX_PAYLOAD];
  ASSERT_EQ(wesc_read_packet(rd(), buf, sizeof(buf), 100), 300);
  EXPECT_EQ(Bytes(buf, buf + 300), payload);
}

TEST_F(Pipe, BadCrc)
{
  Bytes f = frame(values_payload());
  f[f.size() - 2] ^= 0x01;
  feed(f);
  wesc_values_t v;
  EXPECT_EQ(wesc_read_values(rd(), &v, 100), -1);
  EXPECT_EQ(errno, EBADMSG);
}

TEST_F(Pipe, BadEndByte)
{
  Bytes f = frame(values_payload());
  f.back() = 0x00;
  feed(f);
  wesc_values_t v;
  EXPECT_EQ(wesc_read_values(rd(), &v, 100), -1);
  EXPECT_EQ(errno, EPROTO);
}

TEST_F(Pipe, TruncatedFrameTimesOut)
{
  Bytes f = frame(values_payload());
  f.resize(f.size() / 2);
  feed(f);
  wesc_values_t v;
  EXPECT_EQ(wesc_read_values(rd(), &v, 30), -1);
  EXPECT_EQ(errno, ETIMEDOUT);
}

TEST_F(Pipe, NothingTimesOut)
{
  uint8_t buf[16];
  EXPECT_EQ(wesc_read_packet(rd(), buf, sizeof(buf), 20), -1);
  EXPECT_EQ(errno, ETIMEDOUT);
}

TEST_F(Pipe, ClosedPort)
{
  close_writer();
  uint8_t buf[16];
  EXPECT_EQ(wesc_read_packet(rd(), buf, sizeof(buf), 100), -1);
  EXPECT_EQ(errno, ECONNRESET);
}

TEST(Parse, Rejects)
{
  Bytes p = values_payload();
  wesc_values_t v;
  EXPECT_EQ(wesc_parse_values(p.data(), 40, &v), -1);
  EXPECT_EQ(errno, EMSGSIZE);
  p[0] = WESC_COMM_ALIVE;
  EXPECT_EQ(wesc_parse_values(p.data(), p.size(), &v), -1);
  EXPECT_EQ(errno, EPROTO);
}

TEST(Open, BadBaud)
{
  EXPECT_EQ(wesc_open("/dev/null", 1234), -1);
  EXPECT_EQ(errno, EINVAL);
}

}  // namespace
