#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <signal.h>

static volatile int keep_running = 1;

void intHandler(int dummy)
{
  keep_running = 0;
}

// NOTE this answer if compiling errors for symbol CRTSCTS
// https://stackoverflow.com/questions/26944217/crtscts-not-define-when-compiling-as-c99

#define TTY_NAME_LEN 64

struct context_tty
{
  char serial_port_device[TTY_NAME_LEN];
  int serial_port;
  struct termios tty;
} ctx_tty;

void clear_context_tty()
{
  memset(ctx_tty.serial_port_device, 0, sizeof(ctx_tty.serial_port_device));
  ctx_tty.serial_port = 0;
}

void name_context_tty(char *buf)
{
  snprintf(ctx_tty.serial_port_device, TTY_NAME_LEN, "%s", buf);
}

int open_context_tty()
{
  ctx_tty.serial_port = open(ctx_tty.serial_port_device, O_RDWR);

  if (tcgetattr(ctx_tty.serial_port, &ctx_tty.tty) != 0)
  {
    fprintf(stderr, "ERROR: %i from tcgetattr: %s\n", errno, strerror(errno));
    return 1;
  }

  ctx_tty.tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
  ctx_tty.tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
  ctx_tty.tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
  ctx_tty.tty.c_cflag |= CS8;            // 8 bits per byte (most common)
  ctx_tty.tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
  ctx_tty.tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  ctx_tty.tty.c_lflag &= ~ICANON;
  ctx_tty.tty.c_lflag &= ~ECHO;                                                        // Disable echo
  ctx_tty.tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
  ctx_tty.tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
  ctx_tty.tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
  ctx_tty.tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
  ctx_tty.tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

  ctx_tty.tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  ctx_tty.tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // ctx_tty.tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // ctx_tty.tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  ctx_tty.tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  ctx_tty.tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&ctx_tty.tty, B9600);
  cfsetospeed(&ctx_tty.tty, B9600);

  if (tcsetattr(ctx_tty.serial_port, TCSANOW, &ctx_tty.tty) != 0)
  {
    fprintf(stderr, "ERROR: %i from tcsetattr: %s\n", errno, strerror(errno));
    return 1;
  }

  return 0;
}

void close_context_tty()
{
  close(ctx_tty.serial_port);
}

#define PARSE_BUFFER_SIZE 256

struct context_parse_buffer
{
  uint8_t read_char[1];
  uint8_t parse_buf[PARSE_BUFFER_SIZE];
  int buf_len;
  int buf_p;
} ctx_parse_buf;

void clear_parse_buffer()
{
  memset(ctx_parse_buf.parse_buf, 0, PARSE_BUFFER_SIZE);
  ctx_parse_buf.buf_len = 0;
  ctx_parse_buf.buf_p = 0;
}

void hex_dump(struct context_parse_buffer *buf)
{
  fprintf(stdout, "BUF: ");
  for (int i = 0; i < buf->buf_p; i++)
  {
    fprintf(stdout, "0x%02x ", buf->parse_buf[i]);
  }
  fprintf(stdout, "\n");
}

#define UPDATE_ACC 1 << 0
#define UPDATE_GYRO 1 << 1
#define UPDATE_ANGLE 1 << 2
#define UPDATE_MAG 1 << 3
#define UPDATE_PRESS 1 << 4
#define UPDATE_LONG_LAT 1 << 5
#define UPDATE_GPS 1 << 6
#define UPDATE_QUART 1 << 7
#define UPDATE_GPS_ACC 1 << 8

double acc_range = 16.0;
double gyro_range = 2000.0;
double angle_range = 180.0;

struct context_device
{
  int updated;
  float acc_x, acc_y, acc_z;
  float gyro_x, gyro_y, gyro_z;
  float angle_x, angle_y, angle_z;
  int16_t mag_x, mag_y, mag_z;
  float temperature;
  uint16_t version;
  float pressure; // in hPa
  uint32_t altitude;
  double longitude;
  double latitude;
  float gps_height;
  float gps_yaw;
  float gps_speed;
  float q0, q1, q2, q3;
  uint16_t num_sat;
  float pos_acc;
  float pos_acc_h, pos_acc_v;
} ctx_device;

void print_device_data()
{
  if (ctx_device.updated > 0)
  {
    fprintf(stdout, "ACC  : %f, %f, %f \n", ctx_device.acc_x, ctx_device.acc_y, ctx_device.acc_z);
    fprintf(stdout, "TEMP : %f \n", ctx_device.temperature);
    fprintf(stdout, "GYRO : %f, %f, %f \n", ctx_device.gyro_x, ctx_device.gyro_y, ctx_device.gyro_z);
    fprintf(stdout, "ANGLE: %f, %f, %f \n", ctx_device.angle_x, ctx_device.angle_y, ctx_device.angle_z);
    fprintf(stdout, "VERS : %d \n", ctx_device.version);
    fprintf(stdout, "MAG  : %d, %d, %d \n", ctx_device.mag_x, ctx_device.mag_y, ctx_device.mag_z);
    fprintf(stdout, "PRESS: %f \n", ctx_device.pressure);
    fprintf(stdout, "ALT  : %d \n", ctx_device.altitude);
    fprintf(stdout, "LONG : %lf \n", ctx_device.longitude);
    fprintf(stdout, "LAT  : %lf \n", ctx_device.latitude);
    fprintf(stdout, "GPS H: %f \n", ctx_device.gps_height);
    fprintf(stdout, "GPS Y: %f \n", ctx_device.gps_yaw);
    fprintf(stdout, "GPS S: %f \n", ctx_device.gps_speed);
    fprintf(stdout, "GPS N: %d \n", ctx_device.num_sat);
    fprintf(stdout, "POS A: %f \n", ctx_device.pos_acc);
    fprintf(stdout, "POS H: %f \n", ctx_device.pos_acc_h);
    fprintf(stdout, "POS V: %f \n", ctx_device.pos_acc_v);
    fprintf(stdout, "QUART: %f, %f, %f, %f \n", ctx_device.q0, ctx_device.q1, ctx_device.q2, ctx_device.q3);
  }
}

int checksump_parse_buf(struct context_parse_buffer *buf, uint8_t lead_byte)
{
  uint8_t checksum = 0x55 + lead_byte + buf->parse_buf[2] + buf->parse_buf[3] +
                     buf->parse_buf[4] + buf->parse_buf[5] + buf->parse_buf[6] +
                     buf->parse_buf[7] + buf->parse_buf[8] + buf->parse_buf[9];

  int return_val = (checksum == buf->parse_buf[10]);
  return return_val;
}

void parse_acc(struct context_parse_buffer *buf)
{

  if (!checksump_parse_buf(buf, 0x51))
    return;

  uint8_t axl = buf->parse_buf[2];
  uint8_t axh = buf->parse_buf[3];
  uint8_t ayl = buf->parse_buf[4];
  uint8_t ayh = buf->parse_buf[5];
  uint8_t azl = buf->parse_buf[6];
  uint8_t azh = buf->parse_buf[7];
  uint16_t temp_val = (buf->parse_buf[9] << 8 | buf->parse_buf[8]);

  ctx_device.acc_x = (axh << 8 | axl) / 32768.0 * acc_range;
  ctx_device.acc_y = (ayh << 8 | ayl) / 32768.0 * acc_range;
  ctx_device.acc_z = (azh << 8 | azl) / 32768.0 * acc_range;
  if (ctx_device.acc_x >= acc_range)
    ctx_device.acc_x -= 2 * acc_range;
  if (ctx_device.acc_y >= acc_range)
    ctx_device.acc_y -= 2 * acc_range;
  if (ctx_device.acc_z >= acc_range)
    ctx_device.acc_z -= 2 * acc_range;
  ctx_device.temperature = temp_val / 100.0;

  ctx_device.updated |= UPDATE_ACC;
}

void parse_gyro(struct context_parse_buffer *buf)
{
  if (!checksump_parse_buf(buf, 0x52))
    return;

  uint8_t wxl = buf->parse_buf[2];
  uint8_t wxh = buf->parse_buf[3];
  uint8_t wyl = buf->parse_buf[4];
  uint8_t wyh = buf->parse_buf[5];
  uint8_t wzl = buf->parse_buf[6];
  uint8_t wzh = buf->parse_buf[7];

  ctx_device.gyro_x = (wxh << 8 | wxl) / 32768.0 * gyro_range;
  ctx_device.gyro_y = (wyh << 8 | wyl) / 32768.0 * gyro_range;
  ctx_device.gyro_z = (wzh << 8 | wzl) / 32768.0 * gyro_range;
  if (ctx_device.gyro_x >= gyro_range)
    ctx_device.gyro_x -= 2 * gyro_range;
  if (ctx_device.gyro_y >= gyro_range)
    ctx_device.gyro_y -= 2 * gyro_range;
  if (ctx_device.gyro_z >= gyro_range)
    ctx_device.gyro_z -= 2 * gyro_range;

  ctx_device.updated |= UPDATE_GYRO;
}

void parse_angle(struct context_parse_buffer *buf)
{
  if (!checksump_parse_buf(buf, 0x53))
    return;

  uint8_t rxl = buf->parse_buf[2];
  uint8_t rxh = buf->parse_buf[3];
  uint8_t ryl = buf->parse_buf[4];
  uint8_t ryh = buf->parse_buf[5];
  uint8_t rzl = buf->parse_buf[6];
  uint8_t rzh = buf->parse_buf[7];

  ctx_device.version = (buf->parse_buf[9] << 8 | buf->parse_buf[8]);

  ctx_device.angle_x = (rxh << 8 | rxl) / 32768.0 * angle_range;
  ctx_device.angle_y = (ryh << 8 | ryl) / 32768.0 * angle_range;
  ctx_device.angle_z = (rzh << 8 | rzl) / 32768.0 * angle_range;
  if (ctx_device.angle_x >= angle_range)
    ctx_device.angle_x -= 2 * angle_range;
  if (ctx_device.angle_y >= angle_range)
    ctx_device.angle_y -= 2 * angle_range;
  if (ctx_device.angle_z >= angle_range)
    ctx_device.angle_z -= 2 * angle_range;

  ctx_device.updated |= UPDATE_ANGLE;
}

void parse_mag(struct context_parse_buffer *buf)
{
  if (!checksump_parse_buf(buf, 0x54))
    return;

  uint8_t mxl = buf->parse_buf[2];
  uint8_t mxh = buf->parse_buf[3];
  uint8_t myl = buf->parse_buf[4];
  uint8_t myh = buf->parse_buf[5];
  uint8_t mzl = buf->parse_buf[6];
  uint8_t mzh = buf->parse_buf[7];

  ctx_device.mag_x = (mxh << 8 | mxl);
  ctx_device.mag_y = (myh << 8 | myl);
  ctx_device.mag_z = (mzh << 8 | mzl);

  ctx_device.updated |= UPDATE_MAG;
}

void parse_press_alt(struct context_parse_buffer *buf)
{
  if (!checksump_parse_buf(buf, 0x56))
    return;

  uint8_t p0 = buf->parse_buf[2];
  uint8_t p1 = buf->parse_buf[3];
  uint8_t p2 = buf->parse_buf[4];
  uint8_t p3 = buf->parse_buf[5];
  uint8_t h0 = buf->parse_buf[6];
  uint8_t h1 = buf->parse_buf[7];
  uint8_t h2 = buf->parse_buf[6];
  uint8_t h3 = buf->parse_buf[7];

  ctx_device.pressure = ((p3 << 24) | (p2 << 16) | (p1 << 8) | p0) / 100.0;
  ctx_device.altitude = (h3 << 24) | (h2 << 16) | (h1 << 8) | h0;

  ctx_device.updated |= UPDATE_PRESS;
}

// Convert from sensor format to decimal gps
double sensor_to_decimal_gps_convert(uint32_t input)
{
  int degrees = input / 10000000;
  double minutes = input % 10000000 / 100000.0;
  double minutes_dec = (minutes / 60.0);
  return degrees + minutes_dec;
}

void parse_long_lat(struct context_parse_buffer *buf)
{
  if (!checksump_parse_buf(buf, 0x57))
    return;

  uint8_t g0 = buf->parse_buf[2];
  uint8_t g1 = buf->parse_buf[3];
  uint8_t g2 = buf->parse_buf[4];
  uint8_t g3 = buf->parse_buf[5];
  uint8_t t0 = buf->parse_buf[6];
  uint8_t t1 = buf->parse_buf[7];
  uint8_t t2 = buf->parse_buf[8];
  uint8_t t3 = buf->parse_buf[9];

  uint32_t long_reg = (g3 << 24) | (g2 << 16) | (g1 << 8) | g0;
  uint32_t lat_reg = (t3 << 24) | (t2 << 16) | (t1 << 8) | t0;

  ctx_device.longitude = sensor_to_decimal_gps_convert(long_reg);
  ctx_device.latitude = sensor_to_decimal_gps_convert(lat_reg);

  ctx_device.updated |= UPDATE_LONG_LAT;
}

void parse_gps(struct context_parse_buffer *buf)
{
  if (!checksump_parse_buf(buf, 0x58))
    return;

  uint8_t h0 = buf->parse_buf[2];
  uint8_t h1 = buf->parse_buf[3];
  uint8_t y0 = buf->parse_buf[4];
  uint8_t y1 = buf->parse_buf[5];
  uint8_t s0 = buf->parse_buf[6];
  uint8_t s1 = buf->parse_buf[7];
  uint8_t s2 = buf->parse_buf[8];
  uint8_t s3 = buf->parse_buf[9];

  ctx_device.gps_height = (h1 << 8 | h0) / 10.0;
  ctx_device.gps_yaw = (y1 << 8 | y0) / 100.0;
  uint32_t speed_reg = (s3 << 24) | (s2 << 16) | (s1 << 8) | s0;
  ctx_device.gps_speed = (float)speed_reg;

  ctx_device.updated |= UPDATE_GPS;
}

void parse_quart(struct context_parse_buffer *buf)
{
  if (!checksump_parse_buf(buf, 0x59))
    return;

  uint8_t q0l = buf->parse_buf[2];
  uint8_t q0h = buf->parse_buf[3];
  uint8_t q1l = buf->parse_buf[4];
  uint8_t q1h = buf->parse_buf[5];
  uint8_t q2l = buf->parse_buf[6];
  uint8_t q2h = buf->parse_buf[7];
  uint8_t q3l = buf->parse_buf[8];
  uint8_t q3h = buf->parse_buf[9];

  ctx_device.q0 = (q0h << 8 | q0l) / 32768.0;
  ctx_device.q1 = (q1h << 8 | q1l) / 32768.0;
  ctx_device.q2 = (q2h << 8 | q2l) / 32768.0;
  ctx_device.q3 = (q3h << 8 | q3l) / 32768.0;

  ctx_device.updated |= UPDATE_QUART;
}

void parse_gps_acc(struct context_parse_buffer *buf)
{
  if (!checksump_parse_buf(buf, 0x5A))
    return;

  uint8_t nsl = buf->parse_buf[2];
  uint8_t nsh = buf->parse_buf[3];
  uint8_t pal = buf->parse_buf[4];
  uint8_t pah = buf->parse_buf[5];
  uint8_t hpal = buf->parse_buf[6];
  uint8_t hpah = buf->parse_buf[7];
  uint8_t vpal = buf->parse_buf[8];
  uint8_t vpah = buf->parse_buf[9];

  ctx_device.num_sat = (nsh << 8 | nsl);
  ctx_device.pos_acc = (pah << 8 | pal) / 100.0;
  ctx_device.pos_acc_h = (hpah << 8 | hpal) / 100.0;
  ctx_device.pos_acc_v = (vpah << 8 | vpal) / 100.0;

  ctx_device.updated |= UPDATE_GPS_ACC;
}

void parse(struct context_parse_buffer *buf)
{
  switch (buf->parse_buf[0x01])
  {
  case 0x51:
    parse_acc(buf);
    break;
  case 0x52:
    parse_gyro(buf);
    break;
  case 0x53:
    parse_angle(buf);
    break;
  case 0x54:
    parse_mag(buf);
    break;
  case 0x56:
    parse_press_alt(buf);
    break;
  case 0x57:
    parse_long_lat(buf);
    break;
  case 0x58:
    parse_gps(buf);
    break;
  case 0x59:
    parse_quart(buf);
    break;
  case 0x5A:
    parse_gps_acc(buf);
    break;
  default:
    fprintf(stdout, "UNKNOWN \n");
    break;
  }
}

int main(int argc, char *argv[])
{
  signal(SIGINT, intHandler);

  clear_context_tty();
  name_context_tty("/dev/ttyUSB1");
  if (open_context_tty() != 0)
  {
    fprintf(stdout, "ERROR: tty failure\n\n");
    return 1;
  };

  clear_parse_buffer();
  while (keep_running)
  {
    // Read one byte at a time, Wit Motion gets a bit stressed by more than one byte :/
    ctx_parse_buf.buf_len = read(ctx_tty.serial_port, &ctx_parse_buf.read_char[0], 1);
    if (ctx_parse_buf.buf_len > 0)
    {
      if (ctx_parse_buf.read_char[0] == 0x55)
        clear_parse_buffer();
      ctx_parse_buf.parse_buf[ctx_parse_buf.buf_p++] = ctx_parse_buf.read_char[0];
      if (ctx_parse_buf.buf_p == 11)
      {
        // hex_dump(&ctx_parse_buf);
        parse(&ctx_parse_buf);
        print_device_data();
        ctx_device.updated = 0;
      }
    }
  }

  fprintf(stdout, "Quitting...\n");
  close_context_tty();
  return 0;
}