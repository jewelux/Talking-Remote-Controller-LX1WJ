// Tests for the CDC-ACM port search in a USB configuration descriptor (firmware/usb_cdc_descriptor.cpp).
#include <stdint.h>

#include <vector>

#include "test_runner.h"
#include "usb_cdc_descriptor.h"

namespace {

using Bytes = std::vector<uint8_t>;

void append(Bytes& out, std::initializer_list<uint8_t> bytes) { out.insert(out.end(), bytes); }

void config(Bytes& out) { append(out, {9, 0x02, 0, 0, 2, 1, 0, 0x80, 50}); }

void iad(Bytes& out, uint8_t first) { append(out, {8, 0x0B, first, 2, 0x02, 0x02, 0x01, 0}); }

void interface(Bytes& out, uint8_t number, uint8_t alt, uint8_t endpoints, uint8_t cls, uint8_t subclass) {
  append(out, {9, 0x04, number, alt, endpoints, cls, subclass, 0, 0});
}

void endpoint(Bytes& out, uint8_t address, uint8_t type, uint8_t mps) {
  append(out, {7, 0x05, address, type, mps, 0, 0});
}

// A CDC-ACM function: the ACM control interface with its class descriptors and interrupt
// endpoint, then the data interface with a bulk IN and OUT.
void acmFunction(Bytes& out, uint8_t comm, uint8_t epNotify, uint8_t epIn, uint8_t epOut, uint8_t outMps) {
  iad(out, comm);
  interface(out, comm, 0, 1, 0x02, 0x02);
  append(out, {5, 0x24, 0x00, 0x10, 0x01});           // header
  append(out, {5, 0x24, 0x01, 0x00, (uint8_t)(comm + 1)});  // call management
  append(out, {4, 0x24, 0x02, 0x02});                 // ACM
  append(out, {5, 0x24, 0x06, comm, (uint8_t)(comm + 1)});  // union
  endpoint(out, epNotify, 0x03, 8);
  interface(out, (uint8_t)(comm + 1), 0, 2, 0x0A, 0x00);
  endpoint(out, epIn, 0x02, 64);
  endpoint(out, epOut, 0x02, outMps);
}

void finish(Bytes& out) {
  out[2] = (uint8_t)(out.size() & 0xFF);
  out[3] = (uint8_t)(out.size() >> 8);
}

// The X6100's CH342 as read on a PC: port A on interfaces 0/1, port B (CAT) on 2/3, OUT MPS 32.
Bytes ch342() {
  Bytes d;
  config(d);
  acmFunction(d, 0, 0x81, 0x82, 0x02, 32);
  acmFunction(d, 2, 0x83, 0x84, 0x04, 32);
  finish(d);
  return d;
}

// A composite radio: USB audio first (with a streaming interface whose alternate setting 1 has an
// isochronous endpoint), then two CDC-ACM ports, as on an IC-705 or a QMX.
Bytes audioThenTwoPorts() {
  Bytes d;
  config(d);
  interface(d, 0, 0, 0, 0x01, 0x01);  // audio control
  interface(d, 1, 0, 0, 0x01, 0x02);  // audio streaming, idle
  interface(d, 1, 1, 1, 0x01, 0x02);
  endpoint(d, 0x83, 0x05, 192);      // isochronous IN
  acmFunction(d, 2, 0x85, 0x86, 0x06, 64);
  acmFunction(d, 4, 0x87, 0x88, 0x08, 64);
  finish(d);
  return d;
}

}  // namespace

TEST(first_port_is_index_0) {
  const Bytes d = ch342();
  CdcPortDesc port;
  CHECK(findCdcSerialPort(d.data(), d.size(), 0, port));
  CHECK_EQ(port.dataIntf, 1);
  CHECK_EQ(port.commIntf, 0);
  CHECK_EQ(port.epIn, 0x82);
  CHECK_EQ(port.epOut, 0x02);
}

// The X6100 profile's port B.
TEST(second_port_is_index_1) {
  const Bytes d = ch342();
  CdcPortDesc port;
  CHECK(findCdcSerialPort(d.data(), d.size(), 1, port));
  CHECK_EQ(port.dataIntf, 3);
  CHECK_EQ(port.commIntf, 2);
  CHECK_EQ(port.epIn, 0x84);
  CHECK_EQ(port.epOut, 0x04);
}

// Never another port instead of the one asked for.
TEST(missing_port_is_not_found) {
  const Bytes d = ch342();
  CdcPortDesc port;
  CHECK(!findCdcSerialPort(d.data(), d.size(), 2, port));
}

TEST(audio_interfaces_are_skipped) {
  const Bytes d = audioThenTwoPorts();
  CdcPortDesc port;
  CHECK(findCdcSerialPort(d.data(), d.size(), 0, port));
  CHECK_EQ(port.dataIntf, 3);
  CHECK_EQ(port.commIntf, 2);
  CHECK_EQ(port.epIn, 0x86);
  CHECK(findCdcSerialPort(d.data(), d.size(), 1, port));
  CHECK_EQ(port.dataIntf, 5);
  CHECK_EQ(port.commIntf, 4);
  CHECK_EQ(port.epOut, 0x08);
}

// A sound card (the X6100's CM108) has no serial port.
TEST(device_without_cdc_has_no_port) {
  Bytes d;
  config(d);
  interface(d, 0, 0, 0, 0x01, 0x01);
  interface(d, 3, 0, 1, 0x03, 0x00);  // HID
  endpoint(d, 0x87, 0x03, 4);
  finish(d);
  CdcPortDesc port;
  CHECK(!findCdcSerialPort(d.data(), d.size(), 0, port));
}

// A data interface without both bulk endpoints is not a port, and does not count.
TEST(data_interface_needs_bulk_in_and_out) {
  Bytes d;
  config(d);
  interface(d, 0, 0, 1, 0x0A, 0x00);
  endpoint(d, 0x81, 0x02, 64);  // IN only
  acmFunction(d, 1, 0x83, 0x84, 0x04, 64);
  finish(d);
  CdcPortDesc port;
  CHECK(findCdcSerialPort(d.data(), d.size(), 0, port));
  CHECK_EQ(port.dataIntf, 2);
  CHECK(!findCdcSerialPort(d.data(), d.size(), 1, port));
}

// A data interface with no ACM control interface before it opens without line setup.
TEST(data_interface_without_acm_has_no_control) {
  Bytes d;
  config(d);
  interface(d, 0, 0, 2, 0x0A, 0x00);
  endpoint(d, 0x81, 0x02, 64);
  endpoint(d, 0x01, 0x02, 64);
  finish(d);
  CdcPortDesc port;
  CHECK(findCdcSerialPort(d.data(), d.size(), 0, port));
  CHECK_EQ(port.commIntf, kCdcNoInterface);
}

// A descriptor cut short or with a zero length ends the walk instead of reading past it.
TEST(broken_descriptor_is_safe) {
  Bytes d = ch342();
  CdcPortDesc port;
  CHECK(!findCdcSerialPort(d.data(), 1, 0, port));
  CHECK(!findCdcSerialPort(nullptr, 0, 0, port));
  d[9] = 0;  // the first descriptor after the configuration
  CHECK(!findCdcSerialPort(d.data(), d.size(), 0, port));
  const Bytes whole = ch342();
  CHECK(!findCdcSerialPort(whole.data(), whole.size() - 3, 1, port));  // port B's OUT endpoint cut
}
