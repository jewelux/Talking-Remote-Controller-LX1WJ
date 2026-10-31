#include "transport_usb_host.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>
#include <stdarg.h>

#include "soc/rtc_cntl_struct.h"
#include "soc/usb_dwc_struct.h"
#include "soc/usb_wrap_struct.h"
#include "usb/usb_host.h"

// Two tasks on core 0 run the USB host: usbLib runs the library, usbClient opens the radio's
// serial port and moves its bytes. The main loop talks to usbClient through two stream buffers,
// so it never waits on USB itself.

namespace {

// CDC-ACM (USB CDC PSTN subclass): the ACM control interface and its class requests.
constexpr uint8_t kCdcSubclassAcm = 0x02;
constexpr uint8_t kCdcSetLineCoding = 0x20;
constexpr uint8_t kCdcSetControlLineState = 0x22;
// A virtual COM port ignores the baud; the line coding is sent because some devices wait for it.
constexpr uint32_t kLineCodingBaud = 9600;
// DTR on, RTS off. The QMX keys nothing on either line.
constexpr uint16_t kControlLineDtr = 0x0001;
constexpr uint8_t kNoInterface = 0xFF;

constexpr size_t kRxBufferBytes = 2048;
constexpr size_t kTxBufferBytes = 256;
constexpr size_t kBulkTransferBytes = 64;  // the full-speed bulk packet; a multiple of any smaller one
constexpr size_t kControlTransferBytes = sizeof(usb_setup_packet_t) + 8;
constexpr size_t kMaxPendingDevices = 4;
constexpr size_t kLogLines = 12;
constexpr size_t kLogLineChars = 80;
// From the device descriptor to the library's new-device event takes well under a second.
constexpr uint32_t kEnumTimeoutMs = 3000;

enum class PortState : uint8_t { Idle, Configuring, Ready, Closing };

struct SerialPortDesc {
  uint8_t dataIntf;
  uint8_t commIntf;
  uint8_t epIn;
  uint8_t epOut;
};

usb_host_client_handle_t s_client = nullptr;
StreamBufferHandle_t s_rx = nullptr;  // usbClient -> main loop
StreamBufferHandle_t s_tx = nullptr;  // main loop -> usbClient
QueueHandle_t s_events = nullptr;
SemaphoreHandle_t s_logLock = nullptr;
usb_transfer_t* s_inXfer = nullptr;
usb_transfer_t* s_outXfer = nullptr;
usb_transfer_t* s_ctrlXfer = nullptr;

bool s_started = false;
bool s_holdOnce = false;
bool s_held = false;
esp_err_t s_startError = ESP_OK;

// The serial port. Only usbClient changes it; the main loop reads s_state.
volatile PortState s_state = PortState::Idle;
usb_device_handle_t s_dev = nullptr;
SerialPortDesc s_port = {0, kNoInterface, 0, 0};
uint8_t s_configStep = 0;
bool s_closeFailureLogged = false;
volatile bool s_inBusy = false;
// The read waits for room in s_rx (serviceRx).
bool s_inParked = false;
volatile bool s_outBusy = false;
volatile bool s_ctrlBusy = false;
uint8_t s_pendingAddrs[kMaxPendingDevices];
size_t s_pendingCount = 0;

// For USB?.
uint16_t s_vid = 0;
uint16_t s_pid = 0;
char s_product[33] = "";
volatile uint32_t s_rxBytes = 0;
volatile uint32_t s_txBytes = 0;
uint32_t s_devicesSeen = 0;
// A device that gave its device descriptor and is still being enumerated (set by usbLib in the
// enumeration filter, cleared by usbClient at the new-device event).
volatile bool s_enumPending = false;
volatile uint32_t s_enumSinceMs = 0;
volatile uint16_t s_enumVid = 0;
volatile uint16_t s_enumPid = 0;
char s_log[kLogLines][kLogLineChars];
size_t s_logNext = 0;
size_t s_logCount = 0;

// Keeps the line for USB? and prints it on Serial0, the console that is left while the host runs.
void logEvent(const char* format, ...) {
  char line[kLogLineChars];
  va_list args;
  va_start(args, format);
  vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  if (s_logLock && xSemaphoreTake(s_logLock, pdMS_TO_TICKS(50)) == pdTRUE) {
    snprintf(s_log[s_logNext], kLogLineChars, "%6lu ms %s", (unsigned long)millis(), line);
    s_logNext = (s_logNext + 1) % kLogLines;
    if (s_logCount < kLogLines) ++s_logCount;
    xSemaphoreGive(s_logLock);
  }
  Serial0.print("[USB] ");
  Serial0.println(line);
  if (!s_started) {
    Serial.print("[USB] ");
    Serial.println(line);
  }
}

void pushEvent(UsbLinkEvent event) {
  if (s_events) xQueueSend(s_events, &event, 0);
}

const char* stateName(PortState state) {
  switch (state) {
    case PortState::Idle: return "none";
    case PortState::Configuring: return "opening";
    case PortState::Ready: return "open";
    case PortState::Closing: return "closing";
  }
  return "?";
}

// The product string, cut to ASCII.
void productName(usb_device_handle_t dev, char* out, size_t outLen) {
  out[0] = '\0';
  usb_device_info_t info;
  if (usb_host_device_info(dev, &info) != ESP_OK || !info.str_desc_product) return;
  const usb_str_desc_t* s = info.str_desc_product;
  size_t n = 0;
  for (int i = 0; i < (s->bLength - 2) / 2 && n + 1 < outLen; ++i) {
    const uint16_t c = s->wData[i];
    out[n++] = (c >= 32 && c < 127) ? (char)c : '?';
  }
  out[n] = '\0';
}

// The first CDC data interface with a bulk IN and a bulk OUT endpoint, and the ACM control
// interface before it, which takes the line setup.
bool findSerialPort(const usb_config_desc_t* cfg, SerialPortDesc& port) {
  uint8_t lastComm = kNoInterface;
  int offset = 0;
  const usb_standard_desc_t* desc = reinterpret_cast<const usb_standard_desc_t*>(cfg);
  while ((desc = usb_parse_next_descriptor_of_type(desc, cfg->wTotalLength, USB_B_DESCRIPTOR_TYPE_INTERFACE,
                                                   &offset)) != nullptr) {
    const usb_intf_desc_t* intf = reinterpret_cast<const usb_intf_desc_t*>(desc);
    if (intf->bAlternateSetting != 0) continue;
    if (intf->bInterfaceClass == USB_CLASS_COMM && intf->bInterfaceSubClass == kCdcSubclassAcm) {
      lastComm = intf->bInterfaceNumber;
      continue;
    }
    if (intf->bInterfaceClass != USB_CLASS_CDC_DATA) continue;
    uint8_t epIn = 0;
    uint8_t epOut = 0;
    for (int i = 0; i < intf->bNumEndpoints; ++i) {
      int epOffset = offset;
      const usb_ep_desc_t* ep = usb_parse_endpoint_descriptor_by_index(intf, i, cfg->wTotalLength, &epOffset);
      if (!ep || (ep->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) != USB_BM_ATTRIBUTES_XFER_BULK) continue;
      if (ep->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK) {
        epIn = ep->bEndpointAddress;
      } else {
        epOut = ep->bEndpointAddress;
      }
    }
    if (epIn && epOut) {
      port = {intf->bInterfaceNumber, lastComm, epIn, epOut};
      return true;
    }
  }
  return false;
}

void submitIn() {
  s_inXfer->device_handle = s_dev;
  s_inXfer->bEndpointAddress = s_port.epIn;
  s_inXfer->num_bytes = kBulkTransferBytes;
  s_inBusy = true;
  const esp_err_t err = usb_host_transfer_submit(s_inXfer);
  if (err != ESP_OK) {
    s_inBusy = false;
    logEvent("read could not start: %s", esp_err_to_name(err));
  }
}

// Room for one more bulk packet. The read waits for it instead of dropping bytes: the device is
// NAKed until the main loop has taken what came, as with a NanoVNA's 3 KB scan reply.
bool rxHasRoom() {
  return xStreamBufferSpacesAvailable(s_rx) >= kBulkTransferBytes;
}

void onInDone(usb_transfer_t* xfer) {
  s_inBusy = false;
  if (xfer->status == USB_TRANSFER_STATUS_COMPLETED) {
    if (xfer->actual_num_bytes > 0) {
      xStreamBufferSend(s_rx, xfer->data_buffer, xfer->actual_num_bytes, 0);
      s_rxBytes += xfer->actual_num_bytes;
    }
    if (s_state != PortState::Ready) return;
    if (rxHasRoom()) {
      submitIn();
    } else {
      s_inParked = true;
    }
    return;
  }
  if (s_state == PortState::Ready) logEvent("read stopped: transfer status %d", (int)xfer->status);
}

void onOutDone(usb_transfer_t* xfer) {
  s_outBusy = false;
  if (xfer->status != USB_TRANSFER_STATUS_COMPLETED && s_state == PortState::Ready) {
    logEvent("write failed: transfer status %d", (int)xfer->status);
  }
}

void onControlDone(usb_transfer_t* xfer) {
  s_ctrlBusy = false;
  // Some devices refuse the line setup; the port works without it.
  if (xfer->status != USB_TRANSFER_STATUS_COMPLETED) {
    logEvent("line setup refused: transfer status %d", (int)xfer->status);
  }
}

void submitClassRequest(uint8_t request, uint16_t value, const uint8_t* data, uint16_t length) {
  usb_setup_packet_t* setup = reinterpret_cast<usb_setup_packet_t*>(s_ctrlXfer->data_buffer);
  setup->bmRequestType =
      USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_CLASS | USB_BM_REQUEST_TYPE_RECIP_INTERFACE;
  setup->bRequest = request;
  setup->wValue = value;
  setup->wIndex = s_port.commIntf;
  setup->wLength = length;
  if (length) memcpy(s_ctrlXfer->data_buffer + sizeof(usb_setup_packet_t), data, length);
  s_ctrlXfer->num_bytes = sizeof(usb_setup_packet_t) + length;
  s_ctrlXfer->device_handle = s_dev;
  s_ctrlXfer->bEndpointAddress = 0;
  s_ctrlXfer->timeout_ms = 1000;
  s_ctrlBusy = true;
  const esp_err_t err = usb_host_transfer_submit_control(s_client, s_ctrlXfer);
  if (err != ESP_OK) {
    s_ctrlBusy = false;
    logEvent("line setup could not start: %s", esp_err_to_name(err));
  }
}

// A new device: open its serial port, or report that it has none.
void inspectDevice(uint8_t addr) {
  usb_device_handle_t dev = nullptr;
  esp_err_t err = usb_host_device_open(s_client, addr, &dev);
  if (err != ESP_OK) {
    logEvent("device %u could not be opened: %s", addr, esp_err_to_name(err));
    return;
  }
#if !CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK
  ++s_devicesSeen;
#endif
  const usb_device_desc_t* dd = nullptr;
  const usb_config_desc_t* cfg = nullptr;
  (void)usb_host_get_device_descriptor(dev, &dd);
  (void)usb_host_get_active_config_descriptor(dev, &cfg);
  char product[sizeof(s_product)];
  productName(dev, product, sizeof(product));
  logEvent("device %04X:%04X class %02X \"%s\"", dd ? dd->idVendor : 0, dd ? dd->idProduct : 0,
           dd ? dd->bDeviceClass : 0, product);

  SerialPortDesc port;
  if ((dd && dd->bDeviceClass == USB_CLASS_HUB) || !cfg || !findSerialPort(cfg, port)) {
    logEvent("  no CDC-ACM serial port");
    usb_host_device_close(s_client, dev);
    if (!dd || dd->bDeviceClass != USB_CLASS_HUB) pushEvent(UsbLinkEvent::NoSerialPort);
    return;
  }
  if (s_state != PortState::Idle) {
    logEvent("  a serial port is already open, this one stays unused");
    usb_host_device_close(s_client, dev);
    return;
  }
  err = usb_host_interface_claim(s_client, dev, port.dataIntf, 0);
  if (err != ESP_OK) {
    logEvent("  interface %u could not be claimed: %s", port.dataIntf, esp_err_to_name(err));
    usb_host_device_close(s_client, dev);
    pushEvent(UsbLinkEvent::NoSerialPort);
    return;
  }
  s_dev = dev;
  s_port = port;
  s_vid = dd ? dd->idVendor : 0;
  s_pid = dd ? dd->idProduct : 0;
  strlcpy(s_product, product, sizeof(s_product));
  s_configStep = 0;
  s_state = PortState::Configuring;
  logEvent("  serial port: interface %u, IN %02X, OUT %02X, control interface %d", port.dataIntf, port.epIn,
           port.epOut, port.commIntf == kNoInterface ? -1 : (int)port.commIntf);
}

void beginClose() {
  if (s_state == PortState::Idle || s_state == PortState::Closing) return;
  s_state = PortState::Closing;
  s_closeFailureLogged = false;
  // Ends the transfers still out; their callbacks clear the busy flags.
  for (uint8_t ep : {s_port.epIn, s_port.epOut}) {
    (void)usb_host_endpoint_halt(s_dev, ep);
    (void)usb_host_endpoint_flush(s_dev, ep);
  }
}

void onClientEvent(const usb_host_client_event_msg_t* msg, void*) {
  if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
    s_enumPending = false;
    if (s_pendingCount < kMaxPendingDevices) {
      s_pendingAddrs[s_pendingCount++] = msg->new_dev.address;
    } else {
      logEvent("device %u ignored: too many new devices at once", msg->new_dev.address);
    }
  } else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
    if (msg->dev_gone.dev_hdl == s_dev) {
      logEvent("radio gone");
      beginClose();
    }
  }
}

// New devices wait while a closing port finishes, so a quickly replugged radio is not missed.
void serviceNewDevices() {
  if (s_state == PortState::Closing) return;
  for (size_t i = 0; i < s_pendingCount; ++i) inspectDevice(s_pendingAddrs[i]);
  s_pendingCount = 0;
}

void serviceConfiguring() {
  if (s_state != PortState::Configuring || s_ctrlBusy) return;
  const bool hasControl = s_port.commIntf != kNoInterface;
  if (hasControl && s_configStep == 0) {
    const uint8_t coding[7] = {(uint8_t)kLineCodingBaud, (uint8_t)(kLineCodingBaud >> 8),
                               (uint8_t)(kLineCodingBaud >> 16), (uint8_t)(kLineCodingBaud >> 24),
                               0 /* 1 stop bit */, 0 /* no parity */, 8 /* data bits */};
    s_configStep = 1;
    submitClassRequest(kCdcSetLineCoding, 0, coding, sizeof(coding));
    return;
  }
  if (hasControl && s_configStep == 1) {
    s_configStep = 2;
    submitClassRequest(kCdcSetControlLineState, kControlLineDtr, nullptr, 0);
    return;
  }
  s_state = PortState::Ready;
  // The main loop may not yet have taken what the last device sent (it drops it at SerialGone):
  // the read waits for room then, as in onInDone.
  s_inParked = !rxHasRoom();
  if (!s_inParked) submitIn();
  logEvent("serial port open");
  pushEvent(UsbLinkEvent::SerialReady);
}

// A device that answered but never came as a new device: the library gave up on it, e.g. on a
// descriptor it could not take. Its E lines are on Serial0.
void serviceEnumTimeout() {
  if (!s_enumPending || millis() - s_enumSinceMs < kEnumTimeoutMs) return;
  s_enumPending = false;
  logEvent("device %04X:%04X was not set up: enumeration failed", s_enumVid, s_enumPid);
  pushEvent(UsbLinkEvent::NoSerialPort);
}

// Restarts the read that waited for the main loop to take the bytes before it.
void serviceRx() {
  if (s_state != PortState::Ready || !s_inParked || s_inBusy || !rxHasRoom()) return;
  s_inParked = false;
  submitIn();
}

void serviceTx() {
  if (s_state != PortState::Ready || s_outBusy) return;
  // Busy before the bytes leave the buffer, so usbHostFlushOutput() never sees neither.
  s_outBusy = true;
  const size_t n = xStreamBufferReceive(s_tx, s_outXfer->data_buffer, kBulkTransferBytes, 0);
  if (n == 0) {
    s_outBusy = false;
    return;
  }
  s_outXfer->device_handle = s_dev;
  s_outXfer->bEndpointAddress = s_port.epOut;
  s_outXfer->num_bytes = n;
  const esp_err_t err = usb_host_transfer_submit(s_outXfer);
  if (err != ESP_OK) {
    s_outBusy = false;
    logEvent("write could not start: %s", esp_err_to_name(err));
    return;
  }
  s_txBytes += n;
}

void serviceClosing() {
  if (s_state != PortState::Closing || s_inBusy || s_outBusy || s_ctrlBusy) return;
  (void)usb_host_interface_release(s_client, s_dev, s_port.dataIntf);
  const esp_err_t err = usb_host_device_close(s_client, s_dev);
  if (err != ESP_OK) {
    if (!s_closeFailureLogged) logEvent("device could not be closed yet: %s", esp_err_to_name(err));
    s_closeFailureLogged = true;
    return;
  }
  s_dev = nullptr;
  s_inParked = false;
  uint8_t sink[32];
  while (xStreamBufferReceive(s_tx, sink, sizeof(sink), 0) > 0) {
  }
  s_state = PortState::Idle;
  logEvent("serial port closed");
  pushEvent(UsbLinkEvent::SerialGone);
}

void libTask(void*) {
  for (;;) {
    uint32_t flags = 0;
    (void)usb_host_lib_handle_events(portMAX_DELAY, &flags);
  }
}

void clientTask(void*) {
  for (;;) {
    (void)usb_host_client_handle_events(s_client, pdMS_TO_TICKS(10));
    serviceNewDevices();
    serviceEnumTimeout();
    serviceConfiguring();
    serviceRx();
    serviceTx();
    serviceClosing();
  }
}

// For USB?, while the host runs: the root port, the PHY routing and the host channels as the
// registers have them. D+=1 with no device seen means the device is there but the USB Host Library
// did not enumerate it; D+=0 means the device does not show itself (a bus-powered one without 5 V
// on the socket).
void printPortRegisters(Print& out) {
  usb_dwc_hprt_reg_t hprt;
  hprt.val = USB_DWC.hprt_reg.val;
  usb_dwc_gotgctl_reg_t otg;
  otg.val = USB_DWC.gotgctl_reg.val;
  usb_wrap_otg_conf_reg_t wrap;
  wrap.val = USB_WRAP.otg_conf.val;
  const uint32_t rtcUsbConf = RTCCNTL.usb_conf.val;
  const bool swSelect = RTCCNTL.usb_conf.sw_hw_usb_phy_sel;
  const bool phyToHost = RTCCNTL.usb_conf.sw_usb_phy_sel && !wrap.phy_sel;
  out.printf("  port: connected=%u enabled=%u power=%u D+=%u D-=%u speed=%u (HPRT %08lX)\n", hprt.prtconnsts,
             hprt.prtena, hprt.prtpwr, hprt.prtlnsts & 1, (hprt.prtlnsts >> 1) & 1, hprt.prtspd,
             (unsigned long)hprt.val);
  out.printf("  phy: internal PHY on the %s (otg_conf %08lX, rtc usb_conf %08lX)\n",
             !swSelect ? "console, by eFuse/default" : phyToHost ? "USB host" : "console",
             (unsigned long)wrap.val, (unsigned long)rtcUsbConf);
  out.printf("  otg: pad enable=%u, pull override=%u D+ down=%u D- down=%u, A-valid override=%u/%u, "
             "VBUS-valid override=%u/%u (GOTGCTL %08lX)\n",
             wrap.pad_enable, wrap.pad_pull_override, wrap.dp_pulldown, wrap.dm_pulldown, otg.avalidoven,
             otg.avalidovval, otg.vbvalidoven, otg.vbvalidovval, (unsigned long)otg.val);
  // Every host channel the stack has used: the last request's device address, endpoint and what is
  // left of it. chena=1 means the transfer is still running, e.g. a device that keeps NAKing.
  for (int ch = 0; ch < 8; ++ch) {
    usb_dwc_hcchar_reg_t hcchar;
    hcchar.val = USB_DWC.host_chans[ch].hcchar_reg.val;
    if (!hcchar.val) continue;
    usb_dwc_hctsiz_reg_t hctsiz;
    hctsiz.val = USB_DWC.host_chans[ch].hctsiz_reg.val;
    const uint32_t hcint = USB_DWC.host_chans[ch].hcint_reg.val;
    out.printf("  channel %d: addr=%u ep=%u %s type=%u mps=%u chena=%u | left %u bytes %u packets pid=%u | "
               "HCINT %08lX\n",
               ch, hcchar.devaddr, hcchar.epnum, hcchar.epdir ? "IN" : "OUT", hcchar.eptype, hcchar.mps,
               hcchar.chena, hctsiz.xfersize, hctsiz.pktcnt, hctsiz.pid, (unsigned long)hcint);
  }
  out.printf("  frame number: %lu\n", (unsigned long)(USB_DWC.hfnum_reg.val & 0xFFFF));
  uint8_t addrs[8];
  int count = 0;
  if (usb_host_device_addr_list_fill(sizeof(addrs), addrs, &count) == ESP_OK) {
    out.printf("  devices known to the USB library: %d", count);
    for (int i = 0; i < count; ++i) out.printf(" %u", addrs[i]);
    out.println();
  }
}

// The core is built with CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK, and with that the USB Host
// Library cancels every enumeration when no filter is set, with only a warning that the core does
// not print. Takes every device in its first configuration. Runs once per device that answered,
// before its configuration is read, so it counts the devices seen and starts the enumeration watch.
#if CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK
bool acceptEveryDevice(const usb_device_desc_t* dd, uint8_t* bConfigurationValue) {
  *bConfigurationValue = 1;
  ++s_devicesSeen;
  s_enumVid = dd->idVendor;
  s_enumPid = dd->idProduct;
  s_enumSinceMs = millis();
  s_enumPending = true;
  logEvent("device %04X:%04X answered, reading its configuration", dd->idVendor, dd->idProduct);
  return true;
}
#endif

bool allocTransfer(size_t bytes, usb_transfer_cb_t callback, usb_transfer_t** out) {
  if (usb_host_transfer_alloc(bytes, 0, out) != ESP_OK) return false;
  (*out)->callback = callback;
  (*out)->context = nullptr;
  return true;
}

}  // namespace

bool usbHostStart() {
  if (s_started) return true;
  if (!s_logLock) {
    s_logLock = xSemaphoreCreateMutex();
    s_events = xQueueCreate(8, sizeof(UsbLinkEvent));
    s_rx = xStreamBufferCreate(kRxBufferBytes, 1);
    s_tx = xStreamBufferCreate(kTxBufferBytes, 1);
  }
  if (s_holdOnce) {
    s_holdOnce = false;
    s_held = true;
    logEvent("host held off after a crash restart; pick the profile again to start it");
    pushEvent(UsbLinkEvent::Held);
    return false;
  }

  Serial.println("[USB] The native USB socket becomes the radio's USB host. This console stops;");
  Serial.println("[USB] the UART socket (Serial0) answers USB? and HAMTRC? until the next restart.");
  delay(50);
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
  // Without a PC on the socket, console writes would wait out the timeout; drop them at once.
  Serial.setTxTimeoutMs(0);
#endif

  usb_host_config_t hostConfig = {};
  hostConfig.intr_flags = ESP_INTR_FLAG_LEVEL1;
#if CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK
  hostConfig.enum_filter_cb = acceptEveryDevice;
#endif
  esp_err_t err = usb_host_install(&hostConfig);
  if (err != ESP_OK) {
    s_startError = err;
    logEvent("host could not start: %s", esp_err_to_name(err));
    return false;
  }
  if (!allocTransfer(kBulkTransferBytes, onInDone, &s_inXfer) ||
      !allocTransfer(kBulkTransferBytes, onOutDone, &s_outXfer) ||
      !allocTransfer(kControlTransferBytes, onControlDone, &s_ctrlXfer)) {
    s_startError = ESP_ERR_NO_MEM;
    logEvent("host could not start: no memory for transfers");
    return false;
  }
  usb_host_client_config_t clientConfig = {};
  clientConfig.is_synchronous = false;
  clientConfig.max_num_event_msg = 8;
  clientConfig.async.client_event_callback = onClientEvent;
  clientConfig.async.callback_arg = nullptr;
  err = usb_host_client_register(&clientConfig, &s_client);
  if (err != ESP_OK) {
    s_startError = err;
    logEvent("host client could not register: %s", esp_err_to_name(err));
    return false;
  }
  xTaskCreatePinnedToCore(libTask, "usbLib", 4096, nullptr, 5, nullptr, 0);
  xTaskCreatePinnedToCore(clientTask, "usbClient", 4096, nullptr, 4, nullptr, 0);
  s_started = true;
  logEvent("host started, waiting for the radio");
  return true;
}

bool usbHostStarted() { return s_started; }

void usbHostHoldAfterCrash() { s_holdOnce = true; }

bool usbHostSerialReady() { return s_state == PortState::Ready; }

size_t usbHostAvailable() {
  return s_rx ? xStreamBufferBytesAvailable(s_rx) : 0;
}

int usbHostRead() {
  uint8_t b = 0;
  if (s_rx && xStreamBufferReceive(s_rx, &b, 1, 0) == 1) return b;
  return -1;
}

size_t usbHostReadBytes(uint8_t* buf, size_t len) {
  return s_rx && len ? xStreamBufferReceive(s_rx, buf, len, 0) : 0;
}

size_t usbHostWrite(const uint8_t* data, size_t len) {
  if (!s_started || s_state != PortState::Ready || !len) return 0;
  const size_t n = xStreamBufferSend(s_tx, data, len, pdMS_TO_TICKS(50));
  (void)usb_host_client_unblock(s_client);
  return n;
}

void usbHostFlushOutput(uint32_t timeoutMs) {
  if (!s_started) return;
  const uint32_t startMs = millis();
  while ((!xStreamBufferIsEmpty(s_tx) || s_outBusy) && s_state == PortState::Ready) {
    if (millis() - startMs >= timeoutMs) return;
    delay(1);
  }
}

void usbHostFlushInput() {
  if (!s_rx) return;
  uint8_t sink[32];
  while (xStreamBufferReceive(s_rx, sink, sizeof(sink), 0) > 0) {
  }
}

UsbLinkEvent usbHostTakeEvent() {
  UsbLinkEvent event = UsbLinkEvent::None;
  if (s_events && xQueueReceive(s_events, &event, 0) == pdTRUE) return event;
  return UsbLinkEvent::None;
}

UsbHostSummary usbHostSummary() {
  return {s_started, s_held || s_startError != ESP_OK, s_state == PortState::Ready, s_devicesSeen, s_txBytes,
          s_rxBytes};
}

void usbHostPrintStatus(Print& out) {
  out.print("USB host: ");
  if (s_started) {
    out.println("on");
  } else if (s_held) {
    out.println("held off after a crash restart");
  } else if (s_startError != ESP_OK) {
    out.print("failed, ");
    out.println(esp_err_to_name(s_startError));
  } else {
    out.println("off (no USB profile active since the restart)");
  }
  const PortState state = s_state;
  out.print("  serial port: ");
  out.println(stateName(state));
  if (state != PortState::Idle) {
    out.printf("  device: %04X:%04X \"%s\"\n", s_vid, s_pid, s_product);
  }
  out.printf("  bytes: rx=%lu tx=%lu  devices seen: %lu\n", (unsigned long)s_rxBytes, (unsigned long)s_txBytes,
             (unsigned long)s_devicesSeen);
  if (s_started) printPortRegisters(out);
  if (!s_logLock || xSemaphoreTake(s_logLock, pdMS_TO_TICKS(100)) != pdTRUE) return;
  char lines[kLogLines][kLogLineChars];
  const size_t count = s_logCount;
  const size_t first = (s_logNext + kLogLines - count) % kLogLines;
  for (size_t i = 0; i < count; ++i) memcpy(lines[i], s_log[(first + i) % kLogLines], kLogLineChars);
  xSemaphoreGive(s_logLock);
  out.println("  events:");
  for (size_t i = 0; i < count; ++i) {
    out.print("    ");
    out.println(lines[i]);
  }
}
