#include "usb_serial.h"

#include "board.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/usb/dwc/otg_fs.h>
#include <libopencm3/usb/usbd.h>

#define USB_EP_DATA_OUT 0x01
#define USB_EP_DATA_IN  0x82
#define USB_EP_NOTIFY   0x83
#define USB_PACKET_SIZE 64U
#define USB_RX_LINE_SIZE 128U
#define TX_RING_SIZE 1024U

static const struct usb_device_descriptor device_descriptor = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = USB_CLASS_CDC,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x1209,
    .idProduct = 0xF401,
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor communication_endpoint[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_EP_NOTIFY,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endpoints[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_EP_DATA_OUT,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = USB_PACKET_SIZE,
    .bInterval = 1,
}, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_EP_DATA_IN,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = USB_PACKET_SIZE,
    .bInterval = 1,
}};

static const struct {
    struct usb_cdc_header_descriptor header;
    struct usb_cdc_call_management_descriptor call_mgmt;
    struct usb_cdc_acm_descriptor acm;
    struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdc_functional_descriptors = {
    .header = {
        .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
        .bcdCDC = 0x0110,
    },
    .call_mgmt = {
        .bFunctionLength = sizeof(struct usb_cdc_call_management_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
        .bmCapabilities = 0,
        .bDataInterface = 1,
    },
    .acm = {
        .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_ACM,
        .bmCapabilities = 0,
    },
    .cdc_union = {
        .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_UNION,
        .bControlInterface = 0,
        .bSubordinateInterface0 = 1,
    },
};

static const struct usb_interface_descriptor communication_interface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = USB_CLASS_CDC,
    .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
    .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
    .iInterface = 0,
    .endpoint = communication_endpoint,
    .extra = &cdc_functional_descriptors,
    .extralen = sizeof(cdc_functional_descriptors),
}};

static const struct usb_interface_descriptor data_interface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,
    .endpoint = data_endpoints,
}};

static const struct usb_interface interfaces[] = {{
    .num_altsetting = 1,
    .altsetting = communication_interface,
}, {
    .num_altsetting = 1,
    .altsetting = data_interface,
}};

static const struct usb_config_descriptor config_descriptor = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 2,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 100 / 2,
    .interface = interfaces,
};

static const char *usb_strings[] = {
    "Robot Controller",
    "STM32F401 Robot CDC",
    "F401-ROBOT",
};

static uint8_t control_buffer[128];
static usbd_device *g_usb_device;
static bool g_configured;

static char g_rx_line[USB_RX_LINE_SIZE];
static uint16_t g_rx_line_length;
static bool g_rx_line_ready;

static uint8_t g_tx_ring[TX_RING_SIZE];
static uint16_t g_tx_head;
static uint16_t g_tx_tail;
static bool g_tx_endpoint_ready;
static bool g_tx_zlp_pending;

static uint16_t ring_next(uint16_t value, uint16_t size)
{
    ++value;
    return (value >= size) ? 0U : value;
}

static enum usbd_request_return_codes cdc_control_request(
    usbd_device *usbd_dev,
    struct usb_setup_data *req,
    uint8_t **buf,
    uint16_t *len,
    void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
    (void)usbd_dev;
    (void)buf;
    (void)complete;

    switch (req->bRequest) {
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
        return USBD_REQ_HANDLED;

    case USB_CDC_REQ_SET_LINE_CODING:
        if (*len < sizeof(struct usb_cdc_line_coding)) {
            return USBD_REQ_NOTSUPP;
        }
        return USBD_REQ_HANDLED;

    default:
        return USBD_REQ_NOTSUPP;
    }
}

static void data_rx_callback(usbd_device *usbd_dev, uint8_t ep)
{
    (void)ep;

    uint8_t packet[USB_PACKET_SIZE];
    const int len = usbd_ep_read_packet(usbd_dev, USB_EP_DATA_OUT,
                                        packet, sizeof(packet));

    /*
     * USB callbacks only assemble one complete command line.  Command
     * execution happens later from command_poll(), after usbd_poll() has
     * returned.  This avoids running libc/string parsing from inside the USB
     * stack callback and removes the old byte ring from the receive path.
     */
    for (int i = 0; i < len; ++i) {
        const uint8_t byte = packet[i];

        if (g_rx_line_ready) {
            /* One-line mailbox is full.  Host commands are intentionally
             * serialized, so discard input until main() consumes it. */
            continue;
        }

        if (byte == '\r' || byte == '\n') {
            if (g_rx_line_length != 0U) {
                g_rx_line[g_rx_line_length] = '\0';
                g_rx_line_ready = true;
            }
            continue;
        }

        if (g_rx_line_length < USB_RX_LINE_SIZE - 1U) {
            g_rx_line[g_rx_line_length++] = (char)byte;
        } else {
            /* Drop an overlong command and wait for the next line ending. */
            g_rx_line_length = 0U;
        }
    }
}


static void data_tx_complete_callback(usbd_device *usbd_dev, uint8_t ep)
{
    (void)usbd_dev;
    (void)ep;

    /*
     * The current libopencm3 DWC driver can misinterpret a transient NAK
     * between back-to-back IN writes as a stuck endpoint and flush its FIFO.
     * Only arm the next packet after the previous IN transfer completed.
     */
    g_tx_endpoint_ready = true;
}

static void set_config_callback(usbd_device *usbd_dev, uint16_t wValue)
{
    (void)wValue;

    usbd_ep_setup(usbd_dev, USB_EP_DATA_OUT, USB_ENDPOINT_ATTR_BULK,
                  USB_PACKET_SIZE, data_rx_callback);
    usbd_ep_setup(usbd_dev, USB_EP_DATA_IN, USB_ENDPOINT_ATTR_BULK,
                  USB_PACKET_SIZE, data_tx_complete_callback);
    usbd_ep_setup(usbd_dev, USB_EP_NOTIFY, USB_ENDPOINT_ATTR_INTERRUPT,
                  16, NULL);

    usbd_register_control_callback(
        usbd_dev,
        USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
        cdc_control_request);

    g_tx_endpoint_ready = true;
    g_tx_zlp_pending = false;
    g_configured = true;
}

void usb_serial_init(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_OTGFS);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, USB_DM_PIN | USB_DP_PIN);
    gpio_set_af(GPIOA, GPIO_AF10, USB_DM_PIN | USB_DP_PIN);

    /* PA9 is used by an encoder, so run USB FS without external VBUS sensing. */
    OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS;
    OTG_FS_GCCFG &= ~OTG_GCCFG_VBUSASEN;
    OTG_FS_GCCFG &= ~OTG_GCCFG_VBUSBSEN;

    g_rx_line_length = 0U;
    g_rx_line_ready = false;
    g_tx_head = g_tx_tail = 0;
    g_tx_endpoint_ready = false;
    g_tx_zlp_pending = false;
    g_configured = false;

    g_usb_device = usbd_init(&otgfs_usb_driver,
                             &device_descriptor,
                             &config_descriptor,
                             usb_strings,
                             3,
                             control_buffer,
                             sizeof(control_buffer));
    usbd_register_set_config_callback(g_usb_device, set_config_callback);
}

void usb_serial_poll(void)
{
    if (g_usb_device == NULL) {
        return;
    }

    usbd_poll(g_usb_device);

    if (!g_configured || !g_tx_endpoint_ready) {
        return;
    }

    /*
     * USB bulk transfers are terminated by a short packet.  If the queued
     * stream drained on an exact 64-byte packet, Linux's CDC ACM receive URB
     * may keep that packet buffered waiting for the transfer terminator.
     * Send an explicit zero-length packet after the previous IN transfer has
     * completed.  A ZLP carries no serial data; it only terminates the USB
     * bulk transfer promptly.
     */
    if (g_tx_zlp_pending) {
        (void)usbd_ep_write_packet(g_usb_device, USB_EP_DATA_IN, g_tx_ring, 0);
        g_tx_zlp_pending = false;
        g_tx_endpoint_ready = false;
        return;
    }

    if (g_tx_tail == g_tx_head) {
        return;
    }

    uint8_t packet[USB_PACKET_SIZE];
    uint16_t count = 0;
    uint16_t cursor = g_tx_tail;

    while (cursor != g_tx_head && count < sizeof(packet)) {
        packet[count++] = g_tx_ring[cursor];
        cursor = ring_next(cursor, TX_RING_SIZE);
    }

    const int written = usbd_ep_write_packet(g_usb_device, USB_EP_DATA_IN,
                                              packet, count);
    if (written > 0) {
        for (int i = 0; i < written; ++i) {
            g_tx_tail = ring_next(g_tx_tail, TX_RING_SIZE);
        }

        /*
         * If this full-size packet emptied the software queue, terminate the
         * USB bulk transfer with a ZLP after its completion callback.
         */
        if ((uint16_t)written == USB_PACKET_SIZE && g_tx_tail == g_tx_head) {
            g_tx_zlp_pending = true;
        }

        /* Wait for the IN transfer-complete callback before another write. */
        g_tx_endpoint_ready = false;
    }
}

size_t usb_serial_tx_free(void)
{
    uint16_t used;
    if (g_tx_head >= g_tx_tail) {
        used = (uint16_t)(g_tx_head - g_tx_tail);
    } else {
        used = (uint16_t)(TX_RING_SIZE - (g_tx_tail - g_tx_head));
    }

    /* One slot is deliberately left empty to distinguish full from empty. */
    return (size_t)((TX_RING_SIZE - 1U) - used);
}

bool usb_serial_read_line(char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0U || !g_rx_line_ready) {
        return false;
    }

    size_t i = 0U;
    while (i + 1U < buffer_size && g_rx_line[i] != '\0') {
        buffer[i] = g_rx_line[i];
        ++i;
    }
    buffer[i] = '\0';

    g_rx_line_length = 0U;
    g_rx_line_ready = false;
    return true;
}

void usb_serial_write_n(const char *data, size_t len)
{
    if (data == NULL) {
        return;
    }

    for (size_t i = 0; i < len; ++i) {
        const uint16_t next = ring_next(g_tx_head, TX_RING_SIZE);
        if (next == g_tx_tail) {
            return; /* Non-blocking: drop excess output if TX buffer is full. */
        }
        g_tx_ring[g_tx_head] = (uint8_t)data[i];
        g_tx_head = next;
    }
}

void usb_serial_write(const char *text)
{
    if (text == NULL) {
        return;
    }

    while (*text != '\0') {
        usb_serial_write_n(text, 1);
        ++text;
    }
}

void usb_serial_write_line(const char *text)
{
    usb_serial_write(text);
    usb_serial_write("\r\n");
}

void usb_serial_write_u32(uint32_t value)
{
    char buffer[10];
    unsigned length = 0;

    do {
        buffer[length++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U && length < sizeof(buffer));

    while (length > 0) {
        --length;
        usb_serial_write_n(&buffer[length], 1);
    }
}

void usb_serial_write_i32(int32_t value)
{
    if (value < 0) {
        usb_serial_write("-");
        /* Avoid signed overflow for INT32_MIN. */
        const uint32_t magnitude = (uint32_t)(-(value + 1)) + 1U;
        usb_serial_write_u32(magnitude);
    } else {
        usb_serial_write_u32((uint32_t)value);
    }
}
