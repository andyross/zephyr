/* Copyright (c) 2022 Intel Corporation.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_INOTICE_H_
#define ZEPHYR_INCLUDE_DRIVERS_INOTICE_H_

#include <device.h>

/* Interrupt Notice Communication API
 *
 * Zephyr's inotice API is designed for interrupt-style synchronous
 * communication with foreign processors.  It's limited to
 * syncronously sent and received messages with just two (potentially
 * partial) dwords of data.  Received messages are handled
 * synchronously in interrupt context.
 */

/** @brief INotice Message Handler Callback
 *
 * This function, once registered via inotice_set_message_handler(),
 * is invoked in interrupt context to service messages sent from the
 * foreign/connected inotice context.
 *
 * Some implementations (e.g. on the other side of the link, but
 * potentially also specialized APIs on this device) allow for
 * asynchronous message processing.  This function should return true
 * if the message has been completely handled and return notification
 * should proceed immediately.  Return false only if your local device
 * is capable and you are going to provide completely notification via
 * a separate API.
 *
 * @param dev INotice device
 * @param arg Registered argument from inotice_set_message_handler()
 * @param data Message data from other side
 * @param ext_data Extended message data
 * @return true if the message is completely handled
 */
typedef bool (*inotice_handler_t)(const struct device *dev, void *arg,
				  uint32_t data, uint32_t ext_data);

typedef void (*inotice_set_message_handler_t)(const struct device *dev,
					      inotice_handler_t fn, void *arg);

typedef bool (*inotice_send_message_t)(const struct device *dev,
				       uint32_t data, uint32_t ext_data);

struct inotice_api {
	inotice_set_message_handler_t set_handler;
	inotice_send_message_t send_msg;
	uint8_t msg_data_bits;
	uint8_t msg_ext_data_bits;
};

/** @brief Register message callback handler
 *
 * This function registers a handler function for received messages.
 *
 * @param dev INotice device
 * @param fn Callback function
 * @param arg Value to pass as the "arg" parameter to the function
 */
static inline void inotice_set_message_handler(const struct device *dev,
					       inotice_handler_t fn, void *arg)
{
	const struct inotice_api *api = dev->api;

	api->set_handler(dev, fn, arg);
}

/** @brief Send an INotice message
 *
 * Sends a message to the other side of an INotice link.  The data and
 * ext_data parameters are passed to the other side of the link allow
 * with the interrupt.  Note that only the bottom bits may be
 * transferred, see inotice_msg_data_bits() and
 * inotice_msg_ext_data_bits().
 *
 * Returns true if the message was sent, false otherwise (e.g. a
 * current message is in progress)
 *
 * @param dev INotice device
 * @param data value to transmit with the message
 * @param ext_data Extended value to transmit with the message
 * @return message successfully transmitted
 */
static inline bool inotice_send_message(const struct device *dev,
					uint32_t data, uint32_t ext_data)
{
	const struct inotice_api *api = dev->api;

	return api->send_msg(dev, data, ext_data);
}

/** @brief Size of message data argument
 *
 * Returns the number of significant low bits in the data parameter of
 * a message.  Values with higher bits set cannot be transmitted on
 * the link and will not be presented as the argument to the handler.
 *
 * @return Bit count
 */
static inline uint8_t inotice_msg_data_bits(const struct device *dev)
{
	const struct inotice_api *api = dev->api;

	return api->msg_data_bits;
}

/** @brief Size of message ext_data argument
 *
 * Returns the number of significant low bits in the ext_data
 * parameter of a message.  Values with higher bits set cannot be
 * transmitted on the link and will not be presented as the argument
 * to the handler.
 *
 * @return Bit count
 */
static inline uint8_t inotice_msg_ext_data_bits(const struct device *dev)
{
	const struct inotice_api *api = dev->api;

	return api->msg_ext_data_bits;
}

/** @brief Global inotice device
 *
 * Most device configurations will have a single staticically-defined
 * INotice device available for foreign communication (e.g. for
 * communicating with the host CPUs from an audio DSP).  See your
 * platform documentation for details.
 */
#define INOTICE_DEV DEVICE_DT_GET_ANY(zephyr_inotice)

#endif /* ZEPHYR_INCLUDE_DRIVERS_INOTICE_H_ */
