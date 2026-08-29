/**
 * Copyright 2013-2023 Software Radio Systems Limited
 *
 * This file is part of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include <libbladeRF.h>
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "rf_blade_imp.h"
#include "rf_plugin.h"
#include "srsran/phy/common/timestamp.h"
#include "srsran/phy/utils/debug.h"
#include "srsran/phy/utils/vector.h"

#define UNUSED __attribute__((unused))
#define CONVERT_BUFFER_SIZE (240 * 1024)
#define BLADERF_RX_FULL_SCALE 2048.0
#define BLADERF_RX_CLIP_LEVEL 2047
#define BLADERF_TX_FULL_SCALE 2048.0
#define BLADERF_TX_CLIP_LEVEL 2047

typedef struct {
  struct bladerf*     dev;
  bladerf_sample_rate rx_rate;
  bladerf_sample_rate tx_rate;
  int16_t             rx_buffer[CONVERT_BUFFER_SIZE];
  int16_t             tx_buffer[CONVERT_BUFFER_SIZE];
  bool                rx_stream_enabled;
  bool                tx_stream_enabled;
  bladerf_channel     rx_channel;
  bladerf_channel     tx_channel;
  uint32_t            rx_channel_number;
  uint32_t            tx_channel_number;
  bladerf_sample_rate initial_sample_rate;
  bool                rx_monitor_enabled;
  uint64_t            rx_monitor_last_timestamp;
  uint64_t            rx_monitor_overruns;
  bool                tx_monitor_enabled;
  uint64_t            tx_monitor_sample_count;
  uint64_t            tx_monitor_component_count;
  uint64_t            tx_monitor_sum_squares;
  uint64_t            tx_monitor_clipped;
  uint64_t            tx_monitor_underruns;
  uint32_t            tx_monitor_peak_abs;
  bladerf_gain        rx_gain_config;
  bladerf_gain_mode   rx_gain_mode;
  srsran_rf_info_t    info;
} rf_blade_handler_t;

static srsran_rf_error_handler_t blade_error_handler     = NULL;
static void*                     blade_error_handler_arg = NULL;

void rf_blade_suppress_stdout(UNUSED void* h)
{
  bladerf_log_set_verbosity(BLADERF_LOG_LEVEL_SILENT);
}

void rf_blade_register_error_handler(UNUSED void* ptr, srsran_rf_error_handler_t new_handler, void* arg)
{
  blade_error_handler     = new_handler;
  blade_error_handler_arg = arg;
}

const unsigned int num_buffers       = 256;
const unsigned int ms_buffer_size_rx = 1024;
const unsigned int buffer_size_tx    = 1024;
const unsigned int num_transfers     = 32;
const unsigned int timeout_ms        = 4000;

static char* trim_whitespace(char* value)
{
  while (*value == ' ' || *value == '\t') {
    value++;
  }

  char* end = value + strlen(value);
  while (end > value && (end[-1] == ' ' || end[-1] == '\t')) {
    end--;
  }
  *end = '\0';

  return value;
}

static int parse_channel_number(const char* value, const char* option, uint32_t* channel_number)
{
  char* end    = NULL;
  long  parsed = 0;

  errno  = 0;
  parsed = strtol(value, &end, 10);

  while (end != NULL && (*end == ' ' || *end == '\t')) {
    end++;
  }

  if (errno != 0 || end == value || end == NULL || *end != '\0' || parsed < 1 || parsed > 2) {
    ERROR("Invalid bladeRF %s=%s. Expected physical channel 1 or 2.", option, value);
    return SRSRAN_ERROR;
  }

  *channel_number = (uint32_t)parsed;
  return SRSRAN_SUCCESS;
}

static int parse_bool_option(const char* value, const char* option, bool* parsed_value)
{
  if (strcmp(value, "1") == 0 || strcmp(value, "true") == 0 || strcmp(value, "on") == 0) {
    *parsed_value = true;
    return SRSRAN_SUCCESS;
  }
  if (strcmp(value, "0") == 0 || strcmp(value, "false") == 0 || strcmp(value, "off") == 0) {
    *parsed_value = false;
    return SRSRAN_SUCCESS;
  }

  ERROR("Invalid bladeRF %s=%s. Expected true/false, on/off or 1/0.", option, value);
  return SRSRAN_ERROR;
}

static int parse_sample_rate(const char* value, bladerf_sample_rate* sample_rate)
{
  char*  end    = NULL;
  double parsed = 0.0;

  errno  = 0;
  parsed = strtod(value, &end);
  while (end != NULL && (*end == ' ' || *end == '\t')) {
    end++;
  }
  if (errno != 0 || end == value || end == NULL || *end != '\0' || !isfinite(parsed) || parsed < 520000.0 ||
      parsed > 61440000.0) {
    ERROR("Invalid bladeRF initial_srate=%s. Expected 520000..61440000 samples/s.", value);
    return SRSRAN_ERROR;
  }

  *sample_rate = (bladerf_sample_rate)llround(parsed);
  return SRSRAN_SUCCESS;
}

static int append_device_identifier(char device_identifier[RF_PARAM_LEN], const char* value)
{
  size_t current_len = strlen(device_identifier);
  size_t value_len   = strlen(value);
  size_t separator   = current_len == 0 ? 0 : 1;

  if (current_len + separator + value_len >= RF_PARAM_LEN) {
    ERROR("bladeRF device identifier is too long.");
    return SRSRAN_ERROR;
  }

  if (separator != 0) {
    device_identifier[current_len++] = ',';
  }
  memcpy(device_identifier + current_len, value, value_len + 1);

  return SRSRAN_SUCCESS;
}

static int parse_blade_args(const char* args,
                            uint32_t* tx_channel_number,
                            uint32_t* rx_channel_number,
                            bool* rx_monitor_enabled,
                            bool* tx_monitor_enabled,
                            bladerf_sample_rate* initial_sample_rate,
                            char device_identifier[RF_PARAM_LEN])
{
  *tx_channel_number   = 1;
  *rx_channel_number   = 1;
  *rx_monitor_enabled  = false;
  *tx_monitor_enabled  = false;
  *initial_sample_rate = 1920000;
  device_identifier[0] = '\0';

  if (args == NULL || args[0] == '\0' || strcmp(args, "auto") == 0) {
    return SRSRAN_SUCCESS;
  }

  if (strlen(args) >= RF_PARAM_LEN) {
    ERROR("bladeRF device arguments are too long.");
    return SRSRAN_ERROR;
  }

  char args_copy[RF_PARAM_LEN] = {0};
  memcpy(args_copy, args, strlen(args) + 1);

  char* saveptr = NULL;
  char* token   = strtok_r(args_copy, ",", &saveptr);
  while (token != NULL) {
    char* option = trim_whitespace(token);

    if (strncmp(option, "tx_channel=", strlen("tx_channel=")) == 0) {
      if (parse_channel_number(option + strlen("tx_channel="), "tx_channel", tx_channel_number) !=
          SRSRAN_SUCCESS) {
        return SRSRAN_ERROR;
      }
    } else if (strncmp(option, "rx_channel=", strlen("rx_channel=")) == 0) {
      if (parse_channel_number(option + strlen("rx_channel="), "rx_channel", rx_channel_number) !=
          SRSRAN_SUCCESS) {
        return SRSRAN_ERROR;
      }
    } else if (strncmp(option, "rx_monitor=", strlen("rx_monitor=")) == 0) {
      if (parse_bool_option(option + strlen("rx_monitor="), "rx_monitor", rx_monitor_enabled) != SRSRAN_SUCCESS) {
        return SRSRAN_ERROR;
      }
    } else if (strncmp(option, "tx_monitor=", strlen("tx_monitor=")) == 0) {
      if (parse_bool_option(option + strlen("tx_monitor="), "tx_monitor", tx_monitor_enabled) != SRSRAN_SUCCESS) {
        return SRSRAN_ERROR;
      }
    } else if (strncmp(option, "initial_srate=", strlen("initial_srate=")) == 0) {
      if (parse_sample_rate(option + strlen("initial_srate="), initial_sample_rate) != SRSRAN_SUCCESS) {
        return SRSRAN_ERROR;
      }
    } else if (option[0] != '\0' && strcmp(option, "auto") != 0) {
      if (append_device_identifier(device_identifier, option) != SRSRAN_SUCCESS) {
        return SRSRAN_ERROR;
      }
    }

    token = strtok_r(NULL, ",", &saveptr);
  }

  return SRSRAN_SUCCESS;
}

const char* rf_blade_devname(UNUSED void* h)
{
  return DEVNAME;
}

int rf_blade_start_tx_stream(void* h)
{
  int                 status;
  rf_blade_handler_t* handler = (rf_blade_handler_t*)h;

  status = bladerf_sync_config(handler->dev,
                               BLADERF_TX_X1,
                               BLADERF_FORMAT_SC16_Q11_META,
                               num_buffers,
                               buffer_size_tx,
                               num_transfers,
                               timeout_ms);
  if (status != 0) {
    ERROR("Failed to configure TX sync interface: %s", bladerf_strerror(status));
    return status;
  }
  status = bladerf_enable_module(handler->dev, handler->tx_channel, true);
  if (status != 0) {
    ERROR("Failed to enable TX module: %s", bladerf_strerror(status));
    return status;
  }
  handler->tx_stream_enabled = true;
  return 0;
}

int rf_blade_start_rx_stream(void* h, UNUSED bool now)
{
  int                 status;
  rf_blade_handler_t* handler = (rf_blade_handler_t*)h;

  /* Configure the device's RX module for use with the sync interface.
   * SC16 Q11 samples *with* metadata are used. */
  uint32_t buffer_size_rx = ms_buffer_size_rx * (handler->rx_rate / 1000 / 1024);

  status = bladerf_sync_config(handler->dev,
                               BLADERF_RX_X1,
                               BLADERF_FORMAT_SC16_Q11_META,
                               num_buffers,
                               buffer_size_rx,
                               num_transfers,
                               timeout_ms);
  if (status != 0) {
    ERROR("Failed to configure RX sync interface: %s", bladerf_strerror(status));
    return status;
  }
  status = bladerf_sync_config(handler->dev,
                               BLADERF_TX_X1,
                               BLADERF_FORMAT_SC16_Q11_META,
                               num_buffers,
                               buffer_size_tx,
                               num_transfers,
                               timeout_ms);
  if (status != 0) {
    ERROR("Failed to configure TX sync interface: %s", bladerf_strerror(status));
    return status;
  }
  status = bladerf_enable_module(handler->dev, handler->rx_channel, true);
  if (status != 0) {
    ERROR("Failed to enable RX module: %s", bladerf_strerror(status));
    return status;
  }
  status = bladerf_enable_module(handler->dev, handler->tx_channel, true);
  if (status != 0) {
    ERROR("Failed to enable TX module: %s", bladerf_strerror(status));
    return status;
  }
  handler->rx_stream_enabled = true;
  return 0;
}

int rf_blade_stop_rx_stream(void* h)
{
  rf_blade_handler_t* handler = (rf_blade_handler_t*)h;
  int                 status  = bladerf_enable_module(handler->dev, handler->rx_channel, false);
  if (status != 0) {
    ERROR("Failed to enable RX module: %s", bladerf_strerror(status));
    return status;
  }
  status = bladerf_enable_module(handler->dev, handler->tx_channel, false);
  if (status != 0) {
    ERROR("Failed to enable TX module: %s", bladerf_strerror(status));
    return status;
  }
  handler->rx_stream_enabled = false;
  handler->tx_stream_enabled = false;
  return 0;
}

void rf_blade_flush_buffer(UNUSED void* h) {}

bool rf_blade_has_rssi(UNUSED void* h)
{
  return false;
}

float rf_blade_get_rssi(UNUSED void* h)
{
  return 0;
}

int rf_blade_open_multi(char* args, void** h, UNUSED uint32_t nof_channels)
{
  return rf_blade_open(args, h);
}

int rf_blade_open(char* args, void** h)
{
  const struct bladerf_range* range_tx = NULL;
  const struct bladerf_range* range_rx = NULL;
  char                        device_identifier[RF_PARAM_LEN] = {0};
  uint32_t                    tx_channel_number               = 1;
  uint32_t                    rx_channel_number               = 1;
  bool                        rx_monitor_enabled               = false;
  bool                        tx_monitor_enabled               = false;
  bladerf_sample_rate         initial_sample_rate              = 1920000;
  *h = NULL;

  if (parse_blade_args(args,
                       &tx_channel_number,
                       &rx_channel_number,
                       &rx_monitor_enabled,
                       &tx_monitor_enabled,
                       &initial_sample_rate,
                       device_identifier) != SRSRAN_SUCCESS) {
    return SRSRAN_ERROR;
  }

  rf_blade_handler_t* handler = (rf_blade_handler_t*)malloc(sizeof(rf_blade_handler_t));
  if (!handler) {
    perror("malloc");
    return -1;
  }
  *h = handler;

  handler->dev                 = NULL;
  handler->tx_channel_number   = tx_channel_number;
  handler->initial_sample_rate = initial_sample_rate;
  handler->rx_channel_number   = rx_channel_number;
  handler->tx_channel          = BLADERF_CHANNEL_TX(tx_channel_number - 1);
  handler->rx_channel          = BLADERF_CHANNEL_RX(rx_channel_number - 1);
  handler->rx_monitor_enabled        = rx_monitor_enabled;
  handler->rx_monitor_last_timestamp = 0;
  handler->rx_monitor_overruns       = 0;
  handler->tx_monitor_enabled         = tx_monitor_enabled;
  handler->tx_monitor_sample_count    = 0;
  handler->tx_monitor_component_count = 0;
  handler->tx_monitor_sum_squares     = 0;
  handler->tx_monitor_clipped         = 0;
  handler->tx_monitor_underruns       = 0;
  handler->tx_monitor_peak_abs        = 0;
  handler->rx_gain_config            = 0;
  handler->rx_gain_mode              = BLADERF_GAIN_MGC;

  printf("Opening bladeRF with TX%u/RX%u (initial srate %.3f Msps, RX monitor %s, TX monitor %s)...\n",
         handler->tx_channel_number,
         handler->rx_channel_number,
         (double)handler->initial_sample_rate / 1e6,
         handler->rx_monitor_enabled ? "enabled" : "disabled",
         handler->tx_monitor_enabled ? "enabled" : "disabled");
  const char* identifier = device_identifier[0] == '\0' ? NULL : device_identifier;
  int         status     = bladerf_open(&handler->dev, identifier);
  if (status) {
    ERROR("Unable to open device: %s", bladerf_strerror(status));
    goto clean_exit;
  }

  bladerf_power_sources power_source;
  float                 bus_voltage = 0.0f;
  float                 load_current = 0.0f;
  float                 load_power   = 0.0f;
  int                   source_status;
  int                   voltage_status;
  int                   current_status;
  int                   power_status;

  source_status  = bladerf_get_power_source(handler->dev, &power_source);
  voltage_status = bladerf_get_pmic_register(handler->dev, BLADERF_PMIC_VOLTAGE_BUS, &bus_voltage);
  current_status = bladerf_get_pmic_register(handler->dev, BLADERF_PMIC_CURRENT, &load_current);
  power_status   = bladerf_get_pmic_register(handler->dev, BLADERF_PMIC_POWER, &load_power);
  if (source_status == 0 && voltage_status == 0 && current_status == 0 && power_status == 0) {
    const char* source_name = power_source == BLADERF_PS_DC       ? "DC"
                              : power_source == BLADERF_PS_USB_VBUS ? "USB"
                                                                    : "unknown";
    printf("bladeRF power: source=%s bus=%.3f V current=%.3f A power=%.3f W\n",
           source_name,
           bus_voltage,
           load_current,
           load_power);
  } else {
    printf("bladeRF power telemetry unavailable: source=%s voltage=%s current=%s power=%s\n",
           bladerf_strerror(source_status),
           bladerf_strerror(voltage_status),
           bladerf_strerror(current_status),
           bladerf_strerror(power_status));
  }

  if (handler->tx_channel_number > bladerf_get_channel_count(handler->dev, BLADERF_TX) ||
      handler->rx_channel_number > bladerf_get_channel_count(handler->dev, BLADERF_RX)) {
    ERROR("Selected TX%u/RX%u is unavailable on this bladeRF.",
          handler->tx_channel_number,
          handler->rx_channel_number);
    status = SRSRAN_ERROR;
    goto clean_exit;
  }

  status = bladerf_set_gain_mode(handler->dev, handler->rx_channel, BLADERF_GAIN_MGC);
  if (status) {
    ERROR("Failed to set RX gain mode: %s", bladerf_strerror(status));
    goto clean_exit;
  }

  // bladerf_log_set_verbosity(BLADERF_LOG_LEVEL_VERBOSE);

  /* Get Gain ranges and set Rx to maximum */
  status = bladerf_get_gain_range(handler->dev, handler->rx_channel, &range_rx);
  if ((status != 0) || (range_rx == NULL)) {
    ERROR("Failed to get RX gain range: %s", bladerf_strerror(status));
    goto clean_exit;
  }

  status = bladerf_get_gain_range(handler->dev, handler->tx_channel, &range_tx);
  if ((status != 0) || (range_tx == NULL)) {
    ERROR("Failed to get TX gain range: %s", bladerf_strerror(status));
    goto clean_exit;
  }

  printf("bladeRF gain ranges: TX%u=%.1f..%.1f dB step %.1f; RX%u=%.1f..%.1f dB step %.1f\n",
         handler->tx_channel_number,
         (double)range_tx->min * range_tx->scale,
         (double)range_tx->max * range_tx->scale,
         (double)range_tx->step * range_tx->scale,
         handler->rx_channel_number,
         (double)range_rx->min * range_rx->scale,
         (double)range_rx->max * range_rx->scale,
         (double)range_rx->step * range_rx->scale);

  status = bladerf_set_gain(handler->dev, handler->rx_channel, (bladerf_gain)range_rx->max);
  if (status != 0) {
    ERROR("Failed to set RX LNA gain: %s", bladerf_strerror(status));
    goto clean_exit;
  }
  handler->rx_gain_config = (bladerf_gain)range_rx->max;
  handler->rx_stream_enabled = false;
  handler->tx_stream_enabled = false;

  /* Set default sampling rates */
  rf_blade_set_tx_srate(handler, handler->initial_sample_rate);
  rf_blade_set_rx_srate(handler, handler->initial_sample_rate);

  /* Set info structure */
  handler->info.min_tx_gain = range_tx->min;
  handler->info.max_tx_gain = range_tx->max;
  handler->info.min_rx_gain = range_rx->min;
  handler->info.max_rx_gain = range_rx->max;

  return SRSRAN_SUCCESS;

clean_exit:
  if (handler->dev != NULL) {
    bladerf_close(handler->dev);
  }
  free(handler);
  *h = NULL;
  return status;
}

int rf_blade_close(void* h)
{
  rf_blade_handler_t* handler = (rf_blade_handler_t*)h;
  bladerf_close(handler->dev);
  return 0;
}

double rf_blade_set_rx_srate(void* h, double freq)
{
  uint32_t            bw;
  bladerf_sample_rate requested_rate = (bladerf_sample_rate)llround(freq);
  rf_blade_handler_t* handler = (rf_blade_handler_t*)h;
  int                 status  = bladerf_set_sample_rate(handler->dev, handler->rx_channel, requested_rate, &handler->rx_rate);
  if (status != 0) {
    ERROR("Failed to set samplerate = %u: %s", (uint32_t)freq, bladerf_strerror(status));
    return -1;
  }
  if (handler->rx_rate < 2000000) {
    status = bladerf_set_bandwidth(handler->dev, handler->rx_channel, handler->rx_rate, &bw);
    if (status != 0) {
      ERROR("Failed to set bandwidth = %u: %s", handler->rx_rate, bladerf_strerror(status));
      return -1;
    }
  } else {
    status =
        bladerf_set_bandwidth(handler->dev, handler->rx_channel, (bladerf_bandwidth)(handler->rx_rate * 0.8), &bw);
    if (status != 0) {
      ERROR("Failed to set bandwidth = %u: %s", handler->rx_rate, bladerf_strerror(status));
      return -1;
    }
  }
  const double error_ppm = 1e6 * ((double)handler->rx_rate - requested_rate) / requested_rate;
  printf("Set RX sampling rate: requested=%u actual=%u Hz error=%+.3f ppm, filter BW=%u Hz\n",
         requested_rate,
         handler->rx_rate,
         error_ppm,
         bw);
  return (double)handler->rx_rate;
}

double rf_blade_set_tx_srate(void* h, double freq)
{
  uint32_t            bw;
  bladerf_sample_rate requested_rate = (bladerf_sample_rate)llround(freq);
  rf_blade_handler_t* handler = (rf_blade_handler_t*)h;
  int                 status  = bladerf_set_sample_rate(handler->dev, handler->tx_channel, requested_rate, &handler->tx_rate);
  if (status != 0) {
    ERROR("Failed to set samplerate = %u: %s", (uint32_t)freq, bladerf_strerror(status));
    return -1;
  }
  status = bladerf_set_bandwidth(handler->dev, handler->tx_channel, handler->tx_rate, &bw);
  if (status != 0) {
    ERROR("Failed to set bandwidth = %u: %s", handler->tx_rate, bladerf_strerror(status));
    return -1;
  }
  const double error_ppm = 1e6 * ((double)handler->tx_rate - requested_rate) / requested_rate;
  printf("Set TX sampling rate: requested=%u actual=%u Hz error=%+.3f ppm, filter BW=%u Hz\n",
         requested_rate,
         handler->tx_rate,
         error_ppm,
         bw);
  return (double)handler->tx_rate;
}

int rf_blade_set_rx_gain(void* h, double gain)
{
  int                 status;
  rf_blade_handler_t* handler = (rf_blade_handler_t*)h;

  if (gain < 0) {
    status = bladerf_set_gain_mode(handler->dev, handler->rx_channel, BLADERF_GAIN_DEFAULT);
    if (status != 0) {
      ERROR("Failed to enable RX AGC: %s", bladerf_strerror(status));
      return SRSRAN_ERROR;
    }
    handler->rx_gain_config = -1;
    handler->rx_gain_mode   = BLADERF_GAIN_DEFAULT;
    return SRSRAN_SUCCESS;
  }

  status = bladerf_set_gain_mode(handler->dev, handler->rx_channel, BLADERF_GAIN_MGC);
  if (status != 0) {
    ERROR("Failed to enable manual RX gain control: %s", bladerf_strerror(status));
    return SRSRAN_ERROR;
  }

  status                      = bladerf_set_gain(handler->dev, handler->rx_channel, (bladerf_gain)gain);
  if (status != 0) {
    ERROR("Failed to set RX gain: %s", bladerf_strerror(status));
    return SRSRAN_ERROR;
  }
  handler->rx_gain_config = (bladerf_gain)gain;
  handler->rx_gain_mode   = BLADERF_GAIN_MGC;
  return SRSRAN_SUCCESS;
}

int rf_blade_set_rx_gain_ch(void* h, UNUSED uint32_t ch, double gain)
{
  return rf_blade_set_rx_gain(h, gain);
}

int rf_blade_set_tx_gain(void* h, double gain)
{
  int                 status;
  bladerf_gain        actual_gain;
  rf_blade_handler_t* handler = (rf_blade_handler_t*)h;
  status                      = bladerf_set_gain(handler->dev, handler->tx_channel, (bladerf_gain)gain);
  if (status != 0) {
    ERROR("Failed to set TX gain: %s", bladerf_strerror(status));
    return SRSRAN_ERROR;
  }
  status = bladerf_get_gain(handler->dev, handler->tx_channel, &actual_gain);
  if (status != 0) {
    ERROR("Failed to read back TX gain: %s", bladerf_strerror(status));
    return SRSRAN_ERROR;
  }
  printf("Set TX%u gain: requested=%.1f dB actual=%d dB\n", handler->tx_channel_number, gain, actual_gain);
  return SRSRAN_SUCCESS;
}

int rf_blade_set_tx_gain_ch(void* h, UNUSED uint32_t ch, double gain)
{
  return rf_blade_set_tx_gain(h, gain);
}

double rf_blade_get_rx_gain(void* h)
{
  int                 status;
  bladerf_gain        gain    = 0;
  rf_blade_handler_t* handler = (rf_blade_handler_t*)h;
  status                      = bladerf_get_gain(handler->dev, handler->rx_channel, &gain);
  if (status != 0) {
    ERROR("Failed to get RX gain: %s", bladerf_strerror(status));
    return -1;
  }
  return gain;
}

double rf_blade_get_tx_gain(void* h)
{
  int                 status;
  bladerf_gain        gain    = 0;
  rf_blade_handler_t* handler = (rf_blade_handler_t*)h;
  status                      = bladerf_get_gain(handler->dev, handler->tx_channel, &gain);
  if (status != 0) {
    ERROR("Failed to get TX gain: %s", bladerf_strerror(status));
    return -1;
  }
  return gain;
}

srsran_rf_info_t* rf_blade_get_info(void* h)
{
  srsran_rf_info_t* info = NULL;

  if (h) {
    rf_blade_handler_t* handler = (rf_blade_handler_t*)h;

    info = &handler->info;
  }
  return info;
}

double rf_blade_set_rx_freq(void* h, UNUSED uint32_t ch, double freq)
{
  rf_blade_handler_t* handler = (rf_blade_handler_t*)h;
  bladerf_frequency   f_int   = (uint32_t)round(freq);
  int                 status  = bladerf_set_frequency(handler->dev, handler->rx_channel, f_int);
  if (status != 0) {
    ERROR("Failed to set samplerate = %u: %s", (uint32_t)freq, bladerf_strerror(status));
    return -1;
  }
  f_int = 0;
  bladerf_get_frequency(handler->dev, handler->rx_channel, &f_int);
  printf("set RX frequency to %" PRIu64 "\n", (uint64_t)f_int);

  return freq;
}

double rf_blade_set_tx_freq(void* h, UNUSED uint32_t ch, double freq)
{
  rf_blade_handler_t* handler = (rf_blade_handler_t*)h;
  bladerf_frequency   f_int   = (uint32_t)round(freq);
  int                 status  = bladerf_set_frequency(handler->dev, handler->tx_channel, f_int);
  if (status != 0) {
    ERROR("Failed to set samplerate = %u: %s", (uint32_t)freq, bladerf_strerror(status));
    return -1;
  }

  f_int = 0;
  bladerf_get_frequency(handler->dev, handler->tx_channel, &f_int);
  printf("set TX frequency to %" PRIu64 "\n", (uint64_t)f_int);
  return freq;
}

static void timestamp_to_secs(uint32_t rate, uint64_t timestamp, time_t* secs, double* frac_secs)
{
  double totalsecs = (double)timestamp / rate;
  time_t secs_i    = (time_t)totalsecs;
  if (secs) {
    *secs = secs_i;
  }
  if (frac_secs) {
    *frac_secs = totalsecs - secs_i;
  }
}

void rf_blade_get_time(void* h, time_t* secs, double* frac_secs)
{
  rf_blade_handler_t*     handler = (rf_blade_handler_t*)h;
  struct bladerf_metadata meta;

  int status = bladerf_get_timestamp(handler->dev, BLADERF_RX, &meta.timestamp);
  if (status != 0) {
    ERROR("Failed to get current RX timestamp: %s", bladerf_strerror(status));
  }
  timestamp_to_secs(handler->rx_rate, meta.timestamp, secs, frac_secs);
}

static void rf_blade_monitor_rx(rf_blade_handler_t* handler, const struct bladerf_metadata* meta, uint32_t nsamples)
{
  if (!handler->rx_monitor_enabled || handler->rx_rate == 0 || nsamples == 0) {
    return;
  }

  if (handler->rx_monitor_last_timestamp != 0 &&
      meta->timestamp - handler->rx_monitor_last_timestamp < handler->rx_rate) {
    return;
  }
  handler->rx_monitor_last_timestamp = meta->timestamp;

  const uint32_t component_count = 2 * nsamples;
  uint32_t       peak_abs        = 0;
  uint32_t       clipped         = 0;
  uint64_t       sum_squares     = 0;

  for (uint32_t i = 0; i != component_count; ++i) {
    const int32_t sample     = handler->rx_buffer[i];
    const uint32_t sample_abs = sample < 0 ? (uint32_t)(-sample) : (uint32_t)sample;
    peak_abs                 = sample_abs > peak_abs ? sample_abs : peak_abs;
    sum_squares += (uint64_t)((int64_t)sample * sample);
    clipped += sample_abs >= BLADERF_RX_CLIP_LEVEL;
  }

  const double rms         = sqrt((double)sum_squares / component_count);
  const double peak_dbfs   = 20.0 * log10(fmax((double)peak_abs / BLADERF_RX_FULL_SCALE, 1e-12));
  const double rms_dbfs    = 20.0 * log10(fmax(rms / BLADERF_RX_FULL_SCALE, 1e-12));
  const double clipped_pct = 100.0 * clipped / component_count;

  printf("bladeRF RX monitor: RX%u gain_cfg=%d mode=%d peak=%.2f dBFS rms=%.2f dBFS "
         "clipped=%u/%u (%.6f%%) overruns=%" PRIu64 "\n",
         handler->rx_channel_number,
         (int)handler->rx_gain_config,
         (int)handler->rx_gain_mode,
         peak_dbfs,
         rms_dbfs,
         clipped,
         component_count,
         clipped_pct,
         handler->rx_monitor_overruns);
  fflush(stdout);
}

int rf_blade_recv_with_time_multi(void*    h,
                                  void**   data,
                                  uint32_t nsamples,
                                  bool     blocking,
                                  time_t*  secs,
                                  double*  frac_secs)
{
  return rf_blade_recv_with_time(h, *data, nsamples, blocking, secs, frac_secs);
}

int rf_blade_recv_with_time(void*       h,
                            void*       data,
                            uint32_t    nsamples,
                            UNUSED bool blocking,
                            time_t*     secs,
                            double*     frac_secs)
{
  rf_blade_handler_t*     handler = (rf_blade_handler_t*)h;
  struct bladerf_metadata meta;
  int                     status;

  memset(&meta, 0, sizeof(meta));
  meta.flags = BLADERF_META_FLAG_RX_NOW;

  if (2 * nsamples > CONVERT_BUFFER_SIZE) {
    ERROR("RX failed: nsamples exceeds buffer size (%d>%d)", nsamples, CONVERT_BUFFER_SIZE);
    return -1;
  }
  status = bladerf_sync_rx(handler->dev, handler->rx_buffer, nsamples, &meta, 2000);
  if (status) {
    ERROR("RX failed: %s; nsamples=%d;", bladerf_strerror(status), nsamples);
    return -1;
  } else if (meta.status & BLADERF_META_STATUS_OVERRUN) {
    handler->rx_monitor_overruns++;
    if (blade_error_handler) {
      srsran_rf_error_t error;
      error.opt  = meta.actual_count;
      error.type = SRSRAN_RF_ERROR_OVERFLOW;
      blade_error_handler(blade_error_handler_arg, error);
    } else {
      /*ERROR("Overrun detected in scheduled RX. "
            "%u valid samples were read.", meta.actual_count);*/
    }
  }

  rf_blade_monitor_rx(handler, &meta, nsamples);
  timestamp_to_secs(handler->rx_rate, meta.timestamp, secs, frac_secs);
  srsran_vec_convert_if(handler->rx_buffer, 2048, data, 2 * nsamples);

  return nsamples;
}

int rf_blade_send_timed_multi(void*  h,
                              void*  data[4],
                              int    nsamples,
                              time_t secs,
                              double frac_secs,
                              bool   has_time_spec,
                              bool   blocking,
                              bool   is_start_of_burst,
                              bool   is_end_of_burst)
{
  return rf_blade_send_timed(
      h, data[0], nsamples, secs, frac_secs, has_time_spec, blocking, is_start_of_burst, is_end_of_burst);
}

static void rf_blade_monitor_tx(rf_blade_handler_t* handler, int nsamples)
{
  if (!handler->tx_monitor_enabled || handler->tx_rate == 0 || nsamples <= 0) {
    return;
  }

  const uint32_t component_count = 2U * (uint32_t)nsamples;
  for (uint32_t i = 0; i != component_count; ++i) {
    const int32_t  sample     = handler->tx_buffer[i];
    const uint32_t sample_abs = sample < 0 ? (uint32_t)(-sample) : (uint32_t)sample;
    handler->tx_monitor_peak_abs = sample_abs > handler->tx_monitor_peak_abs ? sample_abs
                                                                              : handler->tx_monitor_peak_abs;
    handler->tx_monitor_sum_squares += (uint64_t)((int64_t)sample * sample);
    handler->tx_monitor_clipped += sample_abs >= BLADERF_TX_CLIP_LEVEL;
  }
  handler->tx_monitor_sample_count += (uint64_t)nsamples;
  handler->tx_monitor_component_count += component_count;

  if (handler->tx_monitor_sample_count < handler->tx_rate) {
    return;
  }

  const double rms = sqrt((double)handler->tx_monitor_sum_squares / handler->tx_monitor_component_count);
  const double peak_dbfs =
      20.0 * log10(fmax((double)handler->tx_monitor_peak_abs / BLADERF_TX_FULL_SCALE, 1e-12));
  const double rms_dbfs = 20.0 * log10(fmax(rms / BLADERF_TX_FULL_SCALE, 1e-12));
  const double clipped_pct = 100.0 * handler->tx_monitor_clipped / handler->tx_monitor_component_count;

  printf("bladeRF TX monitor: TX%u peak=%.2f dBFS rms=%.2f dBFS clipped=%" PRIu64 "/%" PRIu64
         " (%.6f%%) underruns=%" PRIu64 "\n",
         handler->tx_channel_number,
         peak_dbfs,
         rms_dbfs,
         handler->tx_monitor_clipped,
         handler->tx_monitor_component_count,
         clipped_pct,
         handler->tx_monitor_underruns);
  fflush(stdout);

  handler->tx_monitor_sample_count    = 0;
  handler->tx_monitor_component_count = 0;
  handler->tx_monitor_sum_squares     = 0;
  handler->tx_monitor_clipped         = 0;
  handler->tx_monitor_peak_abs        = 0;
}

int rf_blade_send_timed(void*       h,
                        void*       data,
                        int         nsamples,
                        time_t      secs,
                        double      frac_secs,
                        bool        has_time_spec,
                        UNUSED bool blocking,
                        bool        is_start_of_burst,
                        bool        is_end_of_burst)
{
  rf_blade_handler_t*     handler = (rf_blade_handler_t*)h;
  struct bladerf_metadata meta;
  int                     status;

  if (!handler->tx_stream_enabled) {
    rf_blade_start_tx_stream(h);
  }

  if (2 * nsamples > CONVERT_BUFFER_SIZE) {
    ERROR("TX failed: nsamples exceeds buffer size (%d>%d)", nsamples, CONVERT_BUFFER_SIZE);
    return -1;
  }

  srsran_vec_convert_fi(data, 2048, handler->tx_buffer, 2 * nsamples);

  memset(&meta, 0, sizeof(meta));
  if (is_start_of_burst) {
    if (has_time_spec) {
      // Convert time to ticks
      srsran_timestamp_t ts = {.full_secs = secs, .frac_secs = frac_secs};
      meta.timestamp        = srsran_timestamp_uint64(&ts, handler->tx_rate);
    } else {
      meta.flags |= BLADERF_META_FLAG_TX_NOW;
    }
    meta.flags |= BLADERF_META_FLAG_TX_BURST_START;
  }
  if (is_end_of_burst) {
    meta.flags |= BLADERF_META_FLAG_TX_BURST_END;
  }
  srsran_rf_error_t error;
  bzero(&error, sizeof(srsran_rf_error_t));

  status = bladerf_sync_tx(handler->dev, handler->tx_buffer, nsamples, &meta, 2000);
  if (status == BLADERF_ERR_TIME_PAST) {
    if (blade_error_handler) {
      error.type = SRSRAN_RF_ERROR_LATE;
      blade_error_handler(blade_error_handler_arg, error);
    } else {
      ERROR("TX failed: %s", bladerf_strerror(status));
    }
  } else if (status) {
    ERROR("TX failed: %s", bladerf_strerror(status));
    return status;
  } else if (meta.status == BLADERF_META_STATUS_UNDERRUN) {
    handler->tx_monitor_underruns++;
    if (blade_error_handler) {
      error.type = SRSRAN_RF_ERROR_UNDERFLOW;
      blade_error_handler(blade_error_handler_arg, error);
    } else {
      ERROR("TX warning: underflow detected.");
    }
  }

  rf_blade_monitor_tx(handler, nsamples);

  return nsamples;
}

rf_dev_t srsran_rf_dev_blade = {"bladeRF",
                                rf_blade_devname,
                                rf_blade_start_rx_stream,
                                rf_blade_stop_rx_stream,
                                rf_blade_flush_buffer,
                                rf_blade_has_rssi,
                                rf_blade_get_rssi,
                                rf_blade_suppress_stdout,
                                rf_blade_register_error_handler,
                                rf_blade_open,
                                .srsran_rf_open_multi = rf_blade_open_multi,
                                rf_blade_close,
                                rf_blade_set_rx_srate,
                                rf_blade_set_rx_gain,
                                rf_blade_set_rx_gain_ch,
                                rf_blade_set_tx_gain,
                                rf_blade_set_tx_gain_ch,
                                rf_blade_get_rx_gain,
                                rf_blade_get_tx_gain,
                                rf_blade_get_info,
                                rf_blade_set_rx_freq,
                                rf_blade_set_tx_srate,
                                rf_blade_set_tx_freq,
                                rf_blade_get_time,
                                NULL,
                                rf_blade_recv_with_time,
                                rf_blade_recv_with_time_multi,
                                rf_blade_send_timed,
                                .srsran_rf_send_timed_multi = rf_blade_send_timed_multi};

#ifdef ENABLE_RF_PLUGINS
int register_plugin(rf_dev_t** rf_api)
{
  if (rf_api == NULL) {
    return SRSRAN_ERROR;
  }
  *rf_api = &srsran_rf_dev_blade;
  return SRSRAN_SUCCESS;
}
#endif /* ENABLE_RF_PLUGINS */
