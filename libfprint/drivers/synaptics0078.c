/*
 * Synaptics 06cb:0078 (sensor type 0x2449) driver for libfprint
 *
 * Copyright (C) 2026 Raul Dias <raul@dias.com.br>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/*
 * The sensor runs a capture module and a per-unit capture program that the
 * host uploads. Neither ships with libfprint; both are read at open time from
 * a data directory:
 *
 *   directory: $FP_SYNAPTICS0078_DATA_DIR if set and non-empty, otherwise
 *              SYNAPTICS0078_DATA_DIR (meson option synaptics0078_data_dir,
 *              default /var/lib/fprint/synaptics0078). The variable is always
 *              honoured: only root controls fprintd's environment.
 *   module.bin              10500 bytes, sha256 SYNAPTICS0078_MODULE_SHA256
 *   program-<serial>.bin    18869 bytes, the complete 02 command (first byte
 *                           0x02); <serial> is RomInfo bytes 0x12-0x17 with
 *                           each byte's nibbles swapped, as 12 lowercase hex
 *                           digits (it equals the USB serial string)
 *
 * The fingerprint-06cb-0078 setup tool writes these files.
 *
 * Open sends 01 (RomInfo), 19 and 75 (sensor id), then loads both files.
 * Nothing else is written to the sensor until all of those checks pass. The
 * module (06 + module.bin) is uploaded only when RomInfo byte 17 == 0, i.e.
 * no module is loaded; it is then confirmed by a second 01, whose byte 17
 * must be non-zero. When byte 17 != 0 from the start, 06 is never sent.
 *
 * Errors reported by open (FP_DEVICE_ERROR domain):
 *   NOT_SUPPORTED  ROM version is not 6.20, or the 75 reply is not a known
 *                  0x2449 sensor id
 *   GENERAL        a data file is missing or unreadable (the message carries
 *                  the full path and how to provide the file)
 *   DATA_INVALID   module.bin has the wrong size or sha256, or the program
 *                  file has the wrong size or does not start with 0x02
 *   PROTO          a reply is too short or carries a non-zero status
 *                  (0x04aa on 06: module already loaded), or the module upload
 *                  was not confirmed by RomInfo byte 17
 * USB errors (G_USB_DEVICE_ERROR) from the transfers are passed through.
 */

#define FP_COMPONENT "synaptics0078"

#include "drivers_api.h"

#define SYNAPTICS0078_EP_OUT 0x01
#define SYNAPTICS0078_EP_IN 0x81
#define SYNAPTICS0078_INTERFACE 0
#define SYNAPTICS0078_TIMEOUT 5000
#define SYNAPTICS0078_READ_SIZE 1024

#define SYNAPTICS0078_IMAGE_WIDTH 144
#define SYNAPTICS0078_IMAGE_HEIGHT 56

#define SYNAPTICS0078_DATA_DIR_ENV "FP_SYNAPTICS0078_DATA_DIR"
#define SYNAPTICS0078_MODULE_FILE "module.bin"
#define SYNAPTICS0078_MODULE_SIZE 10500
#define SYNAPTICS0078_MODULE_SHA256 \
  "67f4a332d89f76fe12c57fa7f67b43a076f689e78267b888c694e26b05caacc1"
#define SYNAPTICS0078_PROGRAM_SIZE 18869

#define CMD_ROM_INFO 0x01
#define CMD_QUERY_19 0x19
#define CMD_SENSOR_ID 0x75
#define CMD_MODULE 0x06
#define CMD_PROGRAM 0x02

#define STATUS_MODULE_ALREADY_LOADED 0x04aa

/* Reply lengths seen on a real 0078; the minimums are what the parsers read. */
#define ROM_INFO_LEN 38
#define ROM_INFO_MIN_LEN 0x18
#define QUERY_19_LEN 68
#define SENSOR_ID_LEN 10

#define ROM_MAJOR 6
#define ROM_MINOR 0x14
#define SENSOR_TYPE 0x2449

/* (major, minor) of the 75 reply that resolve to sensor type 0x2449. */
static const struct
{
  guint16 major;
  guint16 minor;
  guint16 sensor_type;
} sensor_ids[] = {
  { 0x00b7, 0x0037, SENSOR_TYPE },
  { 0x00b7, 0x0048, SENSOR_TYPE },
};

struct _FpiDeviceSynaptics0078
{
  FpImageDevice parent;

  GBytes       *module;
  GBytes       *program;
  gchar        *serial;
  guint8        module_state;   /* RomInfo byte 17, 0 = no module loaded */
};
G_DECLARE_FINAL_TYPE (FpiDeviceSynaptics0078, fpi_device_synaptics0078, FPI,
                      DEVICE_SYNAPTICS0078, FpImageDevice);
G_DEFINE_TYPE (FpiDeviceSynaptics0078, fpi_device_synaptics0078, FP_TYPE_IMAGE_DEVICE);

enum open_states {
  OPEN_ROM_INFO_SEND,
  OPEN_ROM_INFO_RECV,
  OPEN_QUERY_19_SEND,
  OPEN_QUERY_19_RECV,
  OPEN_SENSOR_ID_SEND,
  OPEN_SENSOR_ID_RECV,
  OPEN_LOAD_FILES,
  OPEN_MODULE_SEND,
  OPEN_MODULE_RECV,
  OPEN_CONFIRM_SEND,
  OPEN_CONFIRM_RECV,
  OPEN_NUM_STATES,
};

/* ========================= Helpers ========================= */

static void
clear_session_data (FpiDeviceSynaptics0078 *self)
{
  g_clear_pointer (&self->module, g_bytes_unref);
  g_clear_pointer (&self->program, g_bytes_unref);
  g_clear_pointer (&self->serial, g_free);
  self->module_state = 0;
}

static guint16
read_u16le (const guint8 *data, gsize offset)
{
  return data[offset] | (data[offset + 1] << 8);
}

static guint32
read_u32le (const guint8 *data, gsize offset)
{
  return (guint32) data[offset] |
         ((guint32) data[offset + 1] << 8) |
         ((guint32) data[offset + 2] << 16) |
         ((guint32) data[offset + 3] << 24);
}

/* Checks the leading u16le status word of a reply. */
static gboolean
check_status (FpiUsbTransfer *transfer,
              guint8          opcode,
              GError        **error)
{
  guint16 status;

  if (transfer->actual_length < 2)
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_PROTO,
                   "Reply to command 0x%02x is %" G_GSSIZE_FORMAT
                   " bytes, too short for a status word",
                   opcode, transfer->actual_length);
      return FALSE;
    }

  status = read_u16le (transfer->buffer, 0);
  fp_dbg ("Command 0x%02x: reply %" G_GSSIZE_FORMAT " bytes, status 0x%04x",
          opcode, transfer->actual_length, status);

  if (status == 0)
    return TRUE;

  if (opcode == CMD_MODULE && status == STATUS_MODULE_ALREADY_LOADED)
    g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_PROTO,
                 "Command 0x%02x failed with status 0x%04x: "
                 "the capture module was already loaded",
                 opcode, status);
  else
    g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_PROTO,
                 "Command 0x%02x failed with status 0x%04x",
                 opcode, status);

  return FALSE;
}

static gboolean
check_length (FpiUsbTransfer *transfer,
              guint8          opcode,
              gssize          expected,
              gssize          minimum,
              GError        **error)
{
  if (transfer->actual_length != expected)
    fp_dbg ("Command 0x%02x: reply is %" G_GSSIZE_FORMAT
            " bytes, expected %" G_GSSIZE_FORMAT,
            opcode, transfer->actual_length, expected);

  if (transfer->actual_length < minimum)
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_PROTO,
                   "Reply to command 0x%02x is %" G_GSSIZE_FORMAT
                   " bytes, need at least %" G_GSSIZE_FORMAT,
                   opcode, transfer->actual_length, minimum);
      return FALSE;
    }

  return TRUE;
}

static void
send_cb (FpiUsbTransfer *transfer, FpDevice *dev,
         gpointer user_data, GError *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  if (transfer->actual_length != transfer->length)
    {
      fpi_ssm_mark_failed (transfer->ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                     "Short write of command 0x%02x: "
                                                     "%" G_GSSIZE_FORMAT " of %" G_GSSIZE_FORMAT " bytes",
                                                     transfer->buffer[0],
                                                     transfer->actual_length,
                                                     transfer->length));
      return;
    }

  fpi_ssm_next_state (transfer->ssm);
}

/* Sends a command on EP_OUT; takes ownership of @buffer. */
static void
send_command (FpiSsm   *ssm,
              FpDevice *dev,
              guint8   *buffer,
              gsize     length)
{
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (dev);

  fp_dbg ("Sending command 0x%02x (%" G_GSIZE_FORMAT " bytes)", buffer[0], length);

  transfer->ssm = ssm;
  transfer->short_is_error = TRUE;
  fpi_usb_transfer_fill_bulk_full (transfer, SYNAPTICS0078_EP_OUT,
                                   buffer, length, g_free);
  fpi_usb_transfer_submit (transfer, SYNAPTICS0078_TIMEOUT,
                           fpi_device_get_cancellable (dev),
                           send_cb, NULL);
}

static void
send_opcode (FpiSsm   *ssm,
             FpDevice *dev,
             guint8    opcode)
{
  guint8 *buffer = g_malloc (1);

  buffer[0] = opcode;
  send_command (ssm, dev, buffer, 1);
}

static void
recv_reply (FpiSsm                *ssm,
            FpDevice              *dev,
            FpiUsbTransferCallback callback)
{
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (dev);

  transfer->ssm = ssm;
  fpi_usb_transfer_fill_bulk (transfer, SYNAPTICS0078_EP_IN,
                              SYNAPTICS0078_READ_SIZE);
  fpi_usb_transfer_submit (transfer, SYNAPTICS0078_TIMEOUT,
                           fpi_device_get_cancellable (dev),
                           callback, NULL);
}

/* ===================== Reply handlers ====================== */

static gboolean
handle_rom_info (FpiDeviceSynaptics0078 *self,
                 FpiUsbTransfer         *transfer,
                 GError                **error)
{
  const guint8 *reply = transfer->buffer;
  g_autoptr(GString) serial = NULL;
  guint8 major, minor;
  guint32 build;
  gint i;

  if (!check_status (transfer, CMD_ROM_INFO, error))
    return FALSE;
  if (!check_length (transfer, CMD_ROM_INFO, ROM_INFO_LEN, ROM_INFO_MIN_LEN, error))
    return FALSE;

  build = read_u32le (reply, 6);
  major = reply[10];
  minor = reply[11];
  self->module_state = reply[17];

  fp_info ("ROM version %u.%u (build %u)", major, minor, build);

  if (major != ROM_MAJOR || minor != ROM_MINOR)
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_NOT_SUPPORTED,
                   "Unsupported ROM version %u.%u, only %u.%u is supported",
                   major, minor, ROM_MAJOR, ROM_MINOR);
      return FALSE;
    }

  serial = g_string_new (NULL);
  for (i = 0x12; i < 0x18; i++)
    g_string_append_printf (serial, "%02x",
                            (guint8) ((reply[i] << 4) | (reply[i] >> 4)));

  g_free (self->serial);
  self->serial = g_string_free (g_steal_pointer (&serial), FALSE);
  fp_info ("Serial %s", self->serial);

  return TRUE;
}

static gboolean
handle_query_19 (FpiDeviceSynaptics0078 *self,
                 FpiUsbTransfer         *transfer,
                 GError                **error)
{
  if (!check_status (transfer, CMD_QUERY_19, error))
    return FALSE;

  /* Only the status is checked; the payload is not decoded. */
  if (transfer->actual_length != QUERY_19_LEN)
    fp_dbg ("Command 0x%02x: reply is %" G_GSSIZE_FORMAT " bytes, expected %d",
            CMD_QUERY_19, transfer->actual_length, QUERY_19_LEN);

  return TRUE;
}

static gboolean
handle_sensor_id (FpiDeviceSynaptics0078 *self,
                  FpiUsbTransfer         *transfer,
                  GError                **error)
{
  const guint8 *reply = transfer->buffer;
  guint32 reserved;
  guint16 major, minor;
  guint i;

  if (!check_status (transfer, CMD_SENSOR_ID, error))
    return FALSE;
  if (!check_length (transfer, CMD_SENSOR_ID, SENSOR_ID_LEN, SENSOR_ID_LEN, error))
    return FALSE;

  reserved = read_u32le (reply, 2);
  minor = read_u16le (reply, 6);
  major = read_u16le (reply, 8);

  if (reserved != 0)
    fp_dbg ("Sensor id reply: reserved word is 0x%08x, expected 0", reserved);

  for (i = 0; i < G_N_ELEMENTS (sensor_ids); i++)
    {
      if (sensor_ids[i].major == major && sensor_ids[i].minor == minor)
        {
          fp_info ("Sensor type 0x%04x (id %04x/%04x)",
                   sensor_ids[i].sensor_type, major, minor);
          return TRUE;
        }
    }

  g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_NOT_SUPPORTED,
               "Unsupported sensor id %04x/%04x, only sensor type 0x%04x is supported",
               major, minor, SENSOR_TYPE);
  return FALSE;
}

static gboolean
handle_module (FpiDeviceSynaptics0078 *self,
               FpiUsbTransfer         *transfer,
               GError                **error)
{
  return check_status (transfer, CMD_MODULE, error);
}

static gboolean
handle_confirm (FpiDeviceSynaptics0078 *self,
                FpiUsbTransfer         *transfer,
                GError                **error)
{
  if (!check_status (transfer, CMD_ROM_INFO, error))
    return FALSE;
  if (!check_length (transfer, CMD_ROM_INFO, ROM_INFO_LEN, ROM_INFO_MIN_LEN, error))
    return FALSE;

  self->module_state = transfer->buffer[17];
  if (self->module_state == 0)
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_PROTO,
                   "Capture module upload not confirmed: RomInfo byte 17 is still 0");
      return FALSE;
    }

  fp_info ("Capture module: uploaded and confirmed (RomInfo byte 17 = 0x%02x)",
           self->module_state);
  return TRUE;
}

static void
open_reply_cb (FpiUsbTransfer *transfer, FpDevice *dev,
               gpointer user_data, GError *error)
{
  FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS0078 (dev);
  gboolean ok = FALSE;

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  switch (fpi_ssm_get_cur_state (transfer->ssm))
    {
    case OPEN_ROM_INFO_RECV:
      ok = handle_rom_info (self, transfer, &error);
      break;

    case OPEN_QUERY_19_RECV:
      ok = handle_query_19 (self, transfer, &error);
      break;

    case OPEN_SENSOR_ID_RECV:
      ok = handle_sensor_id (self, transfer, &error);
      break;

    case OPEN_MODULE_RECV:
      ok = handle_module (self, transfer, &error);
      break;

    case OPEN_CONFIRM_RECV:
      ok = handle_confirm (self, transfer, &error);
      break;

    default:
      g_assert_not_reached ();
    }

  if (!ok)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  fpi_ssm_next_state (transfer->ssm);
}

/* ======================= Data files ======================== */

static const gchar *
get_data_dir (void)
{
  const gchar *dir = g_getenv (SYNAPTICS0078_DATA_DIR_ENV);

  if (dir != NULL && *dir != '\0')
    return dir;

  return SYNAPTICS0078_DATA_DIR;
}

static GBytes *
read_data_file (const gchar *path,
                GError     **error)
{
  g_autoptr(GError) local_error = NULL;
  gchar *contents = NULL;
  gsize length = 0;

  if (!g_file_get_contents (path, &contents, &length, &local_error))
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL,
                   "Cannot read %s: %s. The fingerprint-06cb-0078 setup tool "
                   "writes this file; set " SYNAPTICS0078_DATA_DIR_ENV
                   " to use another directory",
                   path, local_error->message);
      return NULL;
    }

  return g_bytes_new_take (contents, length);
}

static gboolean
load_data_files (FpiDeviceSynaptics0078 *self,
                 GError                **error)
{
  const gchar *dir = get_data_dir ();
  g_autofree gchar *module_path = NULL;
  g_autofree gchar *program_name = NULL;
  g_autofree gchar *program_path = NULL;
  g_autofree gchar *checksum = NULL;
  g_autoptr(GBytes) module = NULL;
  g_autoptr(GBytes) program = NULL;
  const guint8 *data;
  gsize size;

  fp_info ("Data directory %s", dir);

  module_path = g_build_filename (dir, SYNAPTICS0078_MODULE_FILE, NULL);
  module = read_data_file (module_path, error);
  if (!module)
    return FALSE;

  data = g_bytes_get_data (module, &size);
  if (size != SYNAPTICS0078_MODULE_SIZE)
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID,
                   "%s is %" G_GSIZE_FORMAT " bytes, expected %d",
                   module_path, size, SYNAPTICS0078_MODULE_SIZE);
      return FALSE;
    }

  checksum = g_compute_checksum_for_data (G_CHECKSUM_SHA256, data, size);
  if (g_strcmp0 (checksum, SYNAPTICS0078_MODULE_SHA256) != 0)
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID,
                   "%s has sha256 %s, expected %s",
                   module_path, checksum, SYNAPTICS0078_MODULE_SHA256);
      return FALSE;
    }

  program_name = g_strdup_printf ("program-%s.bin", self->serial);
  program_path = g_build_filename (dir, program_name, NULL);
  program = read_data_file (program_path, error);
  if (!program)
    return FALSE;

  data = g_bytes_get_data (program, &size);
  if (size != SYNAPTICS0078_PROGRAM_SIZE)
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID,
                   "%s is %" G_GSIZE_FORMAT " bytes, expected %d",
                   program_path, size, SYNAPTICS0078_PROGRAM_SIZE);
      return FALSE;
    }

  if (data[0] != CMD_PROGRAM)
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID,
                   "%s starts with 0x%02x, expected 0x%02x (a complete 02 command)",
                   program_path, data[0], CMD_PROGRAM);
      return FALSE;
    }

  g_clear_pointer (&self->module, g_bytes_unref);
  g_clear_pointer (&self->program, g_bytes_unref);
  self->module = g_steal_pointer (&module);
  self->program = g_steal_pointer (&program);

  return TRUE;
}

/* ======================= Open / close ====================== */

/*
 * fp_device_open() opens the GUsbDevice before calling img_open, and only
 * fpi_device_close_complete() closes it again. After a failed open the core
 * never calls img_close (the device is not marked open), so nothing would
 * close it, and the next fp_device_open() on the same FpDevice would fail
 * with "already open" until the device is re-enumerated. Close it here.
 */
static void
close_usb_after_failed_open (FpDevice *dev)
{
  g_autoptr(GError) close_error = NULL;

  if (!g_usb_device_close (fpi_device_get_usb_device (dev), &close_error))
    fp_dbg ("Closing the USB device after a failed open: %s",
            close_error->message);
}

static void
send_module (FpiDeviceSynaptics0078 *self,
             FpiSsm                 *ssm)
{
  const guint8 *module;
  guint8 *buffer;
  gsize size;

  module = g_bytes_get_data (self->module, &size);
  buffer = g_malloc (size + 1);
  buffer[0] = CMD_MODULE;
  memcpy (buffer + 1, module, size);

  send_command (ssm, FP_DEVICE (self), buffer, size + 1);
}

static void
open_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS0078 (dev);
  GError *error = NULL;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case OPEN_ROM_INFO_SEND:
    case OPEN_CONFIRM_SEND:
      send_opcode (ssm, dev, CMD_ROM_INFO);
      break;

    case OPEN_QUERY_19_SEND:
      send_opcode (ssm, dev, CMD_QUERY_19);
      break;

    case OPEN_SENSOR_ID_SEND:
      send_opcode (ssm, dev, CMD_SENSOR_ID);
      break;

    case OPEN_ROM_INFO_RECV:
    case OPEN_QUERY_19_RECV:
    case OPEN_SENSOR_ID_RECV:
    case OPEN_MODULE_RECV:
    case OPEN_CONFIRM_RECV:
      recv_reply (ssm, dev, open_reply_cb);
      break;

    case OPEN_LOAD_FILES:
      if (!load_data_files (self, &error))
        {
          fpi_ssm_mark_failed (ssm, error);
          break;
        }

      if (self->module_state != 0)
        {
          fp_info ("Capture module: already loaded (RomInfo byte 17 = 0x%02x)",
                   self->module_state);
          fpi_ssm_mark_completed (ssm);
          break;
        }

      fp_info ("Capture module: not loaded, uploading");
      fpi_ssm_next_state (ssm);
      break;

    case OPEN_MODULE_SEND:
      send_module (self, ssm);
      break;

    default:
      g_assert_not_reached ();
    }
}

static void
open_complete (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS0078 (dev);
  g_autoptr(GError) release_error = NULL;

  if (error)
    {
      /* The core does not call img_close after a failed open. */
      if (!g_usb_device_release_interface (fpi_device_get_usb_device (dev),
                                           SYNAPTICS0078_INTERFACE, 0,
                                           &release_error))
        fp_dbg ("Releasing the interface after a failed open: %s",
                release_error->message);

      close_usb_after_failed_open (dev);
      clear_session_data (self);
    }

  fpi_image_device_open_complete (FP_IMAGE_DEVICE (dev), error);
}

static void
dev_open (FpImageDevice *dev)
{
  FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS0078 (dev);
  GError *error = NULL;
  FpiSsm *ssm;

  clear_session_data (self);

  if (!g_usb_device_claim_interface (fpi_device_get_usb_device (FP_DEVICE (dev)),
                                     SYNAPTICS0078_INTERFACE, 0, &error))
    {
      close_usb_after_failed_open (FP_DEVICE (dev));
      fpi_image_device_open_complete (dev, error);
      return;
    }

  ssm = fpi_ssm_new (FP_DEVICE (dev), open_run_state, OPEN_NUM_STATES);
  fpi_ssm_start (ssm, open_complete);
}

static void
dev_close (FpImageDevice *dev)
{
  FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS0078 (dev);
  GError *error = NULL;

  g_usb_device_release_interface (fpi_device_get_usb_device (FP_DEVICE (dev)),
                                  SYNAPTICS0078_INTERFACE, 0, &error);

  clear_session_data (self);

  fpi_image_device_close_complete (dev, error);
}

/* ================= Activation (capture: A4) ================ */

/*
 * Capture is not implemented yet. Failing activate itself would leave the
 * core in the ACTIVATING state until close, so the next capture would trip
 * its state machine; instead activate succeeds and the session is ended at
 * once: the core deactivates (back to INACTIVE) and the action returns
 * NOT_SUPPORTED.
 */
static void
dev_activate (FpImageDevice *dev)
{
  fpi_image_device_activate_complete (dev, NULL);
  fpi_image_device_session_error (dev,
                                  fpi_device_error_new_msg (FP_DEVICE_ERROR_NOT_SUPPORTED,
                                                            "image capture is not implemented yet"));
}

static void
dev_deactivate (FpImageDevice *dev)
{
  fpi_image_device_deactivate_complete (dev, NULL);
}

static const FpIdEntry id_table[] = {
  { .vid = 0x06cb, .pid = 0x0078, },
  { .vid = 0, .pid = 0, .driver_data = 0 },
};

static void
fpi_device_synaptics0078_init (FpiDeviceSynaptics0078 *self)
{
}

static void
fpi_device_synaptics0078_class_init (FpiDeviceSynaptics0078Class *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);
  FpImageDeviceClass *img_class = FP_IMAGE_DEVICE_CLASS (klass);

  dev_class->id = FP_COMPONENT;
  dev_class->full_name = "Synaptics Fingerprint Sensor 06cb:0078";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->id_table = id_table;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;
  /* nr_enroll_stages and score_threshold keep their defaults; A5 sets them. */

  img_class->img_width = SYNAPTICS0078_IMAGE_WIDTH;
  img_class->img_height = SYNAPTICS0078_IMAGE_HEIGHT;
  img_class->algorithm = FPI_DEVICE_ALGO_SIGFM;

  img_class->img_open = dev_open;
  img_class->img_close = dev_close;
  img_class->activate = dev_activate;
  img_class->deactivate = dev_deactivate;
}
