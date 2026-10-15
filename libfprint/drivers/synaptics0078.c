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
 *
 * Activation re-reads RomInfo, because a USB reset or a resume wipes the
 * RAM-resident module: when byte 17 == 0 it uploads the module again (06,
 * then 01 to confirm), otherwise it sends nothing else.
 *
 * Capture (change_state AWAIT_FINGER_ON): 02 + program arms the sensor
 * (skipped when the finger-off probe left it armed), EP 0x83 reports the
 * finger (02...) and then "image ready" (03 .. 04 00 ..), 51 00 20 00 00
 * returns the 144x56 image after an 18-byte header, and 04, 52 stop the
 * capture. 04 and 52 run on every exit path after a 02 (errors and
 * deactivation included); the one exception is a finger-off report, below.
 * A failed/aborted event, an error in the image header or status 0x0412
 * ("no capture in progress") on 51 is a retry, not an error.
 *
 * Finger off (change_state AWAIT_FINGER_OFF): the sensor sends no event when
 * the finger is lifted, so the driver re-arms it (02) and listens for a short
 * window. A finger event in the window means the finger is still down: the
 * image is read and discarded, 04/52 end the capture and the probe repeats
 * after a pause. A failed/aborted event or a missing image-ready event is
 * inconclusive and ends the probe the same way. A window without a finger
 * event means the finger is off; the sensor is then left armed, so the next
 * AWAIT_FINGER_ON only listens.
 *
 * Debug aids, read at open and off unless their variable is set (only root
 * controls fprintd's environment):
 *   FP_SYNAPTICS0078_DUMP_DIR          logs mean, sd, ridge coverage, gradient
 *                                      and the mean absolute difference to the
 *                                      last delivered image for every 51 image
 *                                      (captures and the discarded finger-off
 *                                      probe images), and writes each as
 *                                      <seq>-<monotonic ms>-<tag>.pgm (144x56,
 *                                      native) into this directory (created
 *                                      0700, files 0600). Biometric data: point
 *                                      it at a private, untracked directory.
 *   FP_SYNAPTICS0078_PROBE_WINDOW_MS   finger-off probe window, 50..10000
 *   FP_SYNAPTICS0078_PROBE_BACKOFF_MS  pause between finger-off probes, 0..60000
 */

#define FP_COMPONENT "synaptics0078"

#include "drivers_api.h"

#include <errno.h>
#include <math.h>

#define SYNAPTICS0078_EP_OUT 0x01
#define SYNAPTICS0078_EP_IN 0x81
#define SYNAPTICS0078_EP_INT 0x83
#define SYNAPTICS0078_INTERFACE 0
#define SYNAPTICS0078_TIMEOUT 5000
#define SYNAPTICS0078_READ_SIZE 1024
#define SYNAPTICS0078_PROGRAM_READ_SIZE 0x1000
#define SYNAPTICS0078_IMAGE_READ_SIZE 16384
/* Interrupt reads are 8 bytes (wMaxPacketSize); the events are 5 bytes. */
#define SYNAPTICS0078_INT_READ_SIZE 8

/* Wait for "image ready" after the finger event; code 5 restarts it. */
#define SYNAPTICS0078_READY_TIMEOUT 5000
/* A 51 reply that says "not ready" is followed by a plain read with this
 * timeout; if nothing arrives, 51 is sent again. At most this many replies. */
#define SYNAPTICS0078_IMAGE_REREAD_TIMEOUT 1000
#define SYNAPTICS0078_IMAGE_MAX_REPLIES (1 + 3)
#define SYNAPTICS0078_CLEANUP_TIMEOUT 2000

/*
 * Finger-off probing (see the top of the file). The finger event of a re-arm
 * with the finger still down came 117 ms after 02 on hardware, so the window
 * leaves margin. Both delays are starting values, tuned in A5.
 */
#define SYNAPTICS0078_FINGER_OFF_PROBE_MS 500
#define SYNAPTICS0078_FINGER_OFF_RETRY_MS 250

/* Debug aids (see the top of the file). */
#define SYNAPTICS0078_DUMP_DIR_ENV "FP_SYNAPTICS0078_DUMP_DIR"
#define SYNAPTICS0078_PROBE_WINDOW_ENV "FP_SYNAPTICS0078_PROBE_WINDOW_MS"
#define SYNAPTICS0078_PROBE_BACKOFF_ENV "FP_SYNAPTICS0078_PROBE_BACKOFF_MS"
/* Ridge coverage as proto/parsers.py image_quality(): the share of 8x8
 * blocks whose standard deviation exceeds 15 grey levels. */
#define DIAG_BLOCK 8
#define DIAG_BLOCK_SD 15.0

#define SYNAPTICS0078_IMAGE_WIDTH 144
#define SYNAPTICS0078_IMAGE_HEIGHT 56
/*
 * Matching parameters, set in A5 from the reference captures; the data and
 * the rules are in fingerprint-06cb-0078/research/steps/a5-threshold.md.
 *
 * UPSCALE: the 144x56 image reaches SIGFM upscaled by nearest-neighbour pixel
 * replication (SIGFM ignores FpiImageFlags, so scaling happens here). 2x beat
 * 1x: no impostor score above 0 (1x: 6) and fewer false rejects at every
 * enrollment size.
 *
 * SCORE_THRESHOLD: SIGFM scores grow with about the 4th power of the
 * consistent keypoint matches; m matches that all agree score
 * C(C(m, 2), 2). 105 is the score of 6 such matches: the first whole count at
 * least two matches above the highest impostor score seen (6, at 1x, worth
 * ~3.4 matches).
 *
 * ENROLL_STAGES: one touch covers only about 7 x 3 mm of the finger, so false
 * rejects come from enrollment coverage; they were still falling at 9 images.
 */
#define SYNAPTICS0078_UPSCALE 2
#define SYNAPTICS0078_SCORE_THRESHOLD 105
#define SYNAPTICS0078_ENROLL_STAGES 18

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
#define CMD_IMAGE 0x51
#define CMD_STOP 0x04
#define CMD_CLEANUP 0x52

#define STATUS_MODULE_ALREADY_LOADED 0x04aa
/* On 51 and on 04: no capture in progress (inferred from hardware runs). */
#define STATUS_NO_CAPTURE 0x0412

/* Reply lengths seen on a real 0078; the minimums are what the parsers read. */
#define ROM_INFO_LEN 38
#define ROM_INFO_MIN_LEN 0x18
#define QUERY_19_LEN 68
#define SENSOR_ID_LEN 10
#define PROGRAM_REPLY_LEN 2154

/* 51 reply: u16 status, u32 length, then u16 x, u16 y, u16 w1, u16 w2,
 * u32 error and the pixels. */
#define IMAGE_HEADER_LEN 18

/* EP 0x83 events: byte 0 is the kind; kind 3 carries a u16le code at [2]. */
#define EVENT_KIND_FINGER 2
#define EVENT_KIND_CAPTURE 3
#define EVENT_CODE_FAILED 0
#define EVENT_CODE_READY 4
#define EVENT_CODE_EXTEND 5
#define EVENT_CODE_ABORTED 6
#define EVENT_CODE_READ 7

static const guint8 cmd_image[] = { CMD_IMAGE, 0x00, 0x20, 0x00, 0x00 };
static const guint8 cmd_stop[] = { CMD_STOP };
static const guint8 cmd_cleanup[] = { CMD_CLEANUP };

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

enum capture_outcome {
  OUTCOME_NONE,
  OUTCOME_IMAGE,
  OUTCOME_RETRY,
};

struct _FpiDeviceSynaptics0078
{
  FpImageDevice        parent;

  GBytes              *module;
  GBytes              *program;
  gchar               *serial;
  guint8               module_state; /* RomInfo byte 17, 0 = no module loaded */

  /* Capture session (one activation) */
  FpiSsm              *ssm;          /* running capture or finger-off SSM */
  GCancellable        *interrupt_cancellable; /* EP 0x83 waits; deactivate cancels it */
  gboolean             deactivating;
  gboolean             armed;        /* 02 sent, 04/52 not yet run */
  gboolean             pause_pending; /* delayed state change of the finger-off SSM */
  GError              *deactivate_error;

  /* Current run of the capture SSM */
  gboolean             probing;      /* FALSE: capture, TRUE: finger-off probing */
  enum capture_outcome outcome;
  gboolean             finger_reported;
  gboolean             finger_off;
  FpImage             *capture_image;
  gint64               ready_deadline_us;
  guint                image_replies;
  gboolean             image_reread;

  /* Finger-off probing */
  guint                probe_count;
  gint64               probe_start_us;
  gint64               window_deadline_us;
  guint                probe_window_ms;
  guint                probe_backoff_ms;

  /* Debug aids (SYNAPTICS0078_DUMP_DIR_ENV); diag_dir NULL = off */
  gchar               *diag_dir;
  gboolean             diag_write_failed; /* warned once, no more writes */
  guint                diag_seq;
  guint8              *diag_last_capture; /* native pixels of the last image delivered */
  gint64               arm_reply_us; /* reply to the last 02, 0 = none yet */
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

enum activate_states {
  ACTIVATE_ROM_INFO_SEND,
  ACTIVATE_ROM_INFO_RECV,
  ACTIVATE_MODULE_SEND,
  ACTIVATE_MODULE_RECV,
  ACTIVATE_CONFIRM_SEND,
  ACTIVATE_CONFIRM_RECV,
  ACTIVATE_NUM_STATES,
};

/*
 * One SSM serves both the capture (AWAIT_FINGER_ON) and the finger-off
 * probing (AWAIT_FINGER_OFF), see self->probing. CAPTURE_DISARM and
 * CAPTURE_PAUSE are reached only while probing. CAPTURE_CLEANUP is the
 * cleanup state: errors, retries, a delivered image and deactivation all end
 * there, and it runs 04/52 whenever the sensor is armed, except after a
 * finger-off report.
 */
enum capture_states {
  CAPTURE_ARM_SEND,
  CAPTURE_ARM_RECV,
  CAPTURE_WAIT_FINGER,
  CAPTURE_WAIT_READY,
  CAPTURE_IMAGE_SEND,
  CAPTURE_IMAGE_RECV,
  CAPTURE_DISARM,
  CAPTURE_PAUSE,
  CAPTURE_CLEANUP,
  CAPTURE_NUM_STATES,
};

/* 04 then 52; never fails. Run as a sub-SSM or on its own at deactivation. */
enum disarm_states {
  DISARM_STOP_SEND,
  DISARM_STOP_RECV,
  DISARM_CLEANUP_SEND,
  DISARM_CLEANUP_RECV,
  DISARM_NUM_STATES,
};

/* ========================= Helpers ========================= */

static void
clear_session_data (FpiDeviceSynaptics0078 *self)
{
  g_clear_pointer (&self->module, g_bytes_unref);
  g_clear_pointer (&self->program, g_bytes_unref);
  g_clear_pointer (&self->serial, g_free);
  self->module_state = 0;
  g_clear_pointer (&self->diag_dir, g_free);
  g_clear_pointer (&self->diag_last_capture, g_free);
}

static void
clear_capture_data (FpiDeviceSynaptics0078 *self)
{
  g_clear_object (&self->interrupt_cancellable);
  g_clear_object (&self->capture_image);
  g_clear_error (&self->deactivate_error);
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

/* Sends a command on EP_OUT; @free_func (if any) releases @buffer. */
static void
send_command_full (FpiSsm                *ssm,
                   FpDevice              *dev,
                   guint8                *buffer,
                   gsize                  length,
                   GDestroyNotify         free_func,
                   guint                  timeout_ms,
                   GCancellable          *cancellable,
                   FpiUsbTransferCallback callback)
{
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (dev);

  fp_dbg ("Sending command 0x%02x (%" G_GSIZE_FORMAT " bytes)", buffer[0], length);

  transfer->ssm = ssm;
  transfer->short_is_error = TRUE;
  fpi_usb_transfer_fill_bulk_full (transfer, SYNAPTICS0078_EP_OUT,
                                   buffer, length, free_func);
  fpi_usb_transfer_submit (transfer, timeout_ms, cancellable, callback, NULL);
}

/* Sends a command on EP_OUT; takes ownership of @buffer. */
static void
send_command (FpiSsm   *ssm,
              FpDevice *dev,
              guint8   *buffer,
              gsize     length)
{
  send_command_full (ssm, dev, buffer, length, g_free, SYNAPTICS0078_TIMEOUT,
                     fpi_device_get_cancellable (dev), send_cb);
}

/*
 * Capture-path commands use no cancellable: the reply to a command is always
 * read before anything else is sent, so the bulk pipe never holds a stale
 * reply. Deactivation is noticed between exchanges instead (each transfer is
 * bounded by its timeout).
 */
static void
send_capture_command (FpiSsm                *ssm,
                      FpDevice              *dev,
                      const guint8          *data,
                      gsize                  length,
                      guint                  timeout_ms,
                      FpiUsbTransferCallback callback)
{
  send_command_full (ssm, dev, g_memdup2 (data, length), length, g_free,
                     timeout_ms, NULL, callback);
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
recv_reply_full (FpiSsm                *ssm,
                 FpDevice              *dev,
                 gsize                  size,
                 guint                  timeout_ms,
                 GCancellable          *cancellable,
                 FpiUsbTransferCallback callback)
{
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (dev);

  transfer->ssm = ssm;
  fpi_usb_transfer_fill_bulk (transfer, SYNAPTICS0078_EP_IN, size);
  fpi_usb_transfer_submit (transfer, timeout_ms, cancellable, callback, NULL);
}

static void
recv_reply (FpiSsm                *ssm,
            FpDevice              *dev,
            FpiUsbTransferCallback callback)
{
  recv_reply_full (ssm, dev, SYNAPTICS0078_READ_SIZE, SYNAPTICS0078_TIMEOUT,
                   fpi_device_get_cancellable (dev), callback);
}

/* ===================== Reply handlers ====================== */

/* Status, length and ROM version of a RomInfo reply; records byte 17. */
static gboolean
check_rom_info (FpiDeviceSynaptics0078 *self,
                FpiUsbTransfer         *transfer,
                GError                **error)
{
  const guint8 *reply = transfer->buffer;
  guint8 major, minor;
  guint32 build;

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

  return TRUE;
}

static gboolean
handle_rom_info (FpiDeviceSynaptics0078 *self,
                 FpiUsbTransfer         *transfer,
                 GError                **error)
{
  const guint8 *reply = transfer->buffer;
  g_autoptr(GString) serial = NULL;
  gint i;

  if (!check_rom_info (self, transfer, error))
    return FALSE;

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

/* ======================= Debug aids ======================== */

/* A millisecond value from @name within [@min, @max], else @fallback. */
static guint
debug_ms_from_env (const gchar *name,
                   guint        fallback,
                   guint        min,
                   guint        max)
{
  const gchar *raw = g_getenv (name);
  g_autoptr(GError) error = NULL;
  guint64 value;

  if (raw == NULL || *raw == '\0')
    return fallback;

  if (!g_ascii_string_to_unsigned (raw, 10, min, max, &value, &error))
    {
      fp_warn ("Ignoring %s=%s: %s", name, raw, error->message);
      return fallback;
    }

  fp_info ("Debug override: %s=%u ms (default %u ms)", name, (guint) value, fallback);
  return (guint) value;
}

static void
load_debug_settings (FpiDeviceSynaptics0078 *self)
{
  const gchar *dir = g_getenv (SYNAPTICS0078_DUMP_DIR_ENV);

  self->probe_window_ms = debug_ms_from_env (SYNAPTICS0078_PROBE_WINDOW_ENV,
                                             SYNAPTICS0078_FINGER_OFF_PROBE_MS,
                                             50, 10000);
  self->probe_backoff_ms = debug_ms_from_env (SYNAPTICS0078_PROBE_BACKOFF_ENV,
                                              SYNAPTICS0078_FINGER_OFF_RETRY_MS,
                                              0, 60000);

  self->diag_write_failed = FALSE;
  self->diag_seq = 0;
  if (dir == NULL || *dir == '\0')
    return;

  self->diag_dir = g_strdup (dir);
  if (g_mkdir_with_parents (dir, 0700) != 0)
    {
      /* Stats are still logged; only the image files are skipped. */
      fp_warn ("Cannot create %s=%s: %s; logging image stats only",
               SYNAPTICS0078_DUMP_DIR_ENV, dir, g_strerror (errno));
      self->diag_write_failed = TRUE;
    }
  fp_info ("Debug: image stats on, images dumped to %s", dir);
}

/* Writes @pixels (native size) as a PGM; never fails the caller. */
static void
diag_write_pgm (FpiDeviceSynaptics0078 *self,
                const gchar            *name,
                const gchar            *comment,
                const guint8           *pixels)
{
  const gsize n_pixels = SYNAPTICS0078_IMAGE_WIDTH * SYNAPTICS0078_IMAGE_HEIGHT;
  g_autofree gchar *path = NULL;
  g_autoptr(GByteArray) data = g_byte_array_new ();
  g_autoptr(GError) error = NULL;
  g_autofree gchar *header = NULL;

  if (self->diag_write_failed)
    return;

  header = g_strdup_printf ("P5\n# %s\n%d %d\n255\n", comment,
                            SYNAPTICS0078_IMAGE_WIDTH, SYNAPTICS0078_IMAGE_HEIGHT);
  g_byte_array_append (data, (const guint8 *) header, strlen (header));
  g_byte_array_append (data, pixels, n_pixels);

  /* CONSISTENT writes a temporary file and renames it: a symlink at the
   * target is replaced, never followed. */
  path = g_build_filename (self->diag_dir, name, NULL);
  if (!g_file_set_contents_full (path, (const gchar *) data->data, data->len,
                                 G_FILE_SET_CONTENTS_CONSISTENT, 0600, &error))
    {
      fp_warn ("Cannot write %s: %s; no further image files", path, error->message);
      self->diag_write_failed = TRUE;
    }
}

/*
 * Logs the stats of a 51 reply that carries a full image (an error header
 * may still carry pixels) and dumps it; remembers a delivered capture as the
 * reference for the difference. A no-op unless the debug aid is on.
 */
static void
diag_image (FpiDeviceSynaptics0078 *self,
            FpiUsbTransfer         *transfer,
            gboolean                delivered)
{
  const gint width = SYNAPTICS0078_IMAGE_WIDTH;
  const gint height = SYNAPTICS0078_IMAGE_HEIGHT;
  const gsize n_pixels = width * height;
  const guint8 *reply = transfer->buffer;
  const guint8 *px;
  g_autofree gchar *tag = NULL;
  g_autofree gchar *name = NULL;
  g_autofree gchar *comment = NULL;
  g_autofree gchar *diff_text = NULL;
  gint64 now_us = g_get_monotonic_time ();
  gdouble sum = 0, sq = 0, mean, sd, grad = 0, diff = 0;
  guint grad_n = 0, blocks = 0, covered = 0;
  gint x, y, bx, by;
  gsize i;

  if (self->diag_dir == NULL ||
      transfer->actual_length != (gssize) (IMAGE_HEADER_LEN + n_pixels) ||
      read_u32le (reply, 2) != (guint32) (transfer->actual_length - 6))
    return;

  px = reply + IMAGE_HEADER_LEN;
  self->diag_seq++;

  for (i = 0; i < n_pixels; i++)
    {
      sum += px[i];
      sq += (gdouble) px[i] * px[i];
    }
  mean = sum / n_pixels;
  sd = sqrt (MAX (sq / n_pixels - mean * mean, 0));

  /* Ridge contrast: mean absolute difference of neighbouring pixels. */
  for (y = 0; y < height; y++)
    for (x = 0; x < width; x++)
      {
        if (x + 1 < width)
          {
            grad += ABS (px[y * width + x + 1] - px[y * width + x]);
            grad_n++;
          }
        if (y + 1 < height)
          {
            grad += ABS (px[(y + 1) * width + x] - px[y * width + x]);
            grad_n++;
          }
      }
  grad /= grad_n;

  for (by = 0; by < height / DIAG_BLOCK; by++)
    for (bx = 0; bx < width / DIAG_BLOCK; bx++)
      {
        gdouble bsum = 0, bsq = 0, bmean;
        const gint n = DIAG_BLOCK * DIAG_BLOCK;

        for (y = by * DIAG_BLOCK; y < (by + 1) * DIAG_BLOCK; y++)
          for (x = bx * DIAG_BLOCK; x < (bx + 1) * DIAG_BLOCK; x++)
            {
              bsum += px[y * width + x];
              bsq += (gdouble) px[y * width + x] * px[y * width + x];
            }
        bmean = bsum / n;
        blocks++;
        if (sqrt (MAX (bsq / n - bmean * bmean, 0)) > DIAG_BLOCK_SD)
          covered++;
      }

  if (self->diag_last_capture)
    {
      for (i = 0; i < n_pixels; i++)
        diff += ABS (px[i] - self->diag_last_capture[i]);
      diff_text = g_strdup_printf ("%.1f", diff / n_pixels);
    }
  else
    {
      diff_text = g_strdup ("-");
    }

  tag = self->probing ? g_strdup_printf ("probe%02u", self->probe_count) :
        g_strdup ("capture");
  name = g_strdup_printf ("%06u-%011" G_GINT64_FORMAT "ms-%s%s.pgm",
                          self->diag_seq, now_us / 1000, tag,
                          (read_u16le (reply, 0) != 0 || read_u32le (reply, 14) != 0) ?
                          "-err" : "");

  fp_dbg ("Diag image %u %s: mean %.1f, sd %.2f, coverage %.0f%%, "
          "gradient %.1f, diff vs last capture %s, %" G_GINT64_FORMAT
          " ms after 02, file %s",
          self->diag_seq, tag, mean, sd, 100.0 * covered / blocks, grad,
          diff_text, self->arm_reply_us ? (now_us - self->arm_reply_us) / 1000 : -1,
          name);

  comment = g_strdup_printf ("synaptics0078 seq %u monotonic_us %" G_GINT64_FORMAT
                             " %s status 0x%04x length %u x %u y %u w1 %u w2 %u error 0x%08x",
                             self->diag_seq, now_us, tag, read_u16le (reply, 0),
                             read_u32le (reply, 2), read_u16le (reply, 6),
                             read_u16le (reply, 8), read_u16le (reply, 10),
                             read_u16le (reply, 12), read_u32le (reply, 14));
  diag_write_pgm (self, name, comment, px);

  if (delivered)
    {
      g_free (self->diag_last_capture);
      self->diag_last_capture = g_memdup2 (px, n_pixels);
    }
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
  load_debug_settings (self);

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
  clear_capture_data (self);

  fpi_image_device_close_complete (dev, error);
}

/* ======================== Activation ======================= */

static void
activate_reply_cb (FpiUsbTransfer *transfer, FpDevice *dev,
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
    case ACTIVATE_ROM_INFO_RECV:
      ok = check_rom_info (self, transfer, &error);
      if (ok && self->module_state != 0)
        {
          fp_info ("Activation: capture module already loaded (RomInfo byte 17 = 0x%02x)",
                   self->module_state);
          fpi_ssm_mark_completed (transfer->ssm);
          return;
        }
      if (ok)
        fp_info ("Activation: capture module not loaded (RomInfo byte 17 = 0), re-uploading");
      break;

    case ACTIVATE_MODULE_RECV:
      ok = handle_module (self, transfer, &error);
      break;

    case ACTIVATE_CONFIRM_RECV:
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

/*
 * A USB reset or a resume wipes the RAM-resident module, so RomInfo byte 17
 * is re-read at every activation. 06 is sent only when it is 0 (a second 06
 * would be refused with 0x04aa).
 */
static void
activate_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS0078 (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case ACTIVATE_ROM_INFO_SEND:
    case ACTIVATE_CONFIRM_SEND:
      send_opcode (ssm, dev, CMD_ROM_INFO);
      break;

    case ACTIVATE_MODULE_SEND:
      send_module (self, ssm);
      break;

    case ACTIVATE_ROM_INFO_RECV:
    case ACTIVATE_MODULE_RECV:
    case ACTIVATE_CONFIRM_RECV:
      recv_reply (ssm, dev, activate_reply_cb);
      break;

    default:
      g_assert_not_reached ();
    }
}

static void
activate_complete (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  fpi_image_device_activate_complete (FP_IMAGE_DEVICE (dev), error);
}

static void
dev_activate (FpImageDevice *dev)
{
  FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS0078 (dev);
  FpiSsm *ssm;

  self->deactivating = FALSE;
  self->armed = FALSE;
  self->pause_pending = FALSE;
  self->arm_reply_us = 0;
  g_clear_object (&self->interrupt_cancellable);
  self->interrupt_cancellable = g_cancellable_new ();

  ssm = fpi_ssm_new (FP_DEVICE (dev), activate_run_state, ACTIVATE_NUM_STATES);
  fpi_ssm_start (ssm, activate_complete);
}

/* ===================== Disarm (04, 52) ===================== */

/*
 * A failed 04 or 52 is logged and never fails the caller. A timeout moves on
 * to the next command; any other error stops talking to the device (it is
 * gone or wedged), as proto/capture.py cleanup_commands() does.
 */
static void
disarm_transfer_failed (FpiSsm *ssm,
                        guint8  opcode,
                        GError *error)
{
  gboolean timed_out = g_error_matches (error, G_USB_DEVICE_ERROR,
                                        G_USB_DEVICE_ERROR_TIMED_OUT);

  fp_warn ("Cleanup command 0x%02x failed: %s", opcode, error->message);
  g_error_free (error);

  if (timed_out && fpi_ssm_get_cur_state (ssm) < DISARM_CLEANUP_SEND)
    fpi_ssm_jump_to_state (ssm, DISARM_CLEANUP_SEND);
  else
    fpi_ssm_mark_completed (ssm);
}

static void
disarm_send_cb (FpiUsbTransfer *transfer, FpDevice *dev,
                gpointer user_data, GError *error)
{
  if (error)
    {
      disarm_transfer_failed (transfer->ssm, transfer->buffer[0], error);
      return;
    }

  fpi_ssm_next_state (transfer->ssm);
}

static void
disarm_reply_cb (FpiUsbTransfer *transfer, FpDevice *dev,
                 gpointer user_data, GError *error)
{
  guint8 opcode;
  guint16 status;

  if (fpi_ssm_get_cur_state (transfer->ssm) == DISARM_STOP_RECV)
    opcode = CMD_STOP;
  else
    opcode = CMD_CLEANUP;

  if (error)
    {
      disarm_transfer_failed (transfer->ssm, opcode, error);
      return;
    }

  if (transfer->actual_length < 2)
    {
      fp_warn ("Cleanup command 0x%02x: reply is %" G_GSSIZE_FORMAT
               " bytes, too short for a status word",
               opcode, transfer->actual_length);
      fpi_ssm_next_state (transfer->ssm);
      return;
    }

  status = read_u16le (transfer->buffer, 0);

  /* 04 answers 0 while a capture is still waiting and 0x0412 when none is
   * in progress (after a delivered image or a failed capture). */
  if (opcode == CMD_CLEANUP)
    fp_dbg ("Command 0x%02x: status 0x%04x", opcode, status);
  else if (status == 0)
    fp_dbg ("Command 0x%02x: status 0x%04x, capture stopped", opcode, status);
  else if (status == STATUS_NO_CAPTURE)
    fp_dbg ("Command 0x%02x: status 0x%04x, no capture in progress",
            opcode, status);
  else
    fp_warn ("Command 0x%02x: unexpected status 0x%04x", opcode, status);

  fpi_ssm_next_state (transfer->ssm);
}

static void
disarm_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS0078 (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case DISARM_STOP_SEND:
      if (!self->armed)
        {
          fpi_ssm_mark_completed (ssm);
          break;
        }
      /* 04 and 52 are sent once, whatever they answer. */
      self->armed = FALSE;
      send_capture_command (ssm, dev, cmd_stop, sizeof (cmd_stop),
                            SYNAPTICS0078_CLEANUP_TIMEOUT, disarm_send_cb);
      break;

    case DISARM_CLEANUP_SEND:
      send_capture_command (ssm, dev, cmd_cleanup, sizeof (cmd_cleanup),
                            SYNAPTICS0078_CLEANUP_TIMEOUT, disarm_send_cb);
      break;

    case DISARM_STOP_RECV:
    case DISARM_CLEANUP_RECV:
      recv_reply_full (ssm, dev, SYNAPTICS0078_READ_SIZE,
                       SYNAPTICS0078_CLEANUP_TIMEOUT, NULL, disarm_reply_cb);
      break;

    default:
      g_assert_not_reached ();
    }
}

static FpiSsm *
disarm_ssm_new (FpDevice *dev)
{
  return fpi_ssm_new_full (dev, disarm_run_state, DISARM_NUM_STATES,
                           DISARM_NUM_STATES, "disarm");
}

/* ======================== Deactivation ===================== */

static void finish_deactivation (FpiDeviceSynaptics0078 *self,
                                 GError                 *error);

static void
deactivate_disarm_complete (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS0078 (dev);

  /* The disarm SSM never fails. */
  g_clear_error (&error);

  finish_deactivation (self, g_steal_pointer (&self->deactivate_error));
}

/* Runs 04/52 if the sensor is still armed, then completes deactivation. */
static void
finish_deactivation (FpiDeviceSynaptics0078 *self,
                     GError                 *error)
{
  if (self->armed)
    {
      g_clear_error (&self->deactivate_error);
      self->deactivate_error = error;
      fpi_ssm_start (disarm_ssm_new (FP_DEVICE (self)),
                     deactivate_disarm_complete);
      return;
    }

  /* The core may deactivate while finger-off probing runs at the end of an
   * enroll; an error unwinding it must not fail the finished action. */
  if (error)
    {
      fp_warn ("Error while deactivating, ignored: %s", error->message);
      g_clear_error (&error);
    }

  self->deactivating = FALSE;
  fpi_image_device_deactivate_complete (FP_IMAGE_DEVICE (self), NULL);
}

static void
dev_deactivate (FpImageDevice *dev)
{
  FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS0078 (dev);

  self->deactivating = TRUE;
  g_cancellable_cancel (self->interrupt_cancellable);

  if (self->ssm)
    {
      /* A pending pause between finger-off probes: end it now. */
      if (self->pause_pending)
        {
          self->pause_pending = FALSE;
          fpi_ssm_cancel_delayed_state_change (self->ssm);
          fpi_ssm_mark_completed (self->ssm);
        }

      /* Otherwise the SSM notices the flag (or its cancelled EP 0x83 wait),
       * unwinds through CAPTURE_CLEANUP and its completion finishes the
       * deactivation. */
      return;
    }

  finish_deactivation (self, NULL);
}

/* ========================== Capture ======================== */

enum event_action {
  EVENT_IGNORE,
  EVENT_FINGER,
  EVENT_READY,
  EVENT_FAIL,
  EVENT_EXTEND,
};

static const gchar *const event_action_names[] = {
  [EVENT_IGNORE] = "ignore",
  [EVENT_FINGER] = "finger",
  [EVENT_READY] = "ready",
  [EVENT_FAIL] = "fail",
  [EVENT_EXTEND] = "extend",
};

enum image_reply {
  IMAGE_REPLY_IMAGE,
  IMAGE_REPLY_NOT_READY,
  IMAGE_REPLY_CAPTURE_ERROR,
  IMAGE_REPLY_NO_CAPTURE,
  IMAGE_REPLY_BAD,
};

static const gchar *const image_reply_names[] = {
  [IMAGE_REPLY_IMAGE] = "image",
  [IMAGE_REPLY_NOT_READY] = "not ready",
  [IMAGE_REPLY_CAPTURE_ERROR] = "capture error",
  [IMAGE_REPLY_NO_CAPTURE] = "no capture in progress",
  [IMAGE_REPLY_BAD] = "bad reply",
};

static gchar *
bytes_to_hex (const guint8 *data,
              gssize        length)
{
  GString *hex = g_string_new (NULL);
  gssize i;

  for (i = 0; i < length; i++)
    g_string_append_printf (hex, "%02x", data[i]);

  return g_string_free (hex, FALSE);
}

/* Milliseconds left until @deadline_us, rounded up; 0 once it has passed. */
static guint
remaining_ms (gint64 deadline_us)
{
  gint64 left = deadline_us - g_get_monotonic_time ();

  if (left <= 0)
    return 0;

  return (guint) ((left + 999) / 1000);
}

/* Classifies one EP 0x83 event as proto/parsers.py classify_interrupt(). */
static enum event_action
classify_event (FpiUsbTransfer *transfer,
                gint64          arm_reply_us,
                guint16        *code)
{
  const guint8 *data = transfer->buffer;
  gssize length = transfer->actual_length;
  g_autofree gchar *hex = bytes_to_hex (data, length);
  enum event_action action = EVENT_IGNORE;

  *code = 0;

  if (length >= 1 && data[0] == EVENT_KIND_FINGER)
    {
      action = EVENT_FINGER;
    }
  else if (length >= 4 && data[0] == EVENT_KIND_CAPTURE)
    {
      *code = read_u16le (data, 2);

      switch (*code)
        {
        case EVENT_CODE_READY:
        case EVENT_CODE_READ:
          action = EVENT_READY;
          break;

        case EVENT_CODE_FAILED:
        case EVENT_CODE_ABORTED:
          action = EVENT_FAIL;
          break;

        case EVENT_CODE_EXTEND:
          action = EVENT_EXTEND;
          break;

        default:
          break;
        }
    }

  if (arm_reply_us > 0)
    fp_dbg ("EP 0x%02x event %s: %s (+%" G_GINT64_FORMAT " ms after 02)",
            SYNAPTICS0078_EP_INT, hex, event_action_names[action],
            (g_get_monotonic_time () - arm_reply_us) / 1000);
  else
    fp_dbg ("EP 0x%02x event %s: %s", SYNAPTICS0078_EP_INT, hex,
            event_action_names[action]);

  return action;
}

static gboolean
image_status_not_ready (guint16 status)
{
  return status == 0x0001 || status == 0x0002 ||
         status == 0x0005 || status == 0x0006;
}

/*
 * Classifies a 51 reply as proto/parsers.py parse_image_reply(). A header is
 * present iff the reply has at least 18 bytes and the u32 at [2] equals the
 * length after it. A header with a non-zero error field (any status) is a
 * capture error; status 1, 2, 5 or 6 without a header is "not ready";
 * 0x0412 is "no capture in progress". On IMAGE_REPLY_IMAGE @pixels points
 * into the transfer buffer; on IMAGE_REPLY_BAD @error says why.
 */
static enum image_reply
parse_image_reply (FpiUsbTransfer *transfer,
                   const guint8  **pixels,
                   GError        **error)
{
  const guint8 *reply = transfer->buffer;
  gssize length = transfer->actual_length;
  gboolean has_header;
  guint16 status, x = 0, y = 0;
  guint32 image_error = 0;
  gssize n_pixels;

  if (length < 2)
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_PROTO,
                   "Image reply is %" G_GSSIZE_FORMAT
                   " bytes, too short for a status word", length);
      return IMAGE_REPLY_BAD;
    }

  status = read_u16le (reply, 0);
  has_header = length >= IMAGE_HEADER_LEN &&
               read_u32le (reply, 2) == (guint32) (length - 6);

  if (has_header)
    {
      x = read_u16le (reply, 6);
      y = read_u16le (reply, 8);
      image_error = read_u32le (reply, 14);
      fp_dbg ("Image reply: %" G_GSSIZE_FORMAT " bytes, status 0x%04x, "
              "length %u, x %u, y %u, w1 %u, w2 %u, error 0x%08x",
              length, status, read_u32le (reply, 2), x, y,
              read_u16le (reply, 10), read_u16le (reply, 12), image_error);
    }
  else
    {
      fp_dbg ("Image reply: %" G_GSSIZE_FORMAT " bytes, status 0x%04x, no header",
              length, status);
    }

  if (status != 0)
    {
      if (has_header && image_error != 0)
        return IMAGE_REPLY_CAPTURE_ERROR;
      if (status == STATUS_NO_CAPTURE)
        return IMAGE_REPLY_NO_CAPTURE;
      if (!has_header && image_status_not_ready (status))
        return IMAGE_REPLY_NOT_READY;

      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_PROTO,
                   "Image reply has status 0x%04x", status);
      return IMAGE_REPLY_BAD;
    }

  if (!has_header)
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_PROTO,
                   "Image reply of %" G_GSSIZE_FORMAT
                   " bytes has no valid image header", length);
      return IMAGE_REPLY_BAD;
    }

  if (image_error != 0)
    return IMAGE_REPLY_CAPTURE_ERROR;

  n_pixels = length - IMAGE_HEADER_LEN;
  if (n_pixels != (gssize) x * y ||
      n_pixels != SYNAPTICS0078_IMAGE_WIDTH * SYNAPTICS0078_IMAGE_HEIGHT)
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_PROTO,
                   "Image reply carries %" G_GSSIZE_FORMAT " pixels for a "
                   "%ux%u header, expected %dx%d",
                   n_pixels, x, y,
                   SYNAPTICS0078_IMAGE_WIDTH, SYNAPTICS0078_IMAGE_HEIGHT);
      return IMAGE_REPLY_BAD;
    }

  *pixels = reply + IMAGE_HEADER_LEN;
  return IMAGE_REPLY_IMAGE;
}

/* Nearest-neighbour upscale by SYNAPTICS0078_UPSCALE (pixel replication). */
static FpImage *
build_image (const guint8 *pixels)
{
  const gint width = SYNAPTICS0078_IMAGE_WIDTH * SYNAPTICS0078_UPSCALE;
  const gint height = SYNAPTICS0078_IMAGE_HEIGHT * SYNAPTICS0078_UPSCALE;
  FpImage *img = fp_image_new (width, height);
  gint x, y;

  for (y = 0; y < height; y++)
    {
      const guint8 *src = pixels +
                          (y / SYNAPTICS0078_UPSCALE) * SYNAPTICS0078_IMAGE_WIDTH;
      guint8 *dst = img->data + y * width;

      for (x = 0; x < width; x++)
        dst[x] = src[x / SYNAPTICS0078_UPSCALE];
    }

  return img;
}

static void
send_program (FpiDeviceSynaptics0078 *self,
              FpiSsm                 *ssm)
{
  const guint8 *program;
  gsize size;

  program = g_bytes_get_data (self->program, &size);

  /* Even a partial write may have started the program. */
  self->armed = TRUE;
  /* Sent without a copy: self->program lives until close, which cannot run
   * while the device is active. */
  send_command_full (ssm, FP_DEVICE (self), (guint8 *) program, size, NULL,
                     SYNAPTICS0078_TIMEOUT, NULL, send_cb);
}

static void capture_interrupt_cb (FpiUsbTransfer *transfer,
                                  FpDevice       *dev,
                                  gpointer        user_data,
                                  GError         *error);

/* Waits on EP 0x83; @timeout_ms 0 waits until deactivation cancels it. */
static void
submit_interrupt (FpiDeviceSynaptics0078 *self,
                  FpiSsm                 *ssm,
                  guint                   timeout_ms)
{
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (FP_DEVICE (self));

  transfer->ssm = ssm;
  fpi_usb_transfer_fill_interrupt (transfer, SYNAPTICS0078_EP_INT,
                                   SYNAPTICS0078_INT_READ_SIZE);
  fpi_usb_transfer_submit (transfer, timeout_ms, self->interrupt_cancellable,
                           capture_interrupt_cb, NULL);
}

/* Finger-off window ended without a finger event: leave the sensor armed. */
static void
probe_finger_off (FpiDeviceSynaptics0078 *self,
                  FpiSsm                 *ssm)
{
  self->finger_off = TRUE;
  fpi_ssm_mark_completed (ssm);
}

/* No image-ready event within SYNAPTICS0078_READY_TIMEOUT of the finger. */
static void
capture_ready_timed_out (FpiDeviceSynaptics0078 *self,
                         FpiSsm                 *ssm)
{
  if (self->probing)
    {
      /* The finger may have lifted right after its event: inconclusive,
       * end this probe and probe again. */
      fp_dbg ("Finger-off probe %u: no image-ready event within %d ms, probing again",
              self->probe_count, SYNAPTICS0078_READY_TIMEOUT);
      fpi_ssm_jump_to_state (ssm, CAPTURE_DISARM);
      return;
    }

  fpi_ssm_mark_failed (ssm,
                       fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                 "Timed out after %d ms waiting "
                                                 "for the image-ready event",
                                                 SYNAPTICS0078_READY_TIMEOUT));
}

/* A failed/aborted event (code 0 or 6), before or after the finger. */
static void
capture_event_failed (FpiDeviceSynaptics0078 *self,
                      FpiSsm                 *ssm,
                      guint16                 code)
{
  if (self->probing)
    {
      /* Inconclusive about the finger: end this probe and probe again. */
      fp_dbg ("Finger-off probe %u: event code %u (failed/aborted), probing again",
              self->probe_count, code);
      fpi_ssm_jump_to_state (ssm, CAPTURE_DISARM);
      return;
    }

  fp_info ("Capture failed: event code %u (failed/aborted), asking for a retry",
           code);
  self->outcome = OUTCOME_RETRY;
  fpi_ssm_mark_completed (ssm);
}

static void
capture_interrupt_cb (FpiUsbTransfer *transfer, FpDevice *dev,
                      gpointer user_data, GError *error)
{
  FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS0078 (dev);
  FpiSsm *ssm = transfer->ssm;
  gint state = fpi_ssm_get_cur_state (ssm);
  enum event_action action;
  guint16 code;

  /* Deactivation cancels this wait; a cancelled (or late) read is the
   * normal way out then, not an error. */
  if (self->deactivating)
    {
      if (error)
        fp_dbg ("EP 0x%02x wait ended by deactivation: %s",
                SYNAPTICS0078_EP_INT, error->message);
      g_clear_error (&error);
      fpi_ssm_mark_completed (ssm);
      return;
    }

  if (error)
    {
      if (!g_error_matches (error, G_USB_DEVICE_ERROR, G_USB_DEVICE_ERROR_TIMED_OUT))
        {
          fpi_ssm_mark_failed (ssm, error);
          return;
        }
      g_error_free (error);

      if (state == CAPTURE_WAIT_FINGER && self->probing)
        probe_finger_off (self, ssm);
      else
        capture_ready_timed_out (self, ssm);
      return;
    }

  action = classify_event (transfer, self->arm_reply_us, &code);

  if (state == CAPTURE_WAIT_FINGER)
    {
      if (action == EVENT_FINGER)
        {
          if (self->probing)
            {
              fp_dbg ("Finger-off probe %u: finger still down", self->probe_count);
            }
          else
            {
              self->finger_reported = TRUE;
              fpi_image_device_report_finger_status (FP_IMAGE_DEVICE (dev), TRUE);
            }

          self->ready_deadline_us = g_get_monotonic_time () +
                                    SYNAPTICS0078_READY_TIMEOUT * G_TIME_SPAN_MILLISECOND;
          fpi_ssm_next_state (ssm);
          return;
        }

      if (action == EVENT_FAIL)
        {
          capture_event_failed (self, ssm, code);
          return;
        }

      /* Anything else, e.g. the 0000000000 that follows every 02: wait on. */
      fpi_ssm_jump_to_state (ssm, CAPTURE_WAIT_FINGER);
      return;
    }

  g_assert (state == CAPTURE_WAIT_READY);

  switch (action)
    {
    case EVENT_READY:
      /*
       * Code 7 counts as ready, as in proto/capture.py (python-validity
       * PR #288 does the same). The one time it was seen on this unit the 51
       * reply carried an error header, i.e. a retry: the 51 reply decides.
       */
      self->image_replies = 0;
      fpi_ssm_next_state (ssm);
      return;

    case EVENT_FAIL:
      capture_event_failed (self, ssm, code);
      return;

    case EVENT_EXTEND:
      self->ready_deadline_us = g_get_monotonic_time () +
                                SYNAPTICS0078_READY_TIMEOUT * G_TIME_SPAN_MILLISECOND;
      fpi_ssm_jump_to_state (ssm, CAPTURE_WAIT_READY);
      return;

    case EVENT_FINGER:
    case EVENT_IGNORE:
    default:
      fpi_ssm_jump_to_state (ssm, CAPTURE_WAIT_READY);
      return;
    }
}

static void
handle_arm_reply (FpiDeviceSynaptics0078 *self,
                  FpiUsbTransfer         *transfer,
                  GError                 *error)
{
  if (error || !check_status (transfer, CMD_PROGRAM, &error))
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  if (transfer->actual_length != PROGRAM_REPLY_LEN)
    fp_dbg ("Command 0x%02x: reply is %" G_GSSIZE_FORMAT " bytes, expected %d",
            CMD_PROGRAM, transfer->actual_length, PROGRAM_REPLY_LEN);

  self->arm_reply_us = g_get_monotonic_time ();
  if (self->probing)
    self->window_deadline_us = self->arm_reply_us +
                               self->probe_window_ms * G_TIME_SPAN_MILLISECOND;

  fpi_ssm_next_state (transfer->ssm);
}

static void
handle_image_reply (FpiDeviceSynaptics0078 *self,
                    FpiUsbTransfer         *transfer,
                    GError                 *error)
{
  FpiSsm *ssm = transfer->ssm;
  g_autoptr(GError) reply_error = NULL;
  const guint8 *pixels = NULL;
  enum image_reply kind;

  if (error)
    {
      if (self->image_reread &&
          g_error_matches (error, G_USB_DEVICE_ERROR, G_USB_DEVICE_ERROR_TIMED_OUT))
        {
          fp_dbg ("No further 51 reply within %d ms, sending 51 again",
                  SYNAPTICS0078_IMAGE_REREAD_TIMEOUT);
          g_error_free (error);
          fpi_ssm_jump_to_state (ssm, CAPTURE_IMAGE_SEND);
          return;
        }

      fpi_ssm_mark_failed (ssm, error);
      return;
    }

  self->image_replies++;
  kind = parse_image_reply (transfer, &pixels, &reply_error);
  diag_image (self, transfer, !self->probing && kind == IMAGE_REPLY_IMAGE);

  if (kind == IMAGE_REPLY_NOT_READY &&
      self->image_replies < SYNAPTICS0078_IMAGE_MAX_REPLIES)
    {
      fp_dbg ("51 reply %u: not ready, reading again", self->image_replies);
      self->image_reread = TRUE;
      fpi_ssm_jump_to_state (ssm, CAPTURE_IMAGE_RECV);
      return;
    }

  if (self->probing)
    {
      fp_dbg ("Finger-off probe %u: discarding the 51 reply (%s)",
              self->probe_count, image_reply_names[kind]);
      fpi_ssm_next_state (ssm);
      return;
    }

  switch (kind)
    {
    case IMAGE_REPLY_IMAGE:
      g_clear_object (&self->capture_image);
      self->capture_image = build_image (pixels);
      self->outcome = OUTCOME_IMAGE;
      fpi_ssm_mark_completed (ssm);
      return;

    case IMAGE_REPLY_CAPTURE_ERROR:
      fp_info ("Capture failed: the image reply carries an error, asking for a retry");
      self->outcome = OUTCOME_RETRY;
      fpi_ssm_mark_completed (ssm);
      return;

    case IMAGE_REPLY_NO_CAPTURE:
      fp_info ("Capture failed: no capture in progress (status 0x%04x), asking for a retry",
               STATUS_NO_CAPTURE);
      self->outcome = OUTCOME_RETRY;
      fpi_ssm_mark_completed (ssm);
      return;

    case IMAGE_REPLY_NOT_READY:
      fpi_ssm_mark_failed (ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                     "Image still not ready after %u replies",
                                                     self->image_replies));
      return;

    case IMAGE_REPLY_BAD:
    default:
      fpi_ssm_mark_failed (ssm, g_steal_pointer (&reply_error));
      return;
    }
}

static void
capture_reply_cb (FpiUsbTransfer *transfer, FpDevice *dev,
                  gpointer user_data, GError *error)
{
  FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS0078 (dev);

  switch (fpi_ssm_get_cur_state (transfer->ssm))
    {
    case CAPTURE_ARM_RECV:
      handle_arm_reply (self, transfer, error);
      break;

    case CAPTURE_IMAGE_RECV:
      handle_image_reply (self, transfer, error);
      break;

    default:
      g_assert_not_reached ();
    }
}

static void
capture_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS0078 (dev);
  gint state = fpi_ssm_get_cur_state (ssm);
  guint remaining;

  /* Entering any state means no delayed state change is pending. */
  self->pause_pending = FALSE;

  /* Deactivation is honoured between exchanges, never between a command
   * and its reply, so the bulk pipe stays in step. */
  if (self->deactivating && state < CAPTURE_CLEANUP &&
      state != CAPTURE_ARM_RECV && state != CAPTURE_IMAGE_RECV)
    {
      fpi_ssm_mark_completed (ssm);
      return;
    }

  switch (state)
    {
    case CAPTURE_ARM_SEND:
      if (self->probing)
        {
          self->probe_count++;
          fp_dbg ("Finger-off probe %u: arming", self->probe_count);
        }
      if (self->armed)
        {
          /* Left armed by the finger-off probe: just listen. (Probing
           * always starts disarmed; the window is set for completeness.) */
          fp_dbg ("Sensor already armed, listening on EP 0x%02x",
                  SYNAPTICS0078_EP_INT);
          self->window_deadline_us = g_get_monotonic_time () +
                                     self->probe_window_ms * G_TIME_SPAN_MILLISECOND;
          fpi_ssm_jump_to_state (ssm, CAPTURE_WAIT_FINGER);
          break;
        }
      send_program (self, ssm);
      break;

    case CAPTURE_ARM_RECV:
      recv_reply_full (ssm, dev, SYNAPTICS0078_PROGRAM_READ_SIZE,
                       SYNAPTICS0078_TIMEOUT, NULL, capture_reply_cb);
      break;

    case CAPTURE_WAIT_FINGER:
      if (!self->probing)
        {
          submit_interrupt (self, ssm, 0);
          break;
        }

      remaining = remaining_ms (self->window_deadline_us);
      if (remaining == 0)
        {
          probe_finger_off (self, ssm);
          break;
        }
      submit_interrupt (self, ssm, remaining);
      break;

    case CAPTURE_WAIT_READY:
      remaining = remaining_ms (self->ready_deadline_us);
      if (remaining == 0)
        {
          capture_ready_timed_out (self, ssm);
          break;
        }
      submit_interrupt (self, ssm, remaining);
      break;

    case CAPTURE_IMAGE_SEND:
      self->image_reread = FALSE;
      send_capture_command (ssm, dev, cmd_image, sizeof (cmd_image),
                            SYNAPTICS0078_TIMEOUT, send_cb);
      break;

    case CAPTURE_IMAGE_RECV:
      recv_reply_full (ssm, dev, SYNAPTICS0078_IMAGE_READ_SIZE,
                       self->image_reread ? SYNAPTICS0078_IMAGE_REREAD_TIMEOUT :
                       SYNAPTICS0078_TIMEOUT,
                       NULL, capture_reply_cb);
      break;

    case CAPTURE_DISARM:
      /* Finger still down (or inconclusive): end this capture. */
      fpi_ssm_start_subsm (ssm, disarm_ssm_new (dev));
      break;

    case CAPTURE_PAUSE:
      self->pause_pending = TRUE;
      fpi_ssm_jump_to_state_delayed (ssm, CAPTURE_ARM_SEND,
                                     self->probe_backoff_ms);
      break;

    case CAPTURE_CLEANUP:
      /* Every exit after a 02 ends with 04/52, except a finger-off report,
       * which leaves the sensor armed for the next touch. */
      if (self->armed && !self->finger_off)
        fpi_ssm_start_subsm (ssm, disarm_ssm_new (dev));
      else
        fpi_ssm_next_state (ssm);
      break;

    default:
      g_assert_not_reached ();
    }
}

/*
 * Outcomes are reported only here, once the SSM is done and self->ssm is
 * cleared: the core may call change_state or deactivate synchronously.
 */
static void
capture_complete (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS0078 (dev);
  FpImageDevice *idev = FP_IMAGE_DEVICE (dev);
  g_autoptr(FpImage) image = g_steal_pointer (&self->capture_image);
  gint64 elapsed_ms;

  self->ssm = NULL;
  self->pause_pending = FALSE;

  if (self->deactivating)
    {
      finish_deactivation (self, error);
      return;
    }

  if (error)
    {
      fpi_image_device_session_error (idev, error);
      return;
    }

  if (self->probing)
    {
      if (!self->finger_off)
        {
          fpi_image_device_session_error (idev,
                                          fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                                                    "Finger-off probing ended without a result"));
          return;
        }

      elapsed_ms = (g_get_monotonic_time () - self->probe_start_us) /
                   G_TIME_SPAN_MILLISECOND;
      fp_info ("Finger off after %u probe(s), %" G_GINT64_FORMAT " ms",
               self->probe_count, elapsed_ms);
      fpi_image_device_report_finger_status (idev, FALSE);
      return;
    }

  switch (self->outcome)
    {
    case OUTCOME_IMAGE:
      fp_dbg ("Image captured");
      fpi_image_device_image_captured (idev, g_steal_pointer (&image));
      break;

    case OUTCOME_RETRY:
      /* The core expects a retry in CAPTURE state (in enroll it moves on to
       * AWAIT_FINGER_OFF), so a failure before the finger event reports the
       * finger first. */
      if (!self->finger_reported)
        fpi_image_device_report_finger_status (idev, TRUE);
      fpi_image_device_retry_scan (idev, FP_DEVICE_RETRY_GENERAL);
      break;

    case OUTCOME_NONE:
    default:
      fpi_image_device_session_error (idev,
                                      fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                                                "Capture ended without a result"));
      break;
    }
}

static void
start_capture (FpiDeviceSynaptics0078 *self,
               gboolean                probing)
{
  if (self->ssm)
    {
      fp_warn ("A capture state machine is already running, not starting another");
      return;
    }

  self->probing = probing;
  self->outcome = OUTCOME_NONE;
  self->finger_reported = FALSE;
  self->finger_off = FALSE;
  self->image_replies = 0;
  self->image_reread = FALSE;
  g_clear_object (&self->capture_image);

  if (probing)
    {
      self->probe_count = 0;
      self->probe_start_us = g_get_monotonic_time ();
    }

  self->ssm = fpi_ssm_new_full (FP_DEVICE (self), capture_run_state,
                                CAPTURE_NUM_STATES, CAPTURE_CLEANUP,
                                probing ? "finger-off" : "capture");
  fpi_ssm_start (self->ssm, capture_complete);
}

static void
dev_change_state (FpImageDevice *dev, FpiImageDeviceState state)
{
  FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS0078 (dev);

  switch (state)
    {
    case FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_ON:
      start_capture (self, FALSE);
      break;

    case FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_OFF:
      start_capture (self, TRUE);
      break;

    /* CAPTURE: reported by the running capture SSM, which carries on. */
    case FPI_IMAGE_DEVICE_STATE_CAPTURE:
    case FPI_IMAGE_DEVICE_STATE_INACTIVE:
    case FPI_IMAGE_DEVICE_STATE_ACTIVATING:
    case FPI_IMAGE_DEVICE_STATE_DEACTIVATING:
    case FPI_IMAGE_DEVICE_STATE_IDLE:
    default:
      break;
    }
}

static const FpIdEntry id_table[] = {
  { .vid = 0x06cb, .pid = 0x0078, },
  { .vid = 0, .pid = 0, .driver_data = 0 },
};

static void
fpi_device_synaptics0078_init (FpiDeviceSynaptics0078 *self)
{
  /* load_debug_settings() at open may override these. */
  self->probe_window_ms = SYNAPTICS0078_FINGER_OFF_PROBE_MS;
  self->probe_backoff_ms = SYNAPTICS0078_FINGER_OFF_RETRY_MS;
}

static void
fpi_device_synaptics0078_finalize (GObject *object)
{
  FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS0078 (object);

  /* Normally freed by img_close already. */
  clear_session_data (self);
  clear_capture_data (self);

  G_OBJECT_CLASS (fpi_device_synaptics0078_parent_class)->finalize (object);
}

static void
fpi_device_synaptics0078_class_init (FpiDeviceSynaptics0078Class *klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);
  FpImageDeviceClass *img_class = FP_IMAGE_DEVICE_CLASS (klass);

  object_class->finalize = fpi_device_synaptics0078_finalize;

  dev_class->id = FP_COMPONENT;
  dev_class->full_name = "Synaptics Fingerprint Sensor 06cb:0078";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->id_table = id_table;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;
  dev_class->nr_enroll_stages = SYNAPTICS0078_ENROLL_STAGES;

  img_class->img_width = SYNAPTICS0078_IMAGE_WIDTH * SYNAPTICS0078_UPSCALE;
  img_class->img_height = SYNAPTICS0078_IMAGE_HEIGHT * SYNAPTICS0078_UPSCALE;
  img_class->algorithm = FPI_DEVICE_ALGO_SIGFM;
  img_class->score_threshold = SYNAPTICS0078_SCORE_THRESHOLD;

  img_class->img_open = dev_open;
  img_class->img_close = dev_close;
  img_class->activate = dev_activate;
  img_class->deactivate = dev_deactivate;
  img_class->change_state = dev_change_state;
}
