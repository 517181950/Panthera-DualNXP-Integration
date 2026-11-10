#include "wirelink/types.h"

const char *wl_err_str(int err) {
  switch (err) {
  case WL_OK:
    return "Success (WL_OK)";
  case WL_ERR_INVALID_ARG:
    return "Invalid argument (WL_ERR_INVALID_ARG)";
  case WL_ERR_NOT_INITIALIZED:
    return "Not initialized (WL_ERR_NOT_INITIALIZED)";
  case WL_ERR_NOT_SUPPORTED:
    return "Not supported (WL_ERR_NOT_SUPPORTED)";
  case WL_ERR_NO_MEM:
    return "Out of memory (WL_ERR_NO_MEM)";
  case WL_ERR_BUF_TOO_SMALL:
    return "Buffer too small (WL_ERR_BUF_TOO_SMALL)";
  case WL_ERR_NO_SPACE:
    return "Insufficient contiguous buffer space (WL_ERR_NO_SPACE)";
  case WL_ERR_NO_DATA:
    return "Insufficient readable buffer data (WL_ERR_NO_DATA)";
  case WL_ERR_BAD_FRAME:
    return "Bad frame format (WL_ERR_BAD_FRAME)";
  case WL_ERR_COBS_DECODE:
    return "COBS decode failed (WL_ERR_COBS_DECODE)";
  case WL_ERR_CRC:
    return "CRC mismatch (WL_ERR_CRC)";
  case WL_ERR_FRAME_TOO_LONG:
    return "Frame too long (WL_ERR_FRAME_TOO_LONG)";
  case WL_ERR_BUSY:
    return "Resource busy (WL_ERR_BUSY)";
  case WL_ERR_TIMEOUT:
    return "Transaction timeout (WL_ERR_TIMEOUT)";
  case WL_ERR_TX_FAILED:
    return "Transmission failed (WL_ERR_TX_FAILED)";
  case WL_ERR_INVALID_STATE:
    return "Invalid state (WL_ERR_INVALID_STATE)";
  case WL_ERR_WOULD_BLOCK:
    return "Operation would block (WL_ERR_WOULD_BLOCK)";
  case WL_ERR_QUEUE_FULL:
    return "Queue or slot is full (WL_ERR_QUEUE_FULL)";
  case WL_ERR_NOT_FOUND:
    return "Object not found (WL_ERR_NOT_FOUND)";
  case WL_ERR_PAYLOAD_TOO_LONG:
    return "Payload exceeds configured limit (WL_ERR_PAYLOAD_TOO_LONG)";
  case WL_ERR_IO:
    return "I/O path error (WL_ERR_IO)";
  case WL_ERR_CANCELLED:
    return "Operation cancelled (WL_ERR_CANCELLED)";
  case WL_ERR_PROTOCOL_VERSION:
    return "Unsupported frame version (WL_ERR_PROTOCOL_VERSION)";
  case WL_ERR_REENTRANT:
    return "Callback re-entry or concurrent access (WL_ERR_REENTRANT)";
  case WL_ERR_ID_EXHAUSTED:
    return "Communication identity exhausted (WL_ERR_ID_EXHAUSTED)";
  case WL_ERR_CORRUPT_PAYLOAD:
    return "Corrupt payload (WL_ERR_CORRUPT_PAYLOAD)";
  default:
    return "Unknown error";
  }
}
