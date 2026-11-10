/* SPDX-License-Identifier: Apache-2.0 */

#include "wirelink/cobs.h"

#include <stddef.h>
#include <stdint.h>
#include <limits.h>

#include "wirelink/types.h"

size_t wl_cobs_encoded_max_size(size_t input_len) {
  if (input_len == 0) {
    return 1;
  }

  if (input_len > SIZE_MAX - input_len / 254 - 1) {
    return SIZE_MAX;
  }

  return input_len + input_len / 254 + 1;
}

static int wl_cobs_count_encoded_len(const uint8_t *input, size_t input_len,
                                    size_t *out_len) {
  size_t encoded_len = 1; // start code at index 0
  uint8_t code = 1;

  if (out_len == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  if (input == NULL && input_len > 0) {
    return WL_ERR_INVALID_ARG;
  }

  if (input_len == 0) {
    *out_len = 1;
    return WL_OK;
  }

  for (size_t i = 0; i < input_len; ++i) {
    if (input[i] == 0) {
      encoded_len += 1; // close current block, reserve next code slot
      code = 1;
      continue;
    }

    encoded_len += 1; // data byte
    code += 1;

    if (code == 0xFF) {
      encoded_len += 1; // force next block when code reaches 0xFF
      code = 1;
    }

    if (encoded_len == SIZE_MAX) {
      return WL_ERR_INVALID_ARG;
    }
  }

  *out_len = encoded_len;
  return WL_OK;
}

int wl_cobs_encode(const uint8_t *input, size_t input_len, uint8_t *output,
                   size_t output_capacity, size_t *output_len) {
  size_t required = 0;
  size_t write_index = 1;
  size_t code_index = 0;
  uint8_t code = 1;
  int ret;

  if (output_len == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  *output_len = 0;

  ret = wl_cobs_count_encoded_len(input, input_len, &required);
  if (ret != WL_OK) {
    return ret;
  }
  if (output_capacity < required) {
    return WL_ERR_BUF_TOO_SMALL;
  }
  if (output == NULL && required > 0) {
    return WL_ERR_INVALID_ARG;
  }

  if (input_len == 0) {
    output[0] = 1;
    *output_len = 1;
    return WL_OK;
  }

  for (size_t i = 0; i < input_len; ++i) {
    if (input[i] == 0) {
      output[code_index] = code;
      code = 1;
      code_index = write_index++;
      continue;
    }

    output[write_index++] = input[i];
    ++code;

    if (code == 0xFF) {
      output[code_index] = code;
      code = 1;
      code_index = write_index++;
    }
  }

  output[code_index] = code;
  *output_len = write_index;
  return WL_OK;
}

int wl_cobs_decode(const uint8_t *input, size_t input_len, uint8_t *output,
                   size_t output_capacity, size_t *output_len) {
  size_t required = 0;
  size_t in_pos = 0;
  uint8_t code = 0;

  if (output_len == NULL) {
    return WL_ERR_INVALID_ARG;
  }
  *output_len = 0;

  if (input == NULL && input_len > 0) {
    return WL_ERR_INVALID_ARG;
  }
  if (input_len == 0) {
    return WL_ERR_COBS_DECODE;
  }

  while (in_pos < input_len) {
    code = input[in_pos++];
    if (code == 0) {
      return WL_ERR_COBS_DECODE;
    }

    if ((in_pos + code - 1) > input_len) {
      return WL_ERR_COBS_DECODE;
    }

    required += (size_t)(code - 1);
    if ((required == 0) && (code - 1) != 0) {
      return WL_ERR_INVALID_ARG;
    }
    in_pos += (size_t)(code - 1);

    if (in_pos + (code - 1) < input_len && code != 0xFF) {
      ++required; // implicit 0 terminator
      if (required == 0) {
        return WL_ERR_INVALID_ARG;
      }
    }
  }

  if (required > output_capacity) {
    return WL_ERR_BUF_TOO_SMALL;
  }
  if (output == NULL && required > 0) {
    return WL_ERR_INVALID_ARG;
  }

  in_pos = 0;
  *output_len = 0;
  while (in_pos < input_len) {
    code = input[in_pos++];

    for (uint8_t i = 1; i < code; ++i) {
      output[(*output_len)++] = input[in_pos++];
    }

    if ((in_pos < input_len) && (code != 0xFF)) {
      output[(*output_len)++] = 0;
    }
  }

  return WL_OK;
}

int wl_cobs_decode_in_place(uint8_t *buffer, size_t input_len,
                            size_t *output_len) {
  return wl_cobs_decode(buffer, input_len, buffer, input_len, output_len);
}
