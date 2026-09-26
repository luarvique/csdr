# FSK/UART development notes

- `FskUartDecoder` is a native `Module<float, unsigned char>`; audio samples must
  never be processed in the Python binding.
- Output records are `bits comma-separated-start-samples hex-octets\n`.
  Bits 8 and 9 mean 8N1 and eight data bits plus an unchecked parity bit.
  Preserve absolute sample positions across input/output buffer boundaries.
- Distinct transmissions may carry identical bytes. A protocol consumer can
  deduplicate parallel UART interpretations using the first character's sample.
- Default runs hold 264 bytes (256-byte Modbus ADU plus eight noise bytes).
  Output must respect writer capacity, including with no remaining input.
- Configure with `-DBUILD_TESTING=ON`, then build and run CTest. Python is
  needed only for test signal generation; the normal library build stays native.
