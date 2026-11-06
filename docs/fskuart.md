# Audio FSK / UART decoder

`Csdr::FskUartDecoder` is a native `Module<float, unsigned char>`, using the same
reader/writer and worker interfaces as `RttyDecoder` and `SitorBDecoder`.
It accepts real audio from an FM discriminator. Two sliding, one-bit DFT windows
compare mark and space energy after DC removal; per-sample decisions feed
parallel asynchronous UART deframers. All DSP and character timing run in C++.

The intended consumer is Modbus RTU telemetry over a narrow FM channel:

```
FM IQ -> FmDemod -> FskUartDecoder -> Modbus protocol parser
```

The protocol parser validates CRCs, resolves candidate UART interpretations and
formats requests/responses. The native decoder emits candidate octet runs.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `sampleRate` | required | Audio samples per second |
| `baudRate` | 1200 | Modem symbols per second |
| `markFreq` / `spaceFreq` | 1300 / 2100 Hz | ITU-T V.23 tones |
| `gapChars` | 3.5 | Idle gap, counted from the last stop-bit centre |
| `maxFrame` | 264 | Maximum run length in octets |

The tone detector also tolerates Bell 202's 1200/2200 Hz tones at these defaults.
For explicitly selected tones, pass their frequencies to the constructor or CLI.
Constructor bounds reject invalid rates, tones, non-finite parameters and
unbounded frame/window allocations. CSDR's `-ffast-math` requires a bitwise
finite-value check before allocation.

## Output contract

Each output line is ASCII:

```
8 1204,1304,1404 000dff
```

The first field is 8 for 8N1 or 9 for eight data bits plus a parity bit and one
stop bit. Parity is not checked. The second field has one absolute start-sample
index per octet; the third contains hex octets, preserving every byte including
NUL, CR and LF. Runs end at an idle gap, bad stop bit or `maxFrame` limit.
A finite capture should include trailing idle audio so its final run can finish.

Two UARTs can interpret the same transmission. A protocol parser can identify
that occurrence by the first valid frame octet's start sample, even when one
candidate run includes extra noise bytes. A later transmission with identical
bytes has a different start sample and must be preserved.

State survives arbitrary input chunks. Pending output is bounded to two runs
and drained within the writer's capacity before more audio is consumed.

## CLI and tests

```sh
csdr fskuartdecode --sample-rate 12000 < audio-f32.raw
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

CTest uses only Python's standard library to generate audio. The compiled module
is tested with single-sample reads and single-byte writes, guarded output memory,
all octet values, even/odd parity, both tone plans, bounded long runs, fractional
samples per bit, Gaussian noise, invalid parameters and CLI parity. Protocol CRC
and request/response regressions belong to the consuming OpenWebRX suite.
