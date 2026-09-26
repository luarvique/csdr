"""Synthetic audio through the compiled CSDR module; standard library only."""
import array
import math
import random
import subprocess
import sys
import unittest

STREAM, CLI = sys.argv[1:3]
del sys.argv[1:3]


def synth(frames, fs=12000, baud=1200, mark=1300, space=2100, parity=None,
          noise=0.0, lead=0.1, gap=0.05):
    phase = elapsed = 0.0
    audio = array.array('f')
    rng = random.Random(1)

    def emit(bit, seconds):
        nonlocal elapsed, phase
        elapsed += seconds
        for _ in range(round(elapsed * fs) - len(audio)):
            phase += 2 * math.pi * (mark if bit else space) / fs
            audio.append(0.5 * math.sin(phase) + rng.gauss(0, noise))

    for frame in frames:
        emit(1, lead)
        for byte in frame:
            bits = [0] + [(byte >> i) & 1 for i in range(8)]
            if parity:
                bit = bin(byte).count('1') & 1
                bits.append(bit if parity == 'E' else 1 - bit)
            for bit in bits + [1]:
                emit(bit, 1 / baud)
        emit(1, gap)
    return audio


def decode(audio, chunk=997, capacity=31, rate=12000, cli=False):
    command = [CLI, 'fskuartdecode', '--sample-rate', str(rate)] if cli else [STREAM, str(chunk), str(capacity), str(rate)]
    result = subprocess.run(command, input=audio.tobytes(), capture_output=True, timeout=15, check=True)
    frames = []
    for line in result.stdout.splitlines():
        bits, starts, data = line.split()
        starts = tuple(map(int, starts.split(b',')))
        data = bytes.fromhex(data.decode())
        assert len(data) == len(starts)
        assert all(a < b for a, b in zip(starts, starts[1:]))
        assert len(data) <= 264
        frames.append((int(bits), starts, data))
    return frames


class FskUartTest(unittest.TestCase):
    def testBinaryOctetsAndParity(self):
        data = bytes(range(256))
        for parity in (None, 'E', 'O'):
            for tones in ((1300, 2100), (1200, 2200)):
                with self.subTest(parity=parity, tones=tones):
                    frames = decode(synth([data], parity=parity, mark=tones[0], space=tones[1]))
                    self.assertIn(data, [f[2] for f in frames])

    def testNoiseAndFractionalSamplesPerBit(self):
        data = bytes.fromhex('0d170002001b000000020404454504c034')
        for rate, noise in ((11025, 0.05), (12000, 0.15), (22050, 0.15)):
            with self.subTest(rate=rate):
                frames = decode(synth([data], fs=rate, noise=noise), rate=rate)
                self.assertIn(data, [f[2] for f in frames])

    def testChunkAndWriterBoundaries(self):
        data = bytes.fromhex('030300000005842b')
        audio = synth([data, data], parity='O')
        expected = decode(audio)
        for chunk, capacity in ((1, 1), (7, 3), (1000000, 1000000)):
            with self.subTest(chunk=chunk, capacity=capacity):
                self.assertEqual(decode(audio, chunk, capacity), expected)
        # Both UARTs interpret the same physical start; real repeats start later.
        matching = [f for f in expected if f[2] == data]
        self.assertEqual(len(matching), 4)
        self.assertEqual(matching[0][1], matching[1][1])
        self.assertNotEqual(matching[0][1][0], matching[2][1][0])

    def testMaximumRun(self):
        data = b'\x55' * 8 + bytes(range(256))
        for parity in (None, 'E'):
            with self.subTest(parity=parity):
                self.assertIn(data, [f[2] for f in decode(synth([data], parity=parity))])

    def testLongStreamAndBoundedRuns(self):
        data = bytes(range(256)) * 3
        frames = [f[2] for f in decode(synth([data])) if f[0] == 8]
        self.assertEqual(b''.join(frames), data)
        self.assertTrue(all(len(f) <= 264 for f in frames))

    def testCli(self):
        audio = synth([bytes(range(100))])
        self.assertEqual(decode(audio, cli=True), decode(audio))

    def testInvalidParameters(self):
        subprocess.run([STREAM, 'invalid'], check=True, timeout=5)


if __name__ == '__main__':
    unittest.main()
