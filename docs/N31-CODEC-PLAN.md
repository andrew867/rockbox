# CS42L81 — plan to match the working Linux driver

The Linux driver (`sound/soc/apple/cs42l81-spi.c`, **3613 lines**) has been
rebuilt from the whole-image decompilation and now produces sound. Ours
(`firmware/drivers/audio/cs42l81.c`, **612 lines**) was written from
per-function extracts and from the bootloader's sequences.

The gap is not a list of tweaks. Ours is missing the entire codec state
machine and the play-graph construction, and three of the subsystems it *does*
have are invented.

## The sequence that works

This is the part to get right first, because nothing else matters if the order
is wrong.

```
cs42l81_play_prepare()   -> cs42_codec_prepare(rate)
cs42l81_play_start()     -> [prepare if needed] -> cs42_retailos_play_start()
cs42l81_play_stop()      -> cs42_retailos_play_stop()

cs42l81_pre_iis_start()  -> rmw(0x000F, 0x80, 0x00)      BEFORE IIS starts
cs42l81_post_iis_start() -> asp_status; play_unmute;      AFTER IIS is running
                            apply_user_vol
```

`cs42_codec_prepare(rate)`:

| # | step | notes |
| --- | --- | --- |
| 1 | `d1830_audio_rails()` | PMIC sibling LDOs 21–23 |
| 2 | tristar audio-path log | telemetry only |
| 3 | idempotence guard | `prepared_rate == rate` returns early |
| 4 | `cs42_d3280_state1_analog_on()` | OSOS's power-up, **not** the bootloader's |
| 5 | `cs42_d3280_state3_unfreeze()` | states 1 and 3 are a matched pair, in this order |
| 6 | `cs42_mailbox_reads()` | |
| 7 | `cs42l81_set_rate(rate)` | three-way dispatch |
| 8 | `cs42l81_output_path_enable()` | releases the hold `183138` raised |
| 9 | `cs42_570620_play_graph(1)` | graph is built at PLAY, after power-up |

**State 4 is a power-DOWN sequence and belongs only on the stop path.**

## Delete from ours

**The ASP lock.** `cs42l81_asp_lock()` polls `0x002F & 0x40` — bit 6 — for
three attempts with rate reprogramming between them. The whole-image decomp
reads `0x002F` exactly **once**: the readiness poll in `sub_D3280(1)`, testing
**bit 7**. Bit 6 is never examined anywhere in the image. There is no ASP lock
handshake in this part. Linux replaced it with `cs42_asp_status()` — one read,
logged, no gate, no retry. Remove `cs42l81_asp_lock()`,
`cs42l81_asp_is_locked()`, `cs42_asp_locked`.

**Headset detection.** `cs42_headset_sense()` and `cs42_headset_type`,
including the `0x0220` save/restore and the reads of `0x0227`, `0x000B`,
`0x0008`, `0x0009`, `0x0528`. Linux removed codec headset detection entirely;
this board does not use jack detect. Ours came from the "read but discard the
result" sweep — the right instinct applied to a function that is not on the
play path.

**The two-armed rate setup.** `cs42_serial_setup()` branches on `code == 12`
and then writes *both* `0x010B`/`0x010C` **and**
`0x0121`/`0x0122`/`0x0130`/`0x0131`. That is two arms of a three-way dispatch
running at once. See below.

## Fix: `sub_D34C0` is a three-way branch

Dispatches on `MEMORY[0x892A038]`, not on the rate:

| condition | arm | Linux function |
| --- | --- | --- |
| `& 0x08` and `& 0x20` | short: `0x000F` / `0x012F` only | `cs42_d34c0_short()` |
| `& 0x08` and not | long: `0x0121/0x0122/0x0130/0x0131` + `0x0222/3/4` | `cs42_d34c0_long()` |
| otherwise | — | `cs42_183138_set_rate()` |

They are alternatives. `183138` uses `0x010B`/`0x010C` where the long arm uses
`0x0223`/`0x0224` — different blocks for the same job, so a rate written by
one arm lands where the current mode does not read it. And the long arm's
closing `0x0220` bit-5 drop releases a hold `183138` had just raised.

Two things `183138` does at its end that ours does not: mute to code `0x40`
(−90 dB), and raise the `0x0220` bit-5 hold. Neither is an oversight to
"correct" — `sub_D2F64` clears `0x0220` mask `0x28` as its second write, so
output-path-enable is what releases the hold, and user volume is reapplied at
the end of prepare. **Hence prepare must run set-rate before
output-path-enable.**

Track `mode38` as a real field, written where the decomp writes it (`0x28` in
`D3280(3)`, `v3 & 0x28` in `sub_D2F64` — zero for modes 271 and 6 alike).

## Add: the `sub_D3280` state machine

We have none of this.

- **State 1 is OSOS's, not the bootloader's.** Five registers differ:
  `0x0225 = 0x33` not `0x19`; `0x0220` mask `0x78 = 0x78` not `0x50`; plus
  `0x0229 = 0x40` and `0x0075` mask `0x80 = 0` which the bootloader never
  writes — and it leaves `0x0007` bit 6 **set** where the bootloader leaves it
  clear. Ours is the bootloader's.
- **States 1 and 3 bracket the codec clock gate, in that order.** State 1 sets
  `0x0006` bit 6 and `0x0007` bit 6 then drops the clock; state 3 restores the
  clock and clears them. `0x0075` bit 7 inverts the same way.
- **State 4 clears what state 1 established** — the analog enable, the 2v5
  rail to `0x0E`, `0x0225 = 0x00` over state 1's `0x33` — and writes
  `0x0223`/`0x0224 = 0x08`/`0x09`, the *native* rate pair, clobbering the SRC
  pair a 44.1 kHz stream was just given. Stop path only.
- **`sub_D2F64` is computed, not fixed.** Mode 271 ("on") must not write
  `0x000D[1:0] = 0`; stock only does that for mode 6 ("off").

## Add: the play graph and its builders

Also entirely absent from ours:

- `cs42_build_play_graph_static()` — 80 writes, verified position-for-position
  against the image
- `cs42_build_play_graph_retailos()`, `cs42_570620_play_graph()`
- `cs42_174e7c_compute_taps()` — including the +2/+1 and the 160 divisor
- `cs42_174e38_program_range()`, `cs42_165bd4_build_slots()`,
  `cs42_write_slot3()`
- `cs42_domain33_enter()` / `_exit()`, `cs42_graph_begin()` / `_end()`
- `cs42_5706f4_route_state()` — writes in-memory state only, as `sub_5706F4` does
- `cs42l81_apply_mode_271()` / `cs42l81_apply_mode_6()`
- `cs42_2v5_backpower_up()` — `sub_400330` raises the 2.5 V analog backpower
  rail; it is **not** a volume setter, which is how it was first read

## `0x0006` bit 0 is a one-shot, not a level

Six writes to `0x0006` exist in the whole image and nothing re-sets bit 0
after the table. It is a one-shot start whose completion `0x002F` bit 7
reports. Anything that "restores" it is putting back a bit stock leaves clear.

## Infrastructure

- **Six-byte write frame** for `0x0225`, `0x0227`, `0x0229` —
  `cs42l81_write_wide()` gated by `cs42l81_reg_wants_wide_write()`.
- **Advertise 44.1 and 48 kHz only.**
- Volume: `cs42l81_vol_to_db()` / `db_to_code()` / `set_output_gain()`.
  Ours reapplies user volume at the end of prepare, which is correct — keep it.

## Cross-driver work this implies

Not confined to the codec:

- **`pcm-s5l8740.c` must call `cs42l81_pre_iis_start()` before starting IIS
  and `cs42l81_post_iis_start()` after it is running.** The unmute happens in
  the *post* hook — unmuting before the interface runs is one way to get
  silence.
- **`pmu-nano7g.c` needs `d1830_audio_rails()`** — sibling LDOs 21–23. Without
  it the analog side is untrimmed.

## Order of work

1. Delete the ASP lock and headset detection. Pure removal; also takes ~2 s of
   pointless polling off every playback start.
2. Add the `D3280` state machine, correct 1→3 order, state 4 on stop only.
3. Replace `cs42_serial_setup()` with the real three-way dispatch on `mode38`.
4. Add the play graph and builders.
5. Wire `pre_iis`/`post_iis` into `pcm-s5l8740.c`; add `d1830_audio_rails()`.
6. Six-byte frames, rate advertisement, volume plumbing.

Steps 1 and 2 are independent. Step 3 depends on 2 — the arm chosen depends on
state `D3280(3)` sets. Step 5 can be done in parallel and is the one most
likely to be silently wrong, because nothing about it produces an error.

## What this cannot promise

Linux is "not perfectly working, but a whole lot better shape". This brings us
to a known-better position, not a known-good one. Two things stay unverified
until sound comes out of this port:

- Whether `0x002F` bit 7 ever drops after the graph is built. If it does, the
  graph has to move ahead of the power-up. That is a measurement, not an
  assumption in either direction.
- Whether the mode-38 dispatch picks the same arm here, since our play path
  reaches it from Rockbox's PCM layer rather than from OSOS's.

Both should be instrumented rather than assumed. Nothing in the audio path has
run on this hardware yet — see `docs/N31-DRIVER-STATUS.md`.
