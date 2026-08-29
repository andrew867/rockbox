# A2DP on the iPod nano 7G — future work

Status: **not started.** This is a record of what exists, what does not, and
what porting it would actually involve, so the next person does not have to
rediscover the scope.

## The short version

Rockbox has no Bluetooth stack at all. Not a partial one, not a disabled one —
none. A search of `firmware/`, `apps/` and `lib/` for `a2dp`, `avdtp`, `l2cap`,
`rfcomm` and `sbc_encode` returns exactly one hit, and it is an unrelated iAP
header (`firmware/usbstack/iap/libiap/spec/lingoes/general/ipod-notification.h`).

So A2DP here is not a matter of wiring something up. There is nothing to wire.

## What we do have

`firmware/target/arm/s5l8740/bcm2078-s5l8740.c` brings the BCM2078 up and
speaks HCI to it over UART: `bcm2078_init()` powers the rail and pad, resets
the controller, negotiates the vendor baud rate, and `hci_send_cmd()` /
`hci_wait_complete()` exchange HCI command and event packets.

That is real and it is genuinely useful — it is what makes the FM tuner work,
since `bcm2078_fm_cmd()` reaches the FM block through HCI vendor commands. But
for audio it is the bottom rung of a tall ladder.

| Layer | Status |
| --- | --- |
| HCI over UART | present |
| L2CAP | none |
| SDP | none |
| AVDTP | none |
| A2DP profile | none |
| SBC encoder | none |
| Pairing / SSP | none |
| PCM routing to a Bluetooth sink | none |

## Why this is a project and not an afternoon

Every layer above HCI has to be written or ported. Beyond the protocol work
there are two things that are easy to underestimate:

**Real-time SBC encoding.** A2DP source means encoding PCM to SBC continuously
while also decoding whatever is playing, on a Cortex-A5, without underrunning
the link. Rockbox is fixed-point throughout and has no SBC encoder.

**Audio routing.** Rockbox's PCM path assumes it ends at the codec. A
Bluetooth sink needs the PCM diverted, encoded and pushed through L2CAP on a
timer, which is a different shape from the existing output path.

## Recommended approach, if it is taken on

**Port BTstack rather than writing a stack.** It is designed for embedded
targets, has a permissive licence, and ships both an A2DP source profile and
an SBC encoder. The work becomes writing an HCI transport that sits on
`bcm2078-s5l8740.c`, plus the audio plumbing — which is a far smaller and much
better-defined job than implementing L2CAP, SDP and AVDTP from scratch.

## Priority

Low, for now. The things that make this a usable music player are much closer:
audio out through the CS42L81, the touchscreen, and plugins. A2DP is a good
milestone *after* the device plays music through its own headphone jack — not
instead of it.
