# h264_wmdetect.c — Line-by-Line Guide

A complete walkthrough of [libavcodec/h264_wmdetect.c](libavcodec/h264_wmdetect.c), the decoder-side
watermark detector that recovers payloads embedded by the x264-side embedder
(`encoder/watermark.c` on the x264 `indee` branch).

# The big picture

The x264 embedder forced a magnitude relationship between two coefficients in each coded luma
block: for payload **bit 1**, `|A| > |B|`; for **bit 0**, `|B| > |A|` (A = zigzag position 4,
B = zigzag position 5). Which payload bit goes into which block was decided by the formula:

```
bit_index = (i_frame*40503 + mb_xy*16 + blk) mod payload_bits
```

The detector's job is to run inside the H.264 decoder, look at those same two coefficients in
every coded block, decide "this block says 1" or "this block says 0", and **vote**. Compression
noise means individual votes are unreliable, but majority voting across thousands of blocks
recovers the payload.

There are two hard problems the file solves:

1. **The decoder sees dequantized coefficients, not raw levels.** The comparison must happen on
   raw quantized levels (what x264 wrote), and positions 4 and 5 get *different* dequant
   multipliers, so you can't compare the dequantized values directly. Solved by `wmd_level()`
   (exact division to undo dequantization).
2. **The decoder decodes in decode order, but `i_frame` in the formula is display order.** At
   decode time you don't yet know a frame's display rank. Solved by accumulating votes *per
   frame* without the frame term, then at the very end sorting frames into display order and
   applying the frame term as a **histogram rotation**.

Three hooks wire it into the decoder ([libavcodec/h264_wmdetect.h](libavcodec/h264_wmdetect.h)):

- `ff_h264_wmdetect_frame_start()` — from `h264_slice.c` (h264_field_start), once per picture
- `ff_h264_wmdetect_mb()` — from `h264_mb.c` (ff_h264_hl_decode_mb), once per macroblock, after
  residual decode, before the IDCT destroys `sl->mb`
- `ff_h264_wmdetect_finalize()` — from `h264dec.c` (h264_decode_end), when the decoder closes

---

# Lines 1–25: File header comment

Documents the contract with the embedder: the magnitude relation, the bit-index formula, and two
subtle facts you'd otherwise trip on:

- **Line 11–12**: `mb_xy = mb_y*mb_width + mb_x` uses *x264's* stride. FFmpeg internally uses a
  stride of `mb_width + 1` (an extra padding column) for its own `mb_xy`, so the detector must
  recompute the x264-style index rather than reuse `sl->mb_xy`.
- **Line 14**: `sl->mb` holds **dequantized** coefficients in **transposed** raster order (FFmpeg
  transposes the whole coefficient block relative to the spec so its IDCT can be written
  column-first). This is why the code later uses `h->zigzag_scan` (FFmpeg's transposed table)
  instead of the spec table — the transpositions cancel out.

# Lines 27–37: Includes

```c
#include <stdio.h>      /* fopen/fprintf for the CSV dump */
#include <stdlib.h>     /* getenv, atoi, strtol, qsort */
#include <string.h>     /* memset */
#include <inttypes.h>   /* PRIu64 printf macros for uint64_t */

#include "libavutil/mem.h"      /* av_mallocz, av_malloc_array, av_free */
#include "libavutil/thread.h"   /* AVMutex — decoder may be multi-threaded */
#include "h264dec.h"            /* H264Context, H264SliceContext, scan8 */
#include "mpegutils.h"          /* MB_TYPE_* flags used by IS_INTRA() etc. */
#include "h264_wmdetect.h"      /* our own prototypes */
```

# Lines 39–43: Constants

| Line | Constant | Meaning |
|---|---|---|
| 39 | `WMD_MAX_PAYLOAD_BYTES 64` | hard cap on payload size (matches embedder's `WM_MAX_PAYLOAD_BYTES`) |
| 40 | `WMD_MAX_BITS` | 64×8 = 512 bits — sizes the vote arrays |
| 41 | `WMD_DEFAULT_PAYLOAD "DEADBEEF"` | same default hex payload as the embedder |
| 42 | `WMD_FRAME_MIX 40503u` | the embedder's frame multiplier — must match `watermark.c`'s `40503u` exactly or the rotation at finalize scrambles everything |
| 43 | `WMD_DEBUG_MAX_LINES 64` | caps debug spam at 64 log lines |

# Lines 45–52: `WMDFrame` — one vote bucket per decoded picture

```c
typedef struct WMDFrame {
    uint64_t gen;                   /* which IDR generation this frame belongs to */
    int      poc;                   /* Picture Order Count (display-order key) */
    uint32_t w1[WMD_MAX_BITS];      /* vote weight for "bit=1", per local index */
    uint32_t w0[WMD_MAX_BITS];      /* vote weight for "bit=0" */
    uint64_t votes;                 /* total votes this frame (stats) */
    struct WMDFrame *next;          /* singly-linked list of all frames */
} WMDFrame;
```

Why `(gen, poc)` as the identity? **POC** orders frames for display, but it *resets at every
IDR*. So POC alone can collide across GOPs. The detector counts IDRs (`gen_counter`) and uses the
pair — sorting by `(gen, poc)` reproduces the exact display order across the whole stream, which
becomes `i_frame`.

`w1`/`w0` are indexed by the **local** bit index `(mb_xy*16 + blk) mod bits` — i.e., the
embedder's formula *without* the frame term, which isn't known yet.

# Lines 54–71: Global state + mutex

```c
typedef struct {
    int inited;            /* wmd_init() has run */
    int enabled;           /* X264_WMD_ENABLE=1 was set */
    int debug;             /* X264_WMD_DEBUG */
    int weight_mode;       /* X264_WMD_WEIGHT: weight votes by margin */
    int min_nz;            /* X264_WMD_MIN_NZ: min nonzero coeffs per block */
    int bits;              /* payload length in bits */
    int have_ref;          /* we know the true payload -> can compute BER */
    uint8_t ref_payload[WMD_MAX_PAYLOAD_BYTES];
    FILE *dump;            /* optional CSV dump file */
    uint64_t gen_counter;  /* increments at each IDR */
    WMDFrame *frames;      /* linked list head */
    uint64_t n_frames, n_votes, n_skipped, n_field_skipped;  /* stats */
    uint64_t n_debug_lines;
} WMDState;

static WMDState wmd;
static AVMutex wmd_mutex = AV_MUTEX_INITIALIZER;
```

One process-wide singleton (`static wmd`), protected by a mutex because FFmpeg's H.264 decoder
can run **frame threads and slice threads** — multiple threads can be inside `wmd_vote_block()`
simultaneously.

# Lines 73–92: Small helpers

**`wmd_env_int` (73–77)**: read an integer environment variable with a default. Same helper the
embedder has.

**`wmd_parse_payload` (79–92)**: parse a hex string like `"DEADBEEF"` into bytes.

- Line 82: loop while there are *two* hex chars left and buffer space remains.
- Lines 83–87: copy two chars into a NUL-terminated 3-byte string, `strtol(..., 16)`; if `*end`
  isn't NUL, a char wasn't valid hex → return 0 (reject).
- Line 91: succeed only if the string was fully consumed (`!hex[0]`, i.e. even length) and
  non-empty. Returns byte count.

# Lines 94–135: `wmd_init` — one-time lazy configuration

Called under the mutex, from the first `frame_start`.

- Line 100–101: zero everything, mark inited.
- Line 102–104: read `X264_WMD_ENABLE`; **default is 0 (off)**. If off, stop here — this is what
  makes the whole feature inert in a normal ffmpeg build.
- Lines 105–107: read tuning knobs. `min_nz` default 2 matches the embedder's skip rule, so the
  detector mostly looks at the same blocks the embedder actually touched.
- Lines 109–113: find the reference payload: `X264_WMD_PAYLOAD`, else `X264_WM_PAYLOAD` (the
  *embedder's* variable — convenient when you run encode and decode in one shell), else
  `"DEADBEEF"`.
- Lines 114–118: parse it; if invalid, fall back to the default. `bits = 8*n`.
- Line 119: `have_ref = 1` — we know the ground truth, so BER can be reported.
- Lines 121–127: `X264_WMD_BITS` lets you override the payload *length* when you know how many
  bits were embedded but not their values. If it disagrees with the parsed payload's length, the
  reference is useless → `have_ref = 0` (blind recovery only). Clipped to [1, 512].
- Lines 129–134: `X264_WMD_DUMP=<path>` opens a CSV file and writes a header row — one row per
  vote later, for offline analysis.

# Lines 137–174: `ff_h264_wmdetect_frame_start` — per-picture setup

```c
h->wm_frame = NULL;
```

Line 141: default to "no watermark bucket" — this is the flag `ff_h264_wmdetect_mb` checks, so
setting it NULL first means any early exit below disables detection for this picture.

- Lines 143–145: take the mutex; lazy-init on first call.
- Line 146: bail if disabled or there's no current picture.
- Lines 148–151: **progressive frames only.** `picture_structure != PICT_FRAME` means field
  coding (top/bottom fields decoded separately); the embedder's mb geometry assumptions don't
  hold there, so such pictures are counted (`n_field_skipped`) and ignored.
- Lines 153–154: if this picture is an IDR, bump `gen_counter` — starting a new POC epoch (see
  the `(gen, poc)` discussion above).
- Lines 158–160: look for an existing bucket with the same `(gen, poc)`. Normally there isn't
  one; the comment notes that if a duplicate ever occurred it would harmlessly merge votes.
- Lines 161–170: allocate a zeroed bucket (`av_mallocz`), stamp its identity, push onto the
  linked list, count it. On allocation failure just skip this frame — detection is best-effort,
  decoding must never break.
- Line 171: publish the bucket in `h->wm_frame` (a `void *` added to `H264Context` in
  h264dec.h) so the per-macroblock hook can find it without touching global state.

# Lines 176–182: `wmd_level` — undoing dequantization (the key math)

FFmpeg dequantizes each coefficient as:

```
dq = (level * qmul + 32) >> 6        // i.e. round(level * qmul / 64)
```

The embedder compared **levels**, and positions 4 and 5 have **different `qmul`** values (they're
in different positions of the scaling matrix / dequant tables), so comparing `dq` values directly
would be wrong. We must invert back to `level`:

```c
static inline int wmd_level(int dq, uint32_t qmul)
{
    int64_t num = (int64_t)dq * 128 + (dq >= 0 ? (int64_t)qmul : -(int64_t)qmul);
    return (int)(num / (2 * (int64_t)qmul));
}
```

This computes `round(dq * 64 / qmul)` — nearest-integer division written as
`(dq*128 ± qmul) / (2*qmul)`, with the `±` making the rounding symmetric for negative values
(C division truncates toward zero). 64-bit intermediates prevent overflow.

Why is this **exact** and not just approximate? Consecutive levels map to dequantized values that
are `qmul/64` apart. If `qmul > 64`, that spacing is > 1, so each `dq` value corresponds to
exactly one possible `level`, and rounding recovers it perfectly. The smallest `qmul` for a flat
matrix is 640 (line 177's comment), so in practice this is a lossless inversion.

# Lines 184–230: `wmd_vote_block` — examine one block, cast one vote

Parameters: the coefficient buffer for this block (`coef`, 16 or 64 entries), the dequant row
`qmul`, the two positions `pos_a`/`pos_b` (already translated to transposed-raster indices by the
caller), the x264-convention `x264_mb_xy`, and the block index `blk`.

- Lines 192–193: count nonzero coefficients. The comment notes `level == 0 ⇔ dq == 0` (dequant
  of 0 is 0), so counting on dequantized values is equivalent to counting on levels.
- Lines 194–197: recover both levels and take absolute values `aa`, `ab`.
- Lines 199–204: **skip rules** — mirror of the embedder's. Too little energy
  (`nnz < min_nz`: the embedder skipped these too), or `aa == ab` (a tie carries no information;
  also covers both-zero). Skips are counted under the mutex and the function returns without
  voting.
- Line 206: the decision itself: `bit = aa > ab` — literally "is |A| > |B|", the embedder's
  encoding of bit 1.
- Line 207: vote weight. Default 1 vote per block. With `X264_WMD_WEIGHT=1`, the vote is weighted
  by the margin `|aa−ab|` (capped at 8): a coefficient pair separated by a wide gap is much more
  likely to have survived intact than one separated by 1.
- Line 208: the **local index**: `(mb_xy*16 + blk) mod bits` — the embedder's formula minus the
  frame term. All arithmetic in `uint32_t` to reproduce the embedder's exact overflow/modulo
  behavior.
- Lines 210–229: under the mutex: add the weight to `w1[local]` or `w0[local]`, bump counters,
  optionally append a CSV row (gen, poc, mb coordinates, block, transform size, QP, both
  recovered levels), and optionally print a human-readable debug line (capped at 64).

# Lines 232–270: `ff_h264_wmdetect_mb` — per-macroblock dispatch

Called from `ff_h264_hl_decode_mb()` for every macroblock, at the one moment where `sl->mb`
still holds the residual coefficients (they're consumed/cleared by the IDCT right after).

- Line 237: bail fast if there's no bucket for this frame (feature off, field picture, alloc
  failure), if `h->pixel_shift` is set (high bit depth — coefficients would be 32-bit, the
  `int16_t` cast would be wrong), or `qscale == 0` (QP 0 / lossless path uses different
  scan/dequant behavior).
- Lines 240–242: fetch the macroblock type; skip `IS_INTRA_PCM` (raw samples, no coefficients at
  all) and `IS_INTERLACED` (MBAFF field macroblocks — wrong geometry, same reason as field
  pictures).
- Line 245: `list = IS_INTRA ? 0 : 3` — selects the dequant table: in FFmpeg's PPS tables,
  index 0 is intra-luma and index 3 is inter-luma. We only ever handle luma (the embedder only
  touched plane 0).
- Line 246: **rebuild the x264-style `mb_xy`** as `mb_y * mb_width + mb_x` — remember FFmpeg's
  own `sl->mb_xy` uses stride `mb_width+1` and would desynchronize the bit index.

**8×8 transform branch (lines 248–257):**

- Line 248: taken when the mb uses the 8×8 DCT *and is not I16x16*. The extra I16x16 guard is
  defensive: FFmpeg can carry the `MB_TYPE_8x8DCT` flag on I16x16 macroblocks in
  `cur_pic.mb_type` (it's ORed in for context/deblocking purposes), but an I16x16 residual is
  always 4×4 blocks.
- Lines 250–251: `pos_a/pos_b = h->zigzag_scan8x8[4]/[5]` — FFmpeg's *transposed* 8×8 zigzag
  table, giving the buffer offsets of zigzag positions 4 and 5 inside the transposed
  64-coefficient block. (This is where the transpositions cancel: transposed table into
  transposed buffer = the same physical coefficients x264 modified.)
- Lines 252–256: loop over the four 8×8 blocks of the macroblock. `sl->non_zero_count_cache`
  stores per-4×4 nonzero counts in the `scan8[]` layout (an 8-wide grid); an 8×8 block covers
  four cache entries at offsets `0, 1, 8, 9` from its top-left (`scan8[4*i8]`). If any is
  nonzero the block is coded → vote on `&sl->mb[64*i8]` (each 8×8 block occupies 64 consecutive
  coefficients) with `count=64` and `blk=i8` (0–3, matching x264's 8×8 block numbering).

**4×4 branch (lines 258–269):**

- Covers intra 4×4, I16x16 AC, and inter 4×4 residuals — sixteen 16-coefficient blocks in
  z-order. The comment explains the two facts that make I16x16 need no special casing: it uses
  the same dequant list 0 as other intra, and its DC coefficient hasn't been inserted into
  slot 0 of each block yet at this point in decoding (the luma DC Hadamard transform writes it
  later), so position 0 being empty doesn't disturb anything — positions 4 and 5 are AC anyway.
- Lines 263–264: same trick with `h->zigzag_scan[4]/[5]` (transposed 4×4 table; these land on
  the same physical cells x264's raster positions 5 and 2 refer to).
- Lines 265–268: for each of the 16 blocks, if its nonzero count cache entry says "coded", vote
  on `&sl->mb[16*i]` with `blk=i` (0–15, x264's z-order block index — same ordering, so the
  indices agree).

# Lines 272–279: `wmd_frame_cmp` — display-order comparator

qsort comparator over `WMDFrame*`: first by IDR generation, then by POC within a generation.
The sorted position of each frame **is** its display-order index, i.e. the embedder's `i_frame`.

# Lines 281–371: `ff_h264_wmdetect_finalize` — aggregate, rotate, decode, report

Runs once when the decoder is closed.

**Setup (284–307):**

- Lines 284–289: `g1[]`/`g0[]` are the *global* vote histograms (64-bit — sums of many 32-bit
  per-frame counters); `recovered[]` the output payload; BER counter; confidence trackers.
- Lines 291–296: under the mutex; if never enabled or no frames were seen, just reset state and
  return silently.
- Lines 299–307: build an array of frame pointers from the linked list and qsort it with the
  comparator → `order[rank]` is the frame displayed at time `rank`.

**The rotation trick (lines 309–318)** — the heart of the file:

The embedder's global index was `(i_frame*40503 + local_part) mod bits`, and the detector stored
votes under `local = local_part mod bits`. By modular arithmetic:

```
global = (local + (i_frame * 40503) mod bits) mod bits
```

So for each frame, compute `rot = (rank * 40503) mod bits` (line 312) and add its histograms into
the global ones **shifted by `rot`** (lines 314–317): `g1[(j + rot) % bits] += f->w1[j]`. One
cheap O(bits) pass per frame replaces having to know the display index during decode.

**Stats line (320–323):** total frames/votes/skips, with a note if field pictures were ignored.

**Bit decisions (325–346):** for each payload bit `i`:

- Line 327: **majority vote** — `bit = v1 > v0`.
- Lines 328–329: set the bit in `recovered[]`, MSB-first within each byte
  (`1 << (7 - (i & 7))`), matching the embedder's bit packing.
- Lines 330–336: confidence = winning votes / total votes for that bit (0.5 = coin flip,
  1.0 = unanimous). Track minimum and mean; count bits that received **no votes at all** (those
  default to 0 and are pure guesses).
- Lines 337–340: if we have the reference payload, extract the true bit the same MSB-first way
  and count mismatches into `ber`.
- Lines 341–345: per-bit log line (always for payloads ≤ 64 bits, otherwise only in debug mode):
  decided bit, winning vs losing vote counts, confidence.

**Final report (348–362):**

- Lines 349–357: hex-encode `recovered[]` and print the headline result: recovered payload, bit
  count, min/mean confidence, number of unvoted bits.
- Lines 359–362: if the reference is known, print the **Bit Error Rate** — `ber/bits` as a
  percentage. This is the number to watch in experiments: 0% = perfect recovery.

**Cleanup (364–370):** free every frame bucket and the order array, close the dump file, and
`memset(&wmd, 0, ...)` so a subsequent decoder instance in the same process starts from a clean
slate (it will re-run `wmd_init`, re-reading the environment).

---

# Mental model of one run

```
X264_WMD_ENABLE=1 ffmpeg -i watermarked.mp4 -f null -
        │
frame_start:  picture arrives → bucket keyed (gen, poc), h->wm_frame set
        │
wmdetect_mb:  per mb → per coded luma block → recover levels at zigzag 4,5
              → |A| vs |B| → vote into bucket at (x264_mb_xy*16+blk) mod bits
        │
finalize:     sort buckets by (gen, poc) → rank = i_frame
              → rotate each histogram by (rank*40503) mod bits → sum
              → majority per bit → payload + confidence + BER
```

Invariants worth keeping in mind when modifying either side: the constant **40503**, the pair
positions (**zigzag 4/5**), and the skip rules (`min_nz`, plane 0 only) must stay mirrored
between `watermark.c` and this file; and everything here is read-only with respect to decoding —
the only writes are to the detector's own state.

# Environment variables (detector)

| Variable | Default | Effect |
|---|---|---|
| `X264_WMD_ENABLE` | 0 | master switch; everything is inert without `=1` |
| `X264_WMD_DEBUG` | 0 | verbose per-vote logging (capped at 64 lines) + per-bit lines for large payloads |
| `X264_WMD_WEIGHT` | 0 | weight votes by magnitude margin (capped at 8) instead of 1 each |
| `X264_WMD_MIN_NZ` | 2 | min nonzero coefficients for a block to vote (mirror of embedder) |
| `X264_WMD_PAYLOAD` | — | reference payload hex (falls back to `X264_WM_PAYLOAD`, then `DEADBEEF`) |
| `X264_WMD_BITS` | payload length | override bit count for blind recovery (disables BER if mismatched) |
| `X264_WMD_DUMP` | — | path for a per-vote CSV dump |
