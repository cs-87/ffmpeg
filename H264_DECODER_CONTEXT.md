# libavcodec's H.264 Decoder — Context for an x264 Developer

Companion to [H264_WMDETECT_GUIDE.md](H264_WMDETECT_GUIDE.md). That file explains *what the
detector does*; this one builds the *background knowledge* it assumes — how the libavcodec
H.264 decoder is put together, using what you already know from x264 as the anchor.

Everything links into this checkout, so read it with the code open.

---

## 1. The mindset shift from x264

Most of the initial confusion is not missing knowledge — it's that four assumptions you've
internalized from x264 are inverted on the decoder side:

1. **x264 is one program; libavcodec is a framework.** In x264, `x264_t` *is* the program.
   In FFmpeg, the H.264 decoder is one plugin among hundreds behind a generic interface
   (`AVCodecContext` → `FFCodec` → callbacks). The outermost layers of h264dec.c are
   framework plumbing, not H.264 logic — you can skim them.
2. **The encoder chooses; the decoder obeys.** x264 is full of *decisions* (RD search, mode
   decision, rate control). The decoder has zero decisions — it's a straight-line
   interpreter of the bitstream. Every branch you see corresponds to a syntax element, not
   a heuristic. This makes the decoder conceptually *simpler* than x264, even though the
   code looks denser.
3. **The decoder must handle everything.** x264 only emits the subset of H.264 it likes
   (e.g. it never emits some exotic field/MBAFF combinations you didn't enable). The decoder
   must accept *every legal stream*: MBAFF, field coding, 4:4:4, 9–14 bit, lossless,
   redundant slices, broken streams needing error concealment. A large share of the code
   you're wading through handles cases your watermark pipeline will never produce. Learning
   which 20% is your hot path is most of the battle — that's what this doc is for.
4. **The decoder doesn't know display order until late.** x264's lookahead reorders frames
   *before* encoding, so `i_frame` (display order) is just there. The decoder receives
   frames in *decode* order and only learns display order through POC and the DPB
   reordering buffer. This single fact is why the detector needs its `(gen, poc)` +
   rotate-at-finalize design.

---

## 2. File map — which files matter to you

The decoder is split by *function*, not by layer. For the watermark work, the files fall
into three tiers:

**Tier 1 — your hot path (read these):**

| File | Role | x264 analogue |
|---|---|---|
| [h264dec.h](libavcodec/h264dec.h) | All the structs: `H264Picture`, `H264SliceContext`, `H264Context` | `common/common.h` (`x264_t`) |
| [h264dec.c](libavcodec/h264dec.c) | Top level: packet in → NAL loop → frame out | `encoder/encoder.c` outer shell |
| [h264_slice.c](libavcodec/h264_slice.c) | Slice header parsing, per-picture setup, **the MB loop** | `encoder/slice*.c` + parts of `encoder/encoder.c` |
| [h264_cavlc.c](libavcodec/h264_cavlc.c) | CAVLC entropy decode → fills `sl->mb` | inverse of `encoder/cavlc.c` |
| [h264_cabac.c](libavcodec/h264_cabac.c) | CABAC entropy decode → fills `sl->mb` | inverse of `encoder/cabac.c` |
| [h264_mb.c](libavcodec/h264_mb.c) + [h264_mb_template.c](libavcodec/h264_mb_template.c) | "Reconstruct one MB": prediction + IDCT + write pixels | inverse of `encoder/macroblock.c` |
| [h264_ps.c](libavcodec/h264_ps.c) / [h264_ps.h](libavcodec/h264_ps.h) | SPS/PPS parsing, **builds the dequant tables** | `encoder/set.c` + `common/quant.c` table init |

**Tier 2 — read when a concept confuses you:**

| File | Role |
|---|---|
| [h2645_parse.c](libavcodec/h2645_parse.c) | Splits a packet into NAL units, strips emulation-prevention bytes (shared with HEVC) |
| [h264_refs.c](libavcodec/h264_refs.c) | Reference list construction, MMCO — the DPB bookkeeping |
| [h264_parse.c](libavcodec/h264_parse.c) | POC computation (`ff_h264_init_poc`), shared header helpers |
| [h264_picture.c](libavcodec/h264_picture.c) | `H264Picture` alloc/ref/unref |
| [h264data.c](libavcodec/h264data.c) | Static tables (the *untransposed* zigzag lives here as `ff_zigzag_scan`) |

**Tier 3 — safely ignore for this project:** `h264_loopfilter.c` (deblocking — runs after
your hook, on pixels), `h264_direct.c` (B-frame direct MV derivation), `h264_mvpred.h`
(MV prediction), `h264pred*.c` (intra prediction DSP), `h264qpel*/h264chroma*` (subpel
motion compensation DSP), `h264idct*` (IDCT DSP), `h264_sei.c`, `h264_parser.c` (the
*parser* is a separate lightweight component for packetization, not the decoder).

> Naming rule of thumb: `*_ps` = parameter sets, `*dsp`/`*pred`/`*qpel`/`*idct` = SIMD-able
> leaf math (function-pointer tables, like x264's `h->quantf`/`h->predict_4x4`), everything
> else = control flow.

---

## 3. The three structs (vs. `x264_t`)

x264 puts everything in one `x264_t` (with per-thread copies). libavcodec splits the same
state three ways, and knowing *which struct holds what* is 80% of reading fluency:

### [`H264Context`](libavcodec/h264dec.h#L338) — one per decoder instance
The global, per-stream state: dimensions, active SPS/PPS (`h->ps`), the DPB
(`h->DPB`), POC machinery (`h->poc`), the scan tables (`h->zigzag_scan`,
`h->zigzag_scan8x8` at [h264dec.h:429-430](libavcodec/h264dec.h#L429-L430)),
`h->mb_width/mb_height/mb_stride`, and the output-reordering queue
(`h->delayed_pic`, [h264dec.h:473](libavcodec/h264dec.h#L473)). Your `h->wm_frame` lives
here. Roughly: the parts of `x264_t` that aren't `h->mb`.

### [`H264SliceContext`](libavcodec/h264dec.h#L178) — one per *slice-decoding thread*
Everything needed to decode MBs of one slice: the bitstream reader / CABAC engine, current
MB position (`sl->mb_x/mb_y/mb_xy` at [h264dec.h:232](libavcodec/h264dec.h#L232)),
`sl->qscale` (QP), the neighbor caches, and — the buffer your detector reads —

```c
DECLARE_ALIGNED(16, int16_t, mb)[16 * 48 * 2];   // h264dec.h:307
```

That sizing decoded: 16 coefficients × 48 blocks (16 luma + 16 Cb + 16 Cr — sized for
4:4:4, where chroma is full 16-block planes) × 2 because at high bit depth each
coefficient is 32-bit and stored in the same `int16_t` array (accessed via
`dctcoef_get/set`, [h264_mb.c:597](libavcodec/h264_mb.c#L597) — this is what
`h->pixel_shift` guards). This is the direct analogue of x264's `h->dct.luma4x4[]` /
`h->dct.luma8x8[]`, with two crucial differences covered in §5.

Why a separate struct at all? **Slice threading**: with `-threads N` on a multi-slice
stream, N `H264SliceContext`s decode different slices of the *same* picture concurrently,
all pointing at one `H264Context`. That's why the per-MB decode functions take `(h, sl)`
pairs and why `h` is `const` in them.

### [`H264Picture`](libavcodec/h264dec.h#L112) — one per frame in the DPB
The analogue of `x264_frame_t`: the pixel buffers (`f`, an `AVFrame`), plus per-MB
side arrays that outlive the slice decode — `mb_type[]`, `qscale_table[]`,
`motion_val[]`, and `field_poc[]/poc` ([h264dec.h:132](libavcodec/h264dec.h#L132)).
Note `h->cur_pic.mb_type[mb_xy]` is the *finalized* mb type your detector reads —
same info as `sl->mb_type` but stored per-picture (x264 equivalent: `frame->mb_type[]`
vs `h->mb.i_type`).

---

## 4. The call chain, top to bottom

This is the skeleton to hang everything on. One packet → one frame (usually):

```
h264_decode_frame()                        h264dec.c:1023   ← FFCodec callback
 └─ decode_nal_units()                     h264dec.c:587
     ├─ ff_h2645_packet_split()                             ← NALs out of the packet
     ├─ per NAL: SPS/PPS → h264_ps.c,  SEI → h264_sei.c
     ├─ slice NAL: ff_h264_queue_decode_slice()  h264_slice.c
     │    └─ first slice of a picture:
     │        h264_field_start()           h264_slice.c:1398
     │          ├─ POC computed, DPB refs built, gaps handled
     │          └─ ff_h264_wmdetect_frame_start()  ← HOOK 1  (h264_slice.c:1646)
     ├─ ff_h264_execute_decode_slices()    h264_slice.c:2779
     │    └─ avctx->execute(decode_slice)  ← thread fan-out for slice threads
     │        decode_slice()               h264_slice.c:2567
     │          └─ loop over MBs:
     │              ff_h264_decode_mb_cabac()   h264_cabac.c:1920   ┐ "parse":
     │              or ff_h264_decode_mb_cavlc() h264_cavlc.c:665   ┘ fill sl->mb + caches
     │              ff_h264_hl_decode_mb()      h264_mb.c:801       ← "reconstruct"
     │                ├─ ff_h264_wmdetect_mb()  ← HOOK 2  (h264_mb.c:808)
     │                └─ hl_decode_mb_simple_8() etc. → predict + IDCT + write pixels
     └─ finish picture → output reordering via h->delayed_pic
...
h264_decode_end()                          h264dec.c:352    ← decoder close
 └─ ff_h264_wmdetect_finalize()            ← HOOK 3  (h264dec.c:358)
```

### The two-phase MB decode — the single most important pattern

Every macroblock goes through exactly two phases, and your hook sits on the boundary:

- **Parse** (`ff_h264_decode_mb_cavlc/cabac`): consume bitstream, produce (a) `sl->mb` —
  the coefficient buffer, (b) the neighbor caches (§6), (c) `sl->mb_type`, MVs, etc.
  This is the mirror image of x264's `encoder/cavlc.c` `block_residual_write` path.
- **Reconstruct** (`ff_h264_hl_decode_mb` → [h264_mb_template.c](libavcodec/h264_mb_template.c)):
  intra prediction or motion compensation into the picture buffer, then IDCT-add of
  `sl->mb` on top ([h264_mb_template.c:163](libavcodec/h264_mb_template.c#L163) predict,
  [:190](libavcodec/h264_mb_template.c#L190) idct). Mirror image of x264's
  `x264_macroblock_encode`, run backwards.

The reconstruct phase *consumes* `sl->mb` (and the buffer is reused by the very next MB) —
which is precisely why the detector hook is the first line of `ff_h264_hl_decode_mb`
([h264_mb.c:808](libavcodec/h264_mb.c#L808)): it's the last moment the residual exists.

Note "hl" = high-level, an old naming convention meaning the pixel-reconstruction stage.
And the "simple/complex" split at [h264_mb.c:789-820](libavcodec/h264_mb.c#L789-L820):
`h264_mb_template.c` is included multiple times with different macros to generate
specialized variants — `hl_decode_mb_simple_8` (8-bit, no weird cases, fast path),
`_simple_16` (high bit depth), `_complex` (fields/MBAFF/PCM). Same
template-by-`#include` trick x264 uses for bit-depth builds, just at function granularity.

---

## 5. The coefficient path — where the detector's two big problems come from

x264's residual path keeps stages separate: DCT → quant (levels, spec raster order) →
zigzag scan at bitstream-write time. The decoder *fuses* the inverse stages into one step,
and this fusion is the source of both "hard problems" in the detector guide.

### 5a. Dequantization is fused into entropy decoding

There is no "inverse quant" pass. As `decode_residual`
([h264_cavlc.c:407](libavcodec/h264_cavlc.c#L407)) reads each level, it immediately
dequantizes and scatters it into the block:

```c
((type*)block)[*scantable] = ((int)(level[i] * qmul[*scantable] + 32)) >> 6;
```

([h264_cavlc.c:564-576](libavcodec/h264_cavlc.c#L564-L576); CABAC does the equivalent.)
So by the time anything downstream sees `sl->mb`, the raw levels x264 wrote are *gone* —
only `round(level * qmul / 64)` remains. `qmul` is a row of
`pps->dequant4_coeff[list][qp]` / `dequant8_coeff` ([h264_ps.h:138-139](libavcodec/h264_ps.h#L138-L139)),
built once at PPS-parse time in h264_ps.c from the scaling matrices — the same
`(quant scale × scaling-matrix) per position` tables as x264's `dequant4_scale`, just
pre-multiplied per QP. The `[list]` index: 0 = intra luma, 1/2 = intra chroma,
3 = inter luma, 4/5 = inter chroma.

Because positions 4 and 5 of the zigzag sit at different positions of the scaling matrix,
they can have different `qmul` — hence `wmd_level()` must divide the dequant back out
before comparing.

### 5b. Everything coefficient-shaped is transposed

FFmpeg's IDCT is written to process the block in transposed orientation. Rather than
transposing every block after decode, they transpose **the scan tables once** at init —
`init_scan_tables` ([h264_slice.c:754](libavcodec/h264_slice.c#L754)):

```c
#define TRANSPOSE(x) ((x) >> 2) | (((x) << 2) & 0xF)     // 4x4: swap row/col bits
h->zigzag_scan[i] = TRANSPOSE(ff_zigzag_scan[i]);
```

So `decode_residual` writes coefficient *zigzag-position-k* into the *transposed* raster
cell, and the whole pipeline downstream (IDCT, and your detector) lives in transposed
space. Consequence for the detector: to find zigzag positions 4/5 inside `sl->mb`, use
`h->zigzag_scan[4]/[5]` (the transposed table) — transposed table into transposed buffer
lands on the same physical coefficients x264 touched. Using the spec table
(`ff_zigzag_scan` from h264data.c) would silently read the wrong cells.

Also note `zigzag_scan8x8_cavlc`: 8×8 blocks in CAVLC are coded as four interleaved 4×4
runs, so there's a separate scan permutation for that path — but the *storage layout* of
the resulting block is the same, which is why the detector doesn't care which entropy
coder produced the stream.

### 5c. Layout of `sl->mb`

Luma: block `i` (z-order, 0–15) occupies `sl->mb[16*i .. 16*i+15]`; an 8×8-transform MB
occupies four 64-coefficient chunks at `sl->mb[64*i8]`. Same z-ordering as x264's
`h->dct.luma4x4[i]`, so `blk` indices agree between embedder and detector with no
translation. I16x16 is the one asymmetry: the 16 DC coefficients live in a *separate*
buffer (`sl->mb_luma_dc`) until the reconstruct phase runs the inverse Hadamard —
at hook time, slot 0 of each 4×4 block is simply absent/zero.

---

## 6. The cache system and the mb_xy trap

Good news: you already know this design. FFmpeg's per-MB neighbor cache is the same idea
as x264's `h->mb.cache` — an 8-wide scratch grid holding the current MB's data plus one
row/column of neighbor data, indexed by a `scan8[]` table so "left neighbor" = `-1` and
"top neighbor" = `-8` uniformly:

| | x264 | libavcodec |
|---|---|---|
| index table | `x264_scan8[]` (common.h) | `scan8[]` ([h264_parse.h:40](libavcodec/h264_parse.h#L40)) |
| nnz cache | `h->mb.cache.non_zero_count[]` | `sl->non_zero_count_cache[]` ([h264dec.h:294](libavcodec/h264dec.h#L294)) |
| filled by | `x264_macroblock_cache_load` | `fill_decode_caches` (h264_mvpred.h) |

The layouts differ in detail (lavc's grid is 15×8 covering luma + both chroma planes) but
the mental model transfers directly: `sl->non_zero_count_cache[scan8[blk]]` = "nnz of 4×4
block `blk` of the current MB". An 8×8 block covers cache offsets `{0, 1, 8, 9}` from
`scan8[4*i8]` — same 8-wide-grid arithmetic as x264.

**The one genuine trap** — the two projects define `mb_xy` differently:

- x264: `i_mb_stride = i_mb_width`, so `mb_xy = mb_y * mb_width + mb_x`.
- libavcodec: `h->mb_stride = h->mb_width + 1`
  ([h264_slice.c:1106](libavcodec/h264_slice.c#L1106)) — one padding column so neighbor
  lookups (`mb_xy - 1`, `mb_xy - mb_stride - 1`, the `slice_table`) never need bounds
  checks at the left/top picture edge.

So `sl->mb_xy` drifts away from x264's index by one per MB row. Any cross-codec identity
(like the watermark bit-index formula) must recompute `mb_y * mb_width + mb_x` from
`sl->mb_x`/`sl->mb_y` — which is exactly what the detector does.

---

## 7. Decode order vs. display order (POC, IDR, the DPB)

In x264 you never think about this: the lookahead hands the encoder frames already in
coding order and `i_frame`/`i_poc` are known attributes. The decoder gets the inverse
problem: frames arrive in decode order (I P B B P B B...) and display order must be
*reconstructed*.

The machinery:

- **POC (Picture Order Count)** — the display-order counter, parsed/derived per picture
  from slice-header syntax (`ff_h264_init_poc` in h264_parse.c, driven from
  `h264_field_start`). x264 writes `i_poc = 2 × (display index within the GOP-ish epoch)`
  under poc_type 0.
- **POC resets at every IDR.** An IDR starts a new epoch, so POC alone is ambiguous
  across the whole stream — two GOPs both contain POC 0. (This is why the detector pairs
  it with an IDR-generation counter.)
- **The DPB and output reordering** — decoded pictures park in `h->DPB`; `h->delayed_pic[]`
  ([h264dec.h:473](libavcodec/h264dec.h#L473)) holds decoded-but-not-yet-displayable
  frames, and the decoder releases the lowest-POC frame once it's safe
  (`next_output_pic`). This delay is bounded but means "frame just decoded" ≠ "frame
  number N in the output" at hook time.

Rule of thumb for reading: anything named `poc`, `refs`, `mmco`, `long_ref` is this
subsystem (h264_refs.c). For the watermark work you only need the concept, not the
MMCO details.

---

## 8. Threading — why the detector is built the way it is

FFmpeg's decoder has **two independent threading modes** (x264 has rough analogues to
both, on the encode side):

1. **Slice threading** (like x264 `--sliced-threads`): one picture, N slices decoded in
   parallel, each on its own `H264SliceContext`. All threads share one `H264Context`.
2. **Frame threading** (like x264's normal lookahead threading): N *complete
   `H264Context` clones*, each decoding a different frame simultaneously, synchronized by
   per-row progress waits. The cloning/sync glue is `ff_h264_update_thread_context`
   (registered at [h264dec.c:1158](libavcodec/h264dec.c#L1158)).

Consequences you can now read off the detector design:

- `ff_h264_wmdetect_mb` can be called concurrently from multiple threads (different
  slices *and* different frames) → global vote state must be mutex-protected.
- `h->wm_frame` must be *per-context* (a field in `H264Context`, cloned per frame
  thread), because "the current picture" differs per frame thread — a global "current
  bucket" pointer would race.
- Frames may *start* out of decode order across threads, so nothing can rely on callback
  ordering — identity must come from stream-derived values (`gen`, `poc`), never from
  "the Nth time frame_start was called". That's also why results are invariant across
  `-threads 1..8`.

ffmpeg defaults to frame threading with N ≈ cores; `-threads 1` serializes everything —
your first move whenever behavior looks nondeterministic.

---

## 9. Rosetta stone

| Concept | x264 | libavcodec h264 |
|---|---|---|
| The context | `x264_t *h` | `H264Context *h` + `H264SliceContext *sl` |
| Per-frame struct | `x264_frame_t` | `H264Picture` (wraps an `AVFrame`) |
| MB index stride | `mb_width` | `mb_width + 1` ⚠ |
| Current MB position | `h->mb.i_mb_x/y/xy` | `sl->mb_x/mb_y/mb_xy` |
| MB type | `h->mb.i_type` (enum) | `sl->mb_type` / `cur_pic.mb_type[]` (bitmask: `MB_TYPE_*`, tested via `IS_INTRA()`, `IS_8x8DCT()`… in mpegutils.h/h264dec.h) |
| Coefficient buffer | `h->dct.luma4x4[16][16]` — raw levels | `sl->mb[]` — **dequantized**, **transposed** ⚠ |
| Zigzag tables | spec order (`common/dct.c`) | transposed (`h->zigzag_scan*`) ⚠ |
| Dequant tables | `dequant4_scale` etc. | `pps->dequant4_coeff[list][qp][pos]`, fused in at entropy-decode time |
| nnz cache | `h->mb.cache.non_zero_count` + `x264_scan8` | `sl->non_zero_count_cache` + `scan8` |
| QP | `h->mb.i_qp` | `sl->qscale` |
| Entropy coding | `encoder/cavlc.c`, `encoder/cabac.c` | `h264_cavlc.c`, `h264_cabac.c` |
| MB encode/decode | `x264_macroblock_encode` | `ff_h264_hl_decode_mb` |
| SPS/PPS | `encoder/set.c` writes | `h264_ps.c` parses |
| Display order | known upfront (`i_frame`, lookahead) | reconstructed late (POC + DPB reordering) ⚠ |
| DSP dispatch | `h->quantf`, `h->predict_4x4`, `h->mc` | `h->h264dsp`, `h->hpc` (pred), `h->h264qpel` |
| Threads | sliced-threads / frame parallel lookahead | slice threads / frame threads (context clones) |

The four ⚠ rows are precisely the four things the detector has special code for.

---

## 10. A guided reading path

Ordered so each step gives you leverage for the next. Total ≈ a focused day.

1. **Structs first.** Read [h264dec.h:112-176](libavcodec/h264dec.h#L112-L176)
   (`H264Picture`), then skim `H264SliceContext` (:178) and `H264Context` (:338) — don't
   memorize, just note which struct owns what (§3). 30 min.
2. **The MB loop.** Read `decode_slice`
   ([h264_slice.c:2567-2770](libavcodec/h264_slice.c#L2567)) end to end. It's the decoder's
   equivalent of x264's slice-encode loop and is very readable: CABAC/CAVLC branch,
   `hl_decode_mb`, error handling, row progress reporting. 30 min.
3. **The residual decode** — your money read. `decode_residual` and its call sites in
   [h264_cavlc.c:407-620](libavcodec/h264_cavlc.c#L407) with x264's `encoder/cavlc.c`
   open beside it. Watch for: the fused dequant, the scantable indirection, the nnz
   bookkeeping. 1–2 h.
4. **Reconstruction.** `hl_decode_mb` in
   [h264_mb_template.c](libavcodec/h264_mb_template.c) (read the SIMPLE paths, ignore
   MBAFF/PCM branches) with `x264_macroblock_encode` beside it. Confirm for yourself
   where `sl->mb` gets consumed. 1 h.
5. **Per-picture setup.** `h264_field_start`
   ([h264_slice.c:1398](libavcodec/h264_slice.c#L1398)) — skim, keeping §7 in mind: find
   where POC is computed and where `wmdetect_frame_start` sits. 30 min.
6. **Dequant table construction.** `init_dequant_tables` + friends in
   [h264_ps.c](libavcodec/h264_ps.c) — connect `qmul` values back to the QP/scaling
   matrix math you know from x264. 30 min.
7. **Top shell, last.** `h264_decode_frame` / `decode_nal_units`
   ([h264dec.c:587-1080](libavcodec/h264dec.c#L587)) — now that the layers below are
   familiar, the plumbing reads itself. 30 min.

### Exercises (each < 1 h, each validates a chunk of the model)

1. **See the transposition.** In `init_scan_tables`, temporarily `av_log` the 16 entries
   of `h->zigzag_scan` next to `ff_zigzag_scan`. Verify by hand that entry 4 and 5 land
   where the detector expects.
2. **Recompute one vote by hand.** Run with `X264_WMD_DUMP=/tmp/votes.csv` and
   `X264_WMD_DEBUG=1`, pick one CSV row, and in gdb (`break ff_h264_wmdetect_mb`) inspect
   `sl->mb[16*blk + h->zigzag_scan[4]]`, the matching `qmul`, and check
   `wmd_level()`'s output against the CSV.
3. **Watch the MB grid.** `ffmpeg -debug mb_type -threads 1 -i in.mp4 -f null -` prints a
   per-MB type map per frame — compare against your x264 intuition of what the encoder
   chose (I/P/B, 8×8 DCT flags).
4. **Watch reordering happen.**
   `ffprobe -show_frames -select_streams v in.mp4 | grep -E 'pict_type|pts'` vs. the
   decode-order slice headers from
   `ffmpeg -i in.mp4 -c:v copy -bsf:v trace_headers -f null - 2>&1 | grep -E 'nal_unit_type|pic_order_cnt'`.
   Seeing POC jump around in decode order makes §7 concrete.
5. **Break the mb_xy trap on purpose.** In a scratch branch, make the detector use
   `sl->mb_xy` instead of the recomputed index and watch BER degrade with frame height —
   nothing teaches the padding column faster.

### Debugging toolbox

```bash
# debug-friendly build (keeps symbols, no inlining surprises)
./configure --enable-debug=3 --disable-optimizations --disable-stripping ...

ffmpeg -threads 1 ...          # kill all threading nondeterminism first
ffmpeg -debug mb_type ...      # per-MB type map (also: -debug qp)
ffmpeg -i in -f framemd5 -     # bit-exactness check (your inertness proof)
ffmpeg -c copy -bsf:v trace_headers -f null -   # full syntax-level header dump
gdb --args ffmpeg_g -threads 1 -i in.mp4 -f null -   # ffmpeg_g = unstripped binary
  (gdb) break ff_h264_hl_decode_mb if sl->mb_x==10 && sl->mb_y==5
```

---

## 11. Ten-line summary

The decoder is a bitstream interpreter with no decisions. h264dec.c unwraps packets into
NALs; h264_slice.c parses slice headers, sets up each picture (`h264_field_start`), and
runs the MB loop; each MB is parsed (h264_cavlc/cabac.c → `sl->mb` + caches) then
reconstructed (h264_mb.c → pixels), and `sl->mb` dies at that boundary — where hook 2
lives. Coefficients in `sl->mb` are dequantized-and-transposed versions of the levels
x264 wrote; the transposed `h->zigzag_scan` tables and exact dequant inversion undo that.
`mb_stride` is `mb_width + 1`, so x264-compatible indices must be recomputed. Display
order is only knowable after DPB reordering, keyed by (IDR generation, POC). Frame
threading clones the whole `H264Context`, so cross-frame state must be global + locked,
and per-picture state must ride inside the context. That's the entire mental scaffold the
detector hangs on.
