/*
 * h264_wmdetect.c: forensic watermark detection on quantized coefficients
 *
 * Research prototype.  Recovers payload bits embedded by the x264-side
 * watermarker (encoder/watermark.c, x264 'indee' branch), which enforces a
 * magnitude relation between the coefficients at zigzag scan positions 4 and
 * 5 of coded luma 4x4/8x8 blocks:
 *     bit 1  ->  |A| > |B| + delta      (A = zigzag pos 4, B = pos 5)
 *     bit 0  ->  |B| > |A| + delta
 * with payload bit index (i_frame*40503 + mb_xy*16 + blk) mod payload_bits,
 * i_frame = display-order frame index, mb_xy = mb_y*mb_width + mb_x
 * (x264 stride convention, NOT ffmpeg's mb_width+1).
 *
 * sl->mb holds DEQUANTIZED coefficients in transposed-raster order.  The two
 * pair positions fall in different dequant position classes, so raw levels
 * are recovered by exact rounding division before comparison (exact whenever
 * qmul > 64, true for all flat/standard scaling matrices).
 *
 * Votes are accumulated per frame keyed by (mb_xy*16 + blk) mod bits; the
 * unknown display-order frame index is applied at finalize as a histogram
 * rotation, once frames have been sorted by (IDR generation, POC).
 *
 * Enabled only with X264_WMD_ENABLE=1; otherwise every hook returns
 * immediately and decoding is unaffected.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "libavutil/mem.h"
#include "libavutil/thread.h"

#include "h264dec.h"
#include "mpegutils.h"
#include "h264_wmdetect.h"

#define WMD_MAX_PAYLOAD_BYTES 64
#define WMD_MAX_BITS          (WMD_MAX_PAYLOAD_BYTES * 8)
#define WMD_DEFAULT_PAYLOAD   "DEADBEEF"
#define WMD_FRAME_MIX         40503u  /* embedder's frame multiplier */
#define WMD_DEBUG_MAX_LINES   64

typedef struct WMDFrame {
    uint64_t gen;                   /* IDR generation counter value */
    int      poc;
    uint32_t w1[WMD_MAX_BITS];      /* votes for bit=1, keyed by local index */
    uint32_t w0[WMD_MAX_BITS];
    uint64_t votes;
    struct WMDFrame *next;
} WMDFrame;

typedef struct {
    int inited;
    int enabled;
    int debug;
    int weight_mode;
    int min_nz;
    int bits;
    int have_ref;
    uint8_t ref_payload[WMD_MAX_PAYLOAD_BYTES];
    FILE *dump;
    uint64_t gen_counter;
    WMDFrame *frames;
    uint64_t n_frames, n_votes, n_skipped, n_field_skipped;
    uint64_t n_debug_lines;
} WMDState;

static WMDState wmd;
static AVMutex wmd_mutex = AV_MUTEX_INITIALIZER;

static int wmd_env_int(const char *name, int def)
{
    const char *v = getenv(name);
    return v ? atoi(v) : def;
}

static int wmd_parse_payload(const char *hex, uint8_t *out, int max_bytes)
{
    int n = 0;
    while (hex[0] && hex[1] && n < max_bytes) {
        char byte[3] = { hex[0], hex[1], 0 };
        char *end;
        long v = strtol(byte, &end, 16);
        if (*end)
            return 0;
        out[n++] = (uint8_t)v;
        hex += 2;
    }
    return (!hex[0] && n) ? n : 0;
}

/* Called under wmd_mutex. */
static void wmd_init(void)
{
    const char *payload, *dump_path;
    int n;

    memset(&wmd, 0, sizeof(wmd));
    wmd.inited      = 1;
    wmd.enabled     = wmd_env_int("X264_WMD_ENABLE", 0);
    if (!wmd.enabled)
        return;
    wmd.debug       = wmd_env_int("X264_WMD_DEBUG", 0);
    wmd.weight_mode = wmd_env_int("X264_WMD_WEIGHT", 0);
    wmd.min_nz      = wmd_env_int("X264_WMD_MIN_NZ", 2);

    payload = getenv("X264_WMD_PAYLOAD");
    if (!payload)
        payload = getenv("X264_WM_PAYLOAD");
    if (!payload)
        payload = WMD_DEFAULT_PAYLOAD;
    n = wmd_parse_payload(payload, wmd.ref_payload, WMD_MAX_PAYLOAD_BYTES);
    if (!n)
        n = wmd_parse_payload(WMD_DEFAULT_PAYLOAD, wmd.ref_payload,
                              WMD_MAX_PAYLOAD_BYTES);
    wmd.bits     = 8 * n;
    wmd.have_ref = 1;

    if (getenv("X264_WMD_BITS")) {
        int bits = wmd_env_int("X264_WMD_BITS", wmd.bits);
        bits = av_clip(bits, 1, WMD_MAX_BITS);
        if (bits != wmd.bits)
            wmd.have_ref = 0;   /* length mismatch: recover only, no BER */
        wmd.bits = bits;
    }

    dump_path = getenv("X264_WMD_DUMP");
    if (dump_path) {
        wmd.dump = fopen(dump_path, "w");
        if (wmd.dump)
            fprintf(wmd.dump, "gen,poc,mb_x,mb_y,blk,size,qp,levelA,levelB\n");
    }
}

void ff_h264_wmdetect_frame_start(H264Context *h)
{
    WMDFrame *f;

    h->wm_frame = NULL;

    ff_mutex_lock(&wmd_mutex);
    if (!wmd.inited)
        wmd_init();
    if (!wmd.enabled || !h->cur_pic_ptr)
        goto end;
    if (h->picture_structure != PICT_FRAME) {
        wmd.n_field_skipped++;      /* progressive only */
        goto end;
    }

    if (h->picture_idr)
        wmd.gen_counter++;

    /* Redundant slices / second field never reach here for PICT_FRAME, but a
     * repeated (gen,poc) would just reuse its bucket. */
    for (f = wmd.frames; f; f = f->next)
        if (f->gen == wmd.gen_counter && f->poc == h->cur_pic_ptr->poc)
            break;
    if (!f) {
        f = av_mallocz(sizeof(*f));
        if (!f)
            goto end;
        f->gen  = wmd.gen_counter;
        f->poc  = h->cur_pic_ptr->poc;
        f->next = wmd.frames;
        wmd.frames = f;
        wmd.n_frames++;
    }
    h->wm_frame = f;
end:
    ff_mutex_unlock(&wmd_mutex);
}

/* Invert dq = (level*qmul + 32) >> 6 by nearest-integer division; exact for
 * qmul > 64 (flat-matrix minimum is 640). */
static inline int wmd_level(int dq, uint32_t qmul)
{
    int64_t num = (int64_t)dq * 128 + (dq >= 0 ? (int64_t)qmul : -(int64_t)qmul);
    return (int)(num / (2 * (int64_t)qmul));
}

static void wmd_vote_block(const H264Context *h, const H264SliceContext *sl,
                           WMDFrame *frame, const int16_t *coef, int count,
                           const uint32_t *qmul, int pos_a, int pos_b,
                           int x264_mb_xy, int blk)
{
    int nnz = 0, la, lb, aa, ab, bit, weight;
    uint32_t local;

    for (int i = 0; i < count; i++)
        nnz += coef[i] != 0;    /* zero level <-> zero dequantized value */
    la = wmd_level(coef[pos_a], qmul[pos_a]);
    lb = wmd_level(coef[pos_b], qmul[pos_b]);
    aa = FFABS(la);
    ab = FFABS(lb);

    if (nnz < wmd.min_nz || aa == ab) {   /* covers both-zero */
        ff_mutex_lock(&wmd_mutex);
        wmd.n_skipped++;
        ff_mutex_unlock(&wmd_mutex);
        return;
    }

    bit    = aa > ab;
    weight = wmd.weight_mode ? FFMIN(FFABS(aa - ab), 8) : 1;
    local  = (uint32_t)((uint32_t)x264_mb_xy * 16u + (uint32_t)blk) % (uint32_t)wmd.bits;

    ff_mutex_lock(&wmd_mutex);
    if (bit)
        frame->w1[local] += weight;
    else
        frame->w0[local] += weight;
    frame->votes++;
    wmd.n_votes++;
    if (wmd.dump)
        fprintf(wmd.dump, "%"PRIu64",%d,%d,%d,%d,%d,%d,%d,%d\n",
                frame->gen, frame->poc, sl->mb_x, sl->mb_y, blk,
                count == 64 ? 8 : 4, sl->qscale, la, lb);
    if (wmd.debug && wmd.n_debug_lines < WMD_DEBUG_MAX_LINES) {
        wmd.n_debug_lines++;
        av_log(NULL, AV_LOG_INFO,
               "wmdetect: gen %"PRIu64" poc %d mb (%d,%d) blk %2d %s qp %2d "
               "A %d B %d -> bit %d\n",
               frame->gen, frame->poc, sl->mb_x, sl->mb_y, blk,
               count == 64 ? "8x8" : "4x4", sl->qscale, la, lb, bit);
    }
    ff_mutex_unlock(&wmd_mutex);
}

void ff_h264_wmdetect_mb(const H264Context *h, const H264SliceContext *sl)
{
    WMDFrame *frame = h->wm_frame;
    int mb_type, qp, list, x264_mb_xy;

    if (!frame || h->pixel_shift || sl->qscale == 0)
        return;

    mb_type = h->cur_pic.mb_type[sl->mb_xy];
    if (IS_INTRA_PCM(mb_type) || IS_INTERLACED(mb_type))
        return;

    qp         = sl->qscale;
    list       = IS_INTRA(mb_type) ? 0 : 3;
    x264_mb_xy = sl->mb_y * h->mb_width + sl->mb_x;

    if (IS_8x8DCT(mb_type) && !IS_INTRA16x16(mb_type)) {
        const uint32_t *qmul = h->ps.pps->dequant8_coeff[list][qp];
        int pos_a = h->zigzag_scan8x8[4];
        int pos_b = h->zigzag_scan8x8[5];
        for (int i8 = 0; i8 < 4; i8++) {
            const uint8_t *nnz = &sl->non_zero_count_cache[scan8[4 * i8]];
            if (nnz[0] | nnz[1] | nnz[8] | nnz[9])
                wmd_vote_block(h, sl, frame, &sl->mb[64 * i8], 64,
                               qmul, pos_a, pos_b, x264_mb_xy, i8);
        }
    } else {
        /* intra 4x4, I16x16 AC, inter 4x4: 16-coef blocks in z-order.
         * I16x16 AC uses dequant list 0 (== intra) and its DC slot is still
         * zero at this point, so no special casing is needed. */
        const uint32_t *qmul = h->ps.pps->dequant4_coeff[list][qp];
        int pos_a = h->zigzag_scan[4];
        int pos_b = h->zigzag_scan[5];
        for (int i = 0; i < 16; i++)
            if (sl->non_zero_count_cache[scan8[i]])
                wmd_vote_block(h, sl, frame, &sl->mb[16 * i], 16,
                               qmul, pos_a, pos_b, x264_mb_xy, i);
    }
}

static int wmd_frame_cmp(const void *a, const void *b)
{
    const WMDFrame *fa = *(const WMDFrame *const *)a;
    const WMDFrame *fb = *(const WMDFrame *const *)b;
    if (fa->gen != fb->gen)
        return fa->gen < fb->gen ? -1 : 1;
    return fa->poc - fb->poc;
}

void ff_h264_wmdetect_finalize(AVCodecContext *avctx)
{
    WMDFrame **order, *f;
    uint64_t g1[WMD_MAX_BITS] = {0}, g0[WMD_MAX_BITS] = {0};
    uint8_t recovered[WMD_MAX_PAYLOAD_BYTES] = {0};
    int bits, ber = 0, no_votes = 0;
    uint64_t rank;
    double conf_min = 1.0, conf_sum = 0.0;
    int conf_cnt = 0;

    ff_mutex_lock(&wmd_mutex);
    if (!wmd.inited || !wmd.enabled || !wmd.n_frames) {
        memset(&wmd, 0, sizeof(wmd));
        ff_mutex_unlock(&wmd_mutex);
        return;
    }
    bits = wmd.bits;

    order = av_malloc_array(wmd.n_frames, sizeof(*order));
    if (!order) {
        ff_mutex_unlock(&wmd_mutex);
        return;
    }
    rank = 0;
    for (f = wmd.frames; f; f = f->next)
        order[rank++] = f;
    qsort(order, wmd.n_frames, sizeof(*order), wmd_frame_cmp);

    /* display-order rank == x264 i_frame; apply the frame term of the
     * embedder's bit index as a rotation of each frame's local histogram */
    for (rank = 0; rank < wmd.n_frames; rank++) {
        uint32_t rot = (uint32_t)((uint32_t)rank * WMD_FRAME_MIX) % (uint32_t)bits;
        f = order[rank];
        for (int j = 0; j < bits; j++) {
            g1[(j + rot) % bits] += f->w1[j];
            g0[(j + rot) % bits] += f->w0[j];
        }
    }

    av_log(avctx, AV_LOG_INFO,
           "wmdetect: %"PRIu64" frames, %"PRIu64" votes, %"PRIu64" blocks skipped"
           "%s\n", wmd.n_frames, wmd.n_votes, wmd.n_skipped,
           wmd.n_field_skipped ? " (field pictures ignored)" : "");

    for (int i = 0; i < bits; i++) {
        uint64_t v1 = g1[i], v0 = g0[i];
        int bit = v1 > v0;
        if (bit)
            recovered[i >> 3] |= 1 << (7 - (i & 7));
        if (v1 + v0) {
            double conf = (double)FFMAX(v1, v0) / (double)(v1 + v0);
            conf_min  = FFMIN(conf_min, conf);
            conf_sum += conf;
            conf_cnt++;
        } else
            no_votes++;
        if (wmd.have_ref) {
            int ref = (wmd.ref_payload[i >> 3] >> (7 - (i & 7))) & 1;
            ber += bit != ref;
        }
        if (wmd.debug || bits <= 64)
            av_log(avctx, AV_LOG_INFO,
                   "wmdetect: bit %2d = %d  (%"PRIu64" vs %"PRIu64", conf %.3f)\n",
                   i, bit, bit ? v1 : v0, bit ? v0 : v1,
                   v1 + v0 ? (double)FFMAX(v1, v0) / (double)(v1 + v0) : 0.0);
    }

    {
        char hex[2 * WMD_MAX_PAYLOAD_BYTES + 1];
        int nbytes = (bits + 7) >> 3;
        for (int i = 0; i < nbytes; i++)
            snprintf(hex + 2 * i, 3, "%02X", recovered[i]);
        av_log(avctx, AV_LOG_INFO,
               "wmdetect: recovered payload: %s (%d bits, min conf %.3f, "
               "mean conf %.3f, %d bits without votes)\n",
               hex, bits, conf_cnt ? conf_min : 0.0,
               conf_cnt ? conf_sum / conf_cnt : 0.0, no_votes);
    }
    if (wmd.have_ref)
        av_log(avctx, AV_LOG_INFO,
               "wmdetect: BER %d/%d (%.2f%%) vs reference payload\n",
               ber, bits, 100.0 * ber / bits);

    for (rank = 0; rank < wmd.n_frames; rank++)
        av_free(order[rank]);
    av_free(order);
    if (wmd.dump)
        fclose(wmd.dump);
    memset(&wmd, 0, sizeof(wmd));   /* clean slate for a next decoder instance */
    ff_mutex_unlock(&wmd_mutex);
}
