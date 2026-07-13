/*
 * h264_wmdetect.h: quantized-coefficient watermark detection (research prototype)
 *
 * Decoder-side counterpart of the x264 watermark embedder (encoder/watermark.c
 * on the x264 'indee' branch).  Inert unless the environment variable
 * X264_WMD_ENABLE=1 is set; never modifies decoding.
 */

#ifndef AVCODEC_H264_WMDETECT_H
#define AVCODEC_H264_WMDETECT_H

struct AVCodecContext;
struct H264Context;
struct H264SliceContext;

/* Called from h264_field_start() once per picture, in decode order, after
 * POC init and picture_idr are set.  Creates/looks up the per-frame vote
 * bucket and stores it in h->wm_frame. */
void ff_h264_wmdetect_frame_start(struct H264Context *h);

/* Called from ff_h264_hl_decode_mb() once per macroblock, after residual
 * decode and before the IDCT consumes sl->mb.  Read-only. */
void ff_h264_wmdetect_mb(const struct H264Context *h,
                         const struct H264SliceContext *sl);

/* Called from h264_decode_end() on the user-facing context (not frame-thread
 * copies).  Sorts frames into display order, aggregates votes, prints the
 * recovered payload, and resets all state. */
void ff_h264_wmdetect_finalize(struct AVCodecContext *avctx);

#endif /* AVCODEC_H264_WMDETECT_H */
