/*
 * Copyright (c) 2020, Derek Buitenhuis
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <assert.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "obuparse.h"

/************************************
 * Bitreader functions and structs. *
 ************************************/

typedef struct _OBPBitReader {
    uint8_t *buf;
    size_t buf_size;
    size_t buf_pos;
    uint64_t bit_buffer;
    uint8_t bits_in_buf;
} _OBPBitReader;

static inline _OBPBitReader _obp_new_br(uint8_t *buf, size_t buf_size)
{
    _OBPBitReader ret = { buf, buf_size, 0, 0, 0 };
    return ret;
}

static inline uint64_t _obp_br_unchecked(_OBPBitReader *br, uint8_t n)
{
    assert(n <= 63);

    while (n > br->bits_in_buf) {
        br->bit_buffer <<= 8;
        br->bit_buffer  |= (uint64_t) br->buf[br->buf_pos];
        br->bits_in_buf += 8;
        br->buf_pos++;

        if (br->bits_in_buf > 56) {
            if (n <= br->bits_in_buf)
                break;

            if (n <= 64)
                return (_obp_br_unchecked(br, 32) << 32) | (_obp_br_unchecked(br, n - 32));
        }
    }

    br->bits_in_buf -= n;
    return (br->bit_buffer >> br->bits_in_buf) & ((((uint64_t)1) << n) - 1);
}

#if OBP_UNCHECKED_BITREADER
#define _obp_br(x, br, n) do { \
    x = _obp_br_unchecked(br, n); \
} while(0)
#else
#define _obp_br(x, br, n) do { \
    size_t bytes_needed = ((n - br->bits_in_buf) + (1<<3) - 1) >> 3; \
    if (bytes_needed > (br->buf_size - br->buf_pos)) { \
        snprintf(err->error, err->size, "Ran out of bytes in buffer."); \
        return -1; \
    } \
    x = _obp_br_unchecked(br, n); \
} while(0)
#endif

/*********************
 * Helper functions. *
 *********************/

static inline int _obp_is_valid_obu(OBPOBUType type)
{
    return type == OBP_OBU_SEQUENCE_HEADER ||
           type == OBP_OBU_TEMPORAL_DELIMITER ||
           type == OBP_OBU_FRAME_HEADER ||
           type == OBP_OBU_TILE_GROUP ||
           type == OBP_OBU_METADATA ||
           type == OBP_OBU_FRAME ||
           type == OBP_OBU_REDUNDANT_FRAME_HEADER ||
           type == OBP_OBU_TILE_LIST ||
           type == OBP_OBU_PADDING;
}

/************************************
 * Functions from AV1 spcification. *
 ************************************/

static inline int _obp_leb128(uint8_t *buf, size_t size, uint64_t *value, ptrdiff_t *consumed, OBPError *err)
{
    *value       = 0;
    *consumed    = 0;

    for (uint64_t i = 0; i < 8; i++) {
        uint8_t b;

        if (((size_t) (*consumed) + 1) > size) {
            snprintf(err->error, err->size, "Buffer too short to read leb128 value.");
            return -1;
        }

        b       = buf[*consumed];
        *value |= ((uint64_t)(b & 0x7F)) << (i * 7);
        (*consumed)++;

        if ((b & 0x80) != 0x80)
            break;
    }

    return 0;
}

static inline int _obp_uvlc(_OBPBitReader *br, uint32_t *value, OBPError *err)
{
    uint32_t leading_zeroes = 0;
    while (leading_zeroes < 32) {
        int b;
        _obp_br(b, br, 1);
        if (b != 0)
            break;
        leading_zeroes++;
    }
    if (leading_zeroes == 32) {
        snprintf(err->error, err->size, "Invalid VLC.");
        return -1;
    }
    uint32_t val;
    _obp_br(val, br, leading_zeroes);
    *value = val + ((1 << leading_zeroes) - 1);
    return 0;
}

static inline int32_t _obp_get_relative_dist(int32_t a, int32_t b, OBPSequenceHeader *seq)
{
    int32_t diff, m;

    if (!seq->enable_order_hint)
        return 0;

    diff = a - b;
    m    = 1 << (seq->OrderHintBits - 1);
    diff = (diff & (m - 1)) - (diff & m);

    return diff;
}

/*****************************
 * API functions start here. *
 *****************************/

int obp_get_next_obu(uint8_t *buf, size_t buf_size, OBPOBUType *obu_type, ptrdiff_t *offset,
                     size_t *size, int *temporal_id, int *spatial_id, OBPError *err)
{
    ptrdiff_t pos = 0;
    int obu_extension_flag;
    int obu_has_size_field;

    if (buf_size < 1) {
        snprintf(err->error, err->size, "Buffer is too small to contain an OBU.");
        return -1;
    }

    *obu_type          = (buf[pos] & 0x78) >> 3;
    obu_extension_flag = (buf[pos] & 0x04) >> 2;
    obu_has_size_field = (buf[pos] & 0x02) >> 1;
    pos++;

    if (!_obp_is_valid_obu(*obu_type)) {
        snprintf(err->error, err->size, "OBU header contains invalid OBU type: %d", *obu_type);
        return -1;
    }

    if (obu_extension_flag) {
        if (buf_size < 1) {
            snprintf(err->error, err->size, "Buffer is too small to contain an OBU extension header.");
            return -1;
        }
        *temporal_id = (buf[pos] & 0xE0) >> 5;
        *spatial_id  = (buf[pos] & 0x18) >> 3;
        pos++;
    } else {
        *temporal_id = 0;
        *spatial_id  = 0;
    }

    if (obu_has_size_field) {
        char err_buf[1024];
        uint64_t value;
        ptrdiff_t consumed;
        OBPError error = { &err_buf[0], 1024 };

        int ret      = _obp_leb128(buf + pos, buf_size - (size_t) pos, &value, &consumed, &error);
        if (ret < 0) {
            snprintf(err->error, err->size, "Failed to read OBU size: %s", &error.error[0]);
            return -1;
        }

        assert(value < UINT32_MAX);

        *offset = (ptrdiff_t) pos + consumed;
        *size   = (size_t) value;
    } else {
        *offset = (ptrdiff_t) pos;
        *size   = buf_size - (size_t) pos;
    }

    if (*size > buf_size - (size_t) offset) {
        snprintf(err->error, err->size, "Invalid OBU size: larger than remaining buffer.");
        return -1;
    }

    return 0;
}

int obp_parse_sequence_header(uint8_t *buf, size_t buf_size, OBPSequenceHeader *seq_header, OBPError *err)
{
    _OBPBitReader b   = _obp_new_br(buf, buf_size);
    _OBPBitReader *br = &b;

    _obp_br(seq_header->seq_profile, br, 3);
    _obp_br(seq_header->still_picture, br, 1);
    _obp_br(seq_header->reduced_still_picture_header, br, 1);
    if (seq_header->reduced_still_picture_header) {
        seq_header->timing_info_present_flag                     = 0;
        seq_header->decoder_model_info_present_flag              = 0;
        seq_header->initial_display_delay_present_flag           = 0;
        seq_header->operating_points_cnt_minus_1                 = 0;
        seq_header->operating_point_idc[0]                       = 0;
        seq_header->seq_level_idx[0]                             = 0;
        seq_header->seq_tier[0]                                  = 0;
        seq_header->decoder_model_present_for_this_op[0]         = 0;
        seq_header->initial_display_delay_present_for_this_op[0] = 0;
    } else {
        _obp_br(seq_header->timing_info_present_flag, br, 1);
        if (seq_header->timing_info_present_flag) {
            /* timing_info() */
            _obp_br(seq_header->timing_info.num_units_in_display_tick, br, 32);
            _obp_br(seq_header->timing_info.time_scale, br, 32);
            _obp_br(seq_header->timing_info.equal_picture_interval, br, 1);
            if (seq_header->timing_info.equal_picture_interval) {
                int ret = _obp_uvlc(br, &seq_header->timing_info.num_ticks_per_picture_minus_1, err);
                if (ret < 0)
                    return -1;
            }
            _obp_br(seq_header->decoder_model_info_present_flag, br, 1);
            if (seq_header->decoder_model_info_present_flag) {
                /* decoder_model_info() */
                _obp_br(seq_header->decoder_model_info.buffer_delay_length_minus_1, br, 5);
                _obp_br(seq_header->decoder_model_info.num_units_in_decoding_tick, br, 32);
                _obp_br(seq_header->decoder_model_info.buffer_removal_time_length_minus_1, br, 5);
                _obp_br(seq_header->decoder_model_info.frame_presentation_time_length_minus_1, br, 5);
            }
        } else {
            seq_header->decoder_model_info_present_flag = 0;
        }
        _obp_br(seq_header->initial_display_delay_present_flag, br, 1);
        _obp_br(seq_header->operating_points_cnt_minus_1, br, 5);
        for (uint8_t i = 0; i <= seq_header->operating_points_cnt_minus_1; i++) {
            _obp_br(seq_header->operating_point_idc[i], br, 12);
            _obp_br(seq_header->seq_level_idx[i], br, 5);
            if (seq_header->seq_level_idx[i] > 7) {
                _obp_br(seq_header->seq_tier[i], br, 1);
            } else {
                seq_header->seq_tier[i] = 0;
            }
            if (seq_header->decoder_model_info_present_flag) {
                _obp_br(seq_header->decoder_model_present_for_this_op[i], br, 1);
                if (seq_header->decoder_model_present_for_this_op[i]) {
                    /* operating_parameters_info() */
                    uint8_t n = seq_header->decoder_model_info.buffer_delay_length_minus_1 + 1;
                    _obp_br(seq_header->operating_parameters_info[i].decoder_buffer_delay, br, n);
                    _obp_br(seq_header->operating_parameters_info[i].encoder_buffer_delay, br, n);
                    _obp_br(seq_header->operating_parameters_info[i].low_delay_mode_flag, br, 1);
                }
            } else {
                seq_header->decoder_model_present_for_this_op[i] = 0;
            }
            if (seq_header->initial_display_delay_present_flag) {
                _obp_br(seq_header->initial_display_delay_present_for_this_op[i], br, 1);
                if (seq_header->initial_display_delay_present_for_this_op[i]) {
                    _obp_br(seq_header->initial_display_delay_minus_1[i], br, 4);
                }
            }
        }
    }
    _obp_br(seq_header->frame_width_bits_minus_1, br, 4);
    _obp_br(seq_header->frame_height_bits_minus_1, br, 4);
    _obp_br(seq_header->max_frame_width_minus_1, br, seq_header->frame_width_bits_minus_1 + 1);
    _obp_br(seq_header->max_frame_height_minus_1, br, seq_header->frame_height_bits_minus_1 + 1);
    if (seq_header->reduced_still_picture_header) {
        seq_header->frame_id_numbers_present_flag = 0;
    } else {
        _obp_br(seq_header->frame_id_numbers_present_flag, br, 1);
    }
    if (seq_header->frame_id_numbers_present_flag) {
        _obp_br(seq_header->delta_frame_id_length_minus_2, br, 4);
        _obp_br(seq_header->additional_frame_id_length_minus_1, br, 3);
    }
    _obp_br(seq_header->use_128x128_superblock, br, 1);
    _obp_br(seq_header->enable_filter_intra, br, 1);
    _obp_br(seq_header->enable_intra_edge_filter, br, 1);
    if (seq_header->reduced_still_picture_header) {
        seq_header->enable_interintra_compound     = 0;
        seq_header->enable_masked_compound         = 0;
        seq_header->enable_warped_motion           = 0;
        seq_header->enable_dual_filter             = 0;
        seq_header->enable_order_hint              = 0;
        seq_header->enable_jnt_comp                = 0;
        seq_header->enable_ref_frame_mvs           = 0;
        seq_header->seq_force_screen_content_tools = 2; /* SELECT_SCREEN_CONTENT_TOOLS */
        seq_header->seq_force_integer_mv           = 2; /* SELECT_INTEGER_MV */
        seq_header->OrderHintBits                  = 0;
    } else {
        _obp_br(seq_header->enable_interintra_compound, br, 1);
        _obp_br(seq_header->enable_masked_compound, br, 1);
        _obp_br(seq_header->enable_warped_motion, br, 1);
        _obp_br(seq_header->enable_dual_filter, br, 1);
        _obp_br(seq_header->enable_order_hint, br, 1);
        if (seq_header->enable_order_hint) {
            _obp_br(seq_header->enable_jnt_comp, br, 1);
            _obp_br(seq_header->enable_ref_frame_mvs, br, 1);
        } else {
            seq_header->enable_jnt_comp = 0;
            seq_header->enable_ref_frame_mvs = 0;
        }
        _obp_br(seq_header->seq_choose_screen_content_tools, br, 1);
        if (seq_header->seq_choose_screen_content_tools) {
            seq_header->seq_force_screen_content_tools = 2; /* SELECT_SCREEN_CONTENT_TOOLS */
        } else {
            _obp_br(seq_header->seq_force_screen_content_tools, br, 1);
        }
        if (seq_header->seq_force_screen_content_tools > 0) {
            _obp_br(seq_header->seq_choose_integer_mv, br, 1);
            if (seq_header->seq_choose_integer_mv) {
                seq_header->seq_force_integer_mv = 2; /* SELECT_INTEGER_MV */
            } else {
                _obp_br(seq_header->seq_force_integer_mv, br, 1);
            }
        } else {
            seq_header->seq_force_integer_mv = 2; /* SELECT_INTEGER_MV */
        }
        if (seq_header->enable_order_hint) {
            _obp_br(seq_header->order_hint_bits_minus_1, br, 3);
            seq_header->OrderHintBits = seq_header->order_hint_bits_minus_1 + 1;
        } else {
            seq_header->OrderHintBits = 0;
        }
    }
    _obp_br(seq_header->enable_superres, br, 1);
    _obp_br(seq_header->enable_cdef, br, 1);
    _obp_br(seq_header->enable_restoration, br, 1);
    /* color_config() */
    _obp_br(seq_header->color_config.high_bitdepth, br, 1);
    if (seq_header->seq_profile == 2 && seq_header->color_config.high_bitdepth) {
        _obp_br(seq_header->color_config.twelve_bit, br, 1);
        seq_header->color_config.BitDepth = seq_header->color_config.twelve_bit ? 12 : 10;
    } else {
        seq_header->color_config.BitDepth = seq_header->color_config.high_bitdepth ? 10 : 8;
    }
    if (seq_header->seq_profile == 1) {
        seq_header->color_config.mono_chrome = 0;
    } else {
        _obp_br(seq_header->color_config.mono_chrome, br, 1);
    }
    seq_header->color_config.NumPlanes = seq_header->color_config.mono_chrome ? 1 : 3;
    _obp_br(seq_header->color_config.color_description_present_flag, br, 1);
    if (seq_header->color_config.color_description_present_flag) {
        _obp_br(seq_header->color_config.color_primaries, br, 8);
        _obp_br(seq_header->color_config.transfer_characteristics, br, 8);
        _obp_br(seq_header->color_config.matrix_coefficients, br, 8);
    } else {
        seq_header->color_config.color_primaries          = OBP_CP_UNSPECIFIED;
        seq_header->color_config.transfer_characteristics = OBP_TC_UNSPECIFIED;
        seq_header->color_config.matrix_coefficients      = OBP_MC_UNSPECIFIED;
    }
    if (seq_header->color_config.mono_chrome) {
        _obp_br(seq_header->color_config.color_range, br, 1);
        seq_header->color_config.subsampling_x          = 1;
        seq_header->color_config.subsampling_y          = 1;
        seq_header->color_config.chroma_sample_position = OBP_CSP_UNKNOWN;
        seq_header->color_config.separate_uv_delta_q    = 0;
        goto color_done;
    } else if (seq_header->color_config.color_primaries == OBP_CP_BT_709 &&
               seq_header->color_config.transfer_characteristics == OBP_TC_SRGB &&
               seq_header->color_config.matrix_coefficients == OBP_MC_IDENTITY) {
        seq_header->color_config.color_range = 1;
        seq_header->color_config.subsampling_x = 0;
        seq_header->color_config.subsampling_y = 0;
    } else {
        _obp_br(seq_header->color_config.color_range, br, 1);
        if (seq_header->seq_profile == 0) {
            seq_header->color_config.subsampling_x = 1;
            seq_header->color_config.subsampling_y = 1;
        } else if (seq_header->seq_profile == 1) {
            seq_header->color_config.subsampling_x = 0;
            seq_header->color_config.subsampling_y = 0;
        } else {
            if (seq_header->color_config.BitDepth == 12) {
                _obp_br(seq_header->color_config.subsampling_x, br, 1);
                if (seq_header->color_config.subsampling_x) {
                    _obp_br(seq_header->color_config.subsampling_y, br, 1);
                } else {
                    seq_header->color_config.subsampling_y = 0;
                }
            } else {
                seq_header->color_config.subsampling_x = 1;
                seq_header->color_config.subsampling_y = 0;
            }
        }
        if (seq_header->color_config.subsampling_x && seq_header->color_config.subsampling_y) {
            _obp_br(seq_header->color_config.chroma_sample_position, br, 2);
        }
    }
    _obp_br(seq_header->color_config.separate_uv_delta_q, br, 1);

color_done:
    _obp_br(seq_header->film_grain_params_present, br, 1);

    return 0;
}

int obp_parse_tile_list(uint8_t *buf, size_t buf_size, OBPTileList *tile_list, OBPError *err)
{
    size_t pos = 0;

    if (buf_size < 4) {
        snprintf(err->error, err->size, "Tile list OBU must be at least 4 bytes.");
        return -1;
    }

    tile_list->output_frame_width_in_tiles_minus_1  = buf[0];
    tile_list->output_frame_height_in_tiles_minus_1 = buf[1];
    tile_list->tile_count_minus_1                   = (((uint16_t) buf[2]) << 8) | buf[3];
    pos += 4;

    for (uint16_t i = 0; i < tile_list->tile_count_minus_1; i++) {
        if (pos + 5 > buf_size) {
            snprintf(err->error, err->size, "Tile list OBU malformed: Not enough bytes for next tile_list_entry().");
            return -1;
        }

        tile_list->tile_list_entry[i].anchor_frame_idx       = buf[pos];
        tile_list->tile_list_entry[i].anchor_tile_row        = buf[pos + 1];
        tile_list->tile_list_entry[i].anchor_tile_col        = buf[pos + 2];
        tile_list->tile_list_entry[i].tile_data_size_minus_1 = (((uint16_t) buf[pos + 3]) << 8) | buf[pos + 4];
        pos += 5;

        size_t N = 8 * (((size_t) tile_list->tile_list_entry[i].tile_data_size_minus_1) + 1);

        if (pos + N > buf_size) {
            snprintf(err->error, err->size, "Tile list OBU malformed: Not enough bytes for next tile_list_entry()'s data.");
            return -1;
        }

        tile_list->tile_list_entry[i].coded_tile_data      = buf + pos;
        tile_list->tile_list_entry[i].coded_tile_data_size = N;
        pos += N;
    }

    return 0;
}

int obp_parse_metadata(uint8_t *buf, size_t buf_size, OBPMetadata *metadata, OBPError *err)
{
    uint64_t val;
    ptrdiff_t consumed;
    char err_buf[1024];
    _OBPBitReader b;
    _OBPBitReader *br;
    OBPError error = { &err_buf[0], 1024 };

    int ret = _obp_leb128(buf, buf_size, &val, &consumed, &error);
    if (ret < 0) {
        snprintf(err->error, err->size, "Couldn't read metadata type: %s", error.error);
        return -1;
    }
    metadata->metadata_type = val;

    b  = _obp_new_br(buf + consumed, buf_size - consumed);
    br = &b;

    if (metadata->metadata_type == OBP_METADATA_TYPE_HDR_CLL) {
        _obp_br(metadata->metadata_hdr_cll.max_cll, br, 16);
        _obp_br(metadata->metadata_hdr_cll.max_fall, br, 16);
    } else if (metadata->metadata_type == OBP_METADATA_TYPE_HDR_MDCV) {
        for (int i = 0; i < 3; i++) {
            _obp_br(metadata->metadata_hdr_mdcv.primary_chromaticity_x[i], br, 16);
            _obp_br(metadata->metadata_hdr_mdcv.primary_chromaticity_y[i], br, 16);
        }
        _obp_br(metadata->metadata_hdr_mdcv.white_point_chromaticity_x, br, 16);
        _obp_br(metadata->metadata_hdr_mdcv.white_point_chromaticity_y, br, 16);
        _obp_br(metadata->metadata_hdr_mdcv.luminance_max, br, 32);
        _obp_br(metadata->metadata_hdr_mdcv.luminance_min, br, 32);
    } else if (metadata->metadata_type == OBP_METADATA_TYPE_SCALABILITY) {
        _obp_br(metadata->metadata_scalability.scalability_mode_idc, br, 8);
        if (metadata->metadata_scalability.scalability_mode_idc) {
            /* scalability_structure() */
            _obp_br(metadata->metadata_scalability.scalability_structure.spatial_layers_cnt_minus_1, br, 2);
            _obp_br(metadata->metadata_scalability.scalability_structure.spatial_layer_dimensions_present_flag, br, 1);
            _obp_br(metadata->metadata_scalability.scalability_structure.spatial_layer_description_present_flag, br, 1);
            _obp_br(metadata->metadata_scalability.scalability_structure.temporal_group_description_present_flag, br, 1);
            _obp_br(metadata->metadata_scalability.scalability_structure.scalability_structure_reserved_3bits, br, 3);
            if (metadata->metadata_scalability.scalability_structure.spatial_layer_dimensions_present_flag) {
                for (uint8_t i = 0; i < metadata->metadata_scalability.scalability_structure.spatial_layers_cnt_minus_1; i++) {
                    _obp_br(metadata->metadata_scalability.scalability_structure.spatial_layer_max_width[i], br, 16);
                    _obp_br(metadata->metadata_scalability.scalability_structure.spatial_layer_max_height[i], br, 16);
                }
            }
            if (metadata->metadata_scalability.scalability_structure.spatial_layer_description_present_flag) {
                for (uint8_t i = 0; i < metadata->metadata_scalability.scalability_structure.spatial_layers_cnt_minus_1; i++) {
                    _obp_br(metadata->metadata_scalability.scalability_structure.spatial_layer_ref_id[i], br, 8);
                }
            }
            if (metadata->metadata_scalability.scalability_structure.temporal_group_description_present_flag) {
                _obp_br(metadata->metadata_scalability.scalability_structure.temporal_group_size, br, 8);
                for (uint8_t i = 0; i < metadata->metadata_scalability.scalability_structure.temporal_group_size; i++) {
                    _obp_br(metadata->metadata_scalability.scalability_structure.temporal_group_temporal_id[i], br, 3);
                    _obp_br(metadata->metadata_scalability.scalability_structure.temporal_group_temporal_switching_up_point_flag[i], br, 1);
                    _obp_br(metadata->metadata_scalability.scalability_structure.temporal_group_spatial_switching_up_point_flag[i], br, 1);
                    _obp_br(metadata->metadata_scalability.scalability_structure.temporal_group_ref_cnt[i], br, 3);
                    for (uint8_t j = 0; j < metadata->metadata_scalability.scalability_structure.temporal_group_ref_cnt[i]; j++) {
                        _obp_br(metadata->metadata_scalability.scalability_structure.temporal_group_ref_pic_diff[i][j], br, 8);
                    }
                }
            }
        }
    } else if (metadata->metadata_type == OBP_METADATA_TYPE_ITUT_T35) {
        size_t offset = 1;
        _obp_br(metadata->metadata_itut_t35.itu_t_t35_country_code, br, 8);
        if (metadata->metadata_itut_t35.itu_t_t35_country_code == 0xFF) {
            _obp_br(metadata->metadata_itut_t35.itu_t_t35_country_code_extension_byte, br, 8);
            offset++;
        }
        metadata->metadata_itut_t35.itu_t_t35_payload_bytes       = buf + consumed + offset;
        metadata->metadata_itut_t35.itu_t_t35_payload_bytes_size = buf_size - consumed - offset;
    } else if (metadata->metadata_type == OBP_METADATA_TYPE_TIMECODE) {
        _obp_br(metadata->metadata_timecode.counting_type, br, 5);
        _obp_br(metadata->metadata_timecode.full_timestamp_flag, br, 1);
        _obp_br(metadata->metadata_timecode.discontinuity_flag, br, 1);
        _obp_br(metadata->metadata_timecode.cnt_dropped_flag, br, 1);
        _obp_br(metadata->metadata_timecode.n_frames, br, 9);
        if (metadata->metadata_timecode.full_timestamp_flag) {
            _obp_br(metadata->metadata_timecode.seconds_value, br, 6);
            _obp_br(metadata->metadata_timecode.minutes_value, br, 6);
            _obp_br(metadata->metadata_timecode.hours_value, br, 5);
        } else {
            _obp_br(metadata->metadata_timecode.seconds_flag, br, 1);
            if (metadata->metadata_timecode.seconds_flag) {
                _obp_br(metadata->metadata_timecode.seconds_value, br, 6);
                _obp_br(metadata->metadata_timecode.minutes_flag, br, 1);
                if (metadata->metadata_timecode.minutes_flag) {
                    _obp_br(metadata->metadata_timecode.minutes_value, br, 6);
                    _obp_br(metadata->metadata_timecode.hours_flag, br, 1);
                    if (metadata->metadata_timecode.hours_flag) {
                        _obp_br(metadata->metadata_timecode.hours_value, br, 5);
                    }
                }
            }
        }
        _obp_br(metadata->metadata_timecode.time_offset_length, br, 5);
        if (metadata->metadata_timecode.time_offset_length > 0) {
             _obp_br(metadata->metadata_timecode.time_offset_value, br, metadata->metadata_timecode.time_offset_length);
        }
    } else if (metadata->metadata_type >= 6 && metadata->metadata_type <= 31) {
        metadata->unregistered.buf      = buf + consumed;
        metadata->unregistered.buf_size = buf_size - consumed;
    } else {
        snprintf(err->error, err->size, "Invalid metadata type: %"PRIu32"\n", metadata->metadata_type);
        return -1;
    }

    return 0;
}

int obp_parse_frame(uint8_t *buf, size_t buf_size, OBPSequenceHeader *seq, OBPState *state,
                    int temporal_id, int spatial_id, OBPFrameHeader *fh, void *tile_group,
                    int *SeenFrameHeader, OBPError *err)
{
    // TODO: Define OBPTileGroup and pals
    // pos = 0;
    return obp_parse_frame_header(buf, buf_size, seq, state, temporal_id, spatial_id, fh, SeenFrameHeader, err);
    // pos = consumed
    // parse_tile_group(buf_pos,size-pos)
}

int obp_parse_frame_header(uint8_t *buf, size_t buf_size, OBPSequenceHeader *seq, OBPState *state,
                           int temporal_id, int spatial_id, OBPFrameHeader *fh, int *SeenFrameHeader, OBPError *err)
{
    _OBPBitReader b   = _obp_new_br(buf, buf_size);
    _OBPBitReader *br = &b;

    if (*SeenFrameHeader == 1) {
        if (!state->prev_filled) {
            snprintf(err->error, err->size, "SeenFrameHeader is one, but no previous header exists in state.");
            return -1;
        }
        *fh = state->prev;
        return 0;
    }

    *SeenFrameHeader = 1;

    /* uncompressed_heade() */
    int idLen = 0; /* only set to 0 to shut up a compiler warning. */
    if (seq->frame_id_numbers_present_flag) {
        idLen = seq->additional_frame_id_length_minus_1 + seq->delta_frame_id_length_minus_2 + 3;
    }
    uint8_t allFrames = 255; /* (1 << 8) - 1 */
    int FrameIsIntra;
    if (seq->reduced_still_picture_header) {
        fh->show_existing_frame = 0;
        fh->frame_type          = OBP_KEY_FRAME;
        FrameIsIntra            = 1;
        fh->show_frame          = 1;
        fh->showable_frame      = 1;
    } else {
        _obp_br(fh->show_existing_frame, br, 1);
        if (fh->show_existing_frame) {
            _obp_br(fh->frame_to_show_map_idx, br, 3);
            if (seq->decoder_model_info_present_flag && !seq->timing_info.equal_picture_interval) {
                /* temporal_point_info() */
                uint8_t n = seq->decoder_model_info.frame_presentation_time_length_minus_1 + 1;
                _obp_br(fh->temporal_point_info.frame_presentation_time, br, n);
            }
            fh->refresh_frame_flags = 0;
            if (seq->frame_id_numbers_present_flag) {
                assert(idLen <= 255);
                _obp_br(fh->display_frame_id, br, (uint8_t) idLen);
            }
            fh->frame_type = state->RefFrameType[fh->frame_to_show_map_idx];
            if (fh->frame_type == OBP_KEY_FRAME) {
                fh->refresh_frame_flags = allFrames;
            }
            if (seq->film_grain_params_present) {
                /* TODO: load_grain_params(frame_to_show_map_idx) */
                assert(0);
            }
            return 0;
        }
        _obp_br(fh->frame_type, br, 2);
        FrameIsIntra = (fh->frame_type == OBP_INTRA_ONLY_FRAME || fh->frame_type == OBP_KEY_FRAME);
        _obp_br(fh->show_frame, br, 1);
        if (fh->show_frame && seq->decoder_model_info_present_flag && !seq->timing_info.equal_picture_interval){
            /* temporal_point_info() */
            uint8_t n = seq->decoder_model_info.frame_presentation_time_length_minus_1 + 1;
            _obp_br(fh->temporal_point_info.frame_presentation_time, br, n);
        }
        if (fh->show_frame) {
            fh->showable_frame = (fh->frame_type != OBP_KEY_FRAME);
        } else {
            _obp_br(fh->showable_frame, br, 1);
        }
        if (fh->frame_type == OBP_SWITCH_FRAME || (fh->frame_type == OBP_KEY_FRAME && fh->show_frame)) {
            fh->error_resilient_mode = 1;
        } else {
            _obp_br(fh->error_resilient_mode, br, 1);
        }
    }
    if (fh->frame_type == OBP_KEY_FRAME && fh->show_frame) {
        for (int i = 0; i < 8; i++) {
            state->RefValid[i]     = 0;
            state->RefOrderHint[i] = 0;
        }
        for (int i = 0; i < 7; i++) {
            state->OrderHint[1 + i] = 0;
        }
    }
    _obp_br(fh->disable_cdf_update, br, 1);
    if (seq->seq_force_screen_content_tools == 2) {
        _obp_br(fh->allow_screen_content_tools, br, 1);
    } else {
        fh->allow_screen_content_tools = seq->seq_force_screen_content_tools;
    }
    if (fh->allow_screen_content_tools) {
        if (seq->seq_force_integer_mv == 2) {
            _obp_br(fh->force_integer_mv, br, 1);
        } else {
            fh->force_integer_mv = seq->seq_force_integer_mv;
        }
    } else {
        fh->force_integer_mv = 0;
    }
    if (FrameIsIntra) {
         fh->force_integer_mv = 1;
    }
    if (seq->frame_id_numbers_present_flag) {
        /*PrevFrameID = current_frame_id */
        assert(idLen <= 255);
        _obp_br(fh->current_frame_id, br, idLen);
        /* mark_ref_frames(idLen) */
        uint8_t diffLen = seq->delta_frame_id_length_minus_2 + 2;
        for (int i = 0; i < 8; i++) {
            if (fh->current_frame_id > (((uint32_t)1) << diffLen)) {
                if (state->RefFrameId[i] > fh->current_frame_id || state->RefFrameId[i] < (fh->current_frame_id - (1 << diffLen))) {
                    state->RefValid[i] = 0;
                }
            } else {
                if (state->RefFrameId[i] > fh->current_frame_id && state->RefFrameId[i] < ((1 << idLen) + fh->current_frame_id + (1 << diffLen))) {
                    state->RefValid[i] = 0;
                }
            }
        }
    } else {
        fh->current_frame_id = 0;
    }
    if (fh->frame_type == OBP_SWITCH_FRAME) {
        fh->frame_size_override_flag = 1;
    } else if (seq->reduced_still_picture_header) {
        fh->frame_size_override_flag = 1;
    } else {
        _obp_br(fh->frame_size_override_flag, br, 1);
    }
    if (seq->OrderHintBits) { /* Added by me. */
        _obp_br(fh->order_hint, br, seq->OrderHintBits);
    } else {
        fh->order_hint = 0;
    }
    uint8_t OrderHint = fh->order_hint;
    if (FrameIsIntra || fh->error_resilient_mode) {
        fh->primary_ref_frame = 7;
    } else {
        _obp_br(fh->primary_ref_frame, br, 3);
    }
    if (seq->decoder_model_info_present_flag) {
        _obp_br(fh->buffer_removal_time_present_flag, br, 1);
        if (fh->buffer_removal_time_present_flag) {
            for (uint8_t opNum = 0; opNum <= seq->operating_points_cnt_minus_1; opNum++) {
                if (seq->decoder_model_present_for_this_op[opNum]) {
                    uint8_t opPtIdc = seq->operating_point_idc[opNum];
                    int inTemporalLayer = (opPtIdc >> temporal_id) & 1;
                    int inSpatialLayer = (opPtIdc >> (spatial_id + 8)) & 1;
                    if (opPtIdc == 0 || (inTemporalLayer && inSpatialLayer)) {
                        uint8_t n = seq->decoder_model_info.buffer_removal_time_length_minus_1 + 1;
                        _obp_br(fh->buffer_removal_time[opNum], br, n);
                    }
                }
            }
        }
    }
    fh->allow_high_precision_mv = 0;
    fh->use_ref_frame_mvs = 0;
    fh->allow_intrabc = 0;
    if (fh->frame_type == OBP_SWITCH_FRAME || (fh->frame_type == OBP_KEY_FRAME && fh->show_frame)) {
        fh->refresh_frame_flags = allFrames;
    } else {
        _obp_br(fh->refresh_frame_flags, br, 8);
    }
    if (!FrameIsIntra || fh->refresh_frame_flags != allFrames) {
        if (fh->error_resilient_mode && seq->enable_order_hint) {
            for (int i = 0; i < 8; i++) {
                _obp_br(fh->ref_order_hint[i], br, seq->OrderHintBits);
                if (fh->ref_order_hint[i] != state->RefOrderHint[i]) {
                    state->RefValid[i] = 0;
                }
            }
        }
    }
    uint32_t FrameWidth, FrameHeight;
    uint32_t UpscaledWidth;
    uint32_t MiCols, MiRows;
    if (FrameIsIntra) {
        /* frame_size() */
        if (fh->frame_size_override_flag) {
            uint8_t n = seq->frame_width_bits_minus_1 + 1;
            _obp_br(fh->frame_width_minus_1, br, n);
            n = seq->frame_height_bits_minus_1 + 1;
            _obp_br(fh->frame_height_minus_1, br, n);
            FrameWidth  = fh->frame_width_minus_1 + 1;
            FrameHeight = fh->frame_height_minus_1 + 1;
        } else {
            FrameWidth  = seq->max_frame_width_minus_1 + 1;
            FrameHeight = seq->max_frame_height_minus_1 + 1;
        }
        /* superres_params() */
        uint32_t SuperresDenom;
        if (seq->enable_superres) {
            _obp_br(fh->superres_params.use_superres, br, 1);
        } else {
            fh->superres_params.use_superres = 0;
        }
        if (fh->superres_params.use_superres) {
            _obp_br(fh->superres_params.coded_denom, br, 3);
            SuperresDenom = fh->superres_params.coded_denom + 9;
        } else {
            SuperresDenom = 8;
        }
        UpscaledWidth = FrameWidth;
        FrameWidth = (UpscaledWidth * 8 + (SuperresDenom / 2)) / SuperresDenom;
        /* compute_image_size() */
        MiCols = 2 * ((FrameWidth + 7) >> 3);
        MiRows = 2 * ((FrameHeight + 7) >> 3);
        /* render_size() */
        _obp_br(fh->render_and_frame_size_different, br, 1);
        if (fh->render_and_frame_size_different == 1) {
            _obp_br(fh->render_width_minus_1, br, 16);
            _obp_br(fh->render_height_minus_1, br, 16);
            fh->RenderWidth  = fh->render_width_minus_1 + 1;
            fh->RenderHeight = fh->render_height_minus_1 + 1;
        } else {
            fh->RenderWidth  = UpscaledWidth;
            fh->RenderHeight = FrameHeight;
        }
        if (fh->allow_screen_content_tools && UpscaledWidth == FrameWidth) {
            _obp_br(fh->allow_intrabc, br, 1);
        }
    } else {
        if (!seq->enable_order_hint) {
            fh->frame_refs_short_signaling = 0;
        } else {
            _obp_br(fh->frame_refs_short_signaling, br, 1);
            if (fh->frame_refs_short_signaling) {
                _obp_br(fh->last_frame_idx, br, 3);
                _obp_br(fh->gold_frame_idx, br, 3);
            }
            /* TODO: set_frame_refs() */
            assert(0);
        }
        for (int i = 0; i < 7; i++) {
            if (!fh->frame_refs_short_signaling) {
                _obp_br(fh->ref_frame_idx[i], br, 3);
            }
            if (seq->frame_id_numbers_present_flag) {
                uint8_t n = seq->delta_frame_id_length_minus_2 + 2;
                _obp_br(fh->delta_frame_id_minus_1[i], br, n);
                uint8_t DeltaFrameId    = fh->delta_frame_id_minus_1[i] + 1;
                uint8_t expectedFrameId = ((fh->current_frame_id + (1 << idLen) - DeltaFrameId) % (1 << idLen));
                if (state->RefFrameId[fh->ref_frame_idx[i]] != expectedFrameId) {
                    snprintf(err->error, err->size, "state->RefFrameId[fh->ref_frame_idx[i]] != expectedFrameId (%"PRIu8" vs %"PRIu8")",
                             state->RefFrameId[fh->ref_frame_idx[i]], expectedFrameId);
                    return -1;
                }
            }
        }

        if (!fh->frame_size_override_flag && !fh->error_resilient_mode) {
            for (int i = 0; i < 7; i++) {
                _obp_br(fh->found_ref, br, 1);
                if (fh->found_ref == 1) {
                    /* TODO: av1_decode_wrapup has https://aomediacodec.github.io/av1-spec/#set-frame-refs-process */
                    UpscaledWidth    = state->RefUpscaledWidth[fh->ref_frame_idx[i]];
                    FrameWidth       = UpscaledWidth;
                    FrameHeight      = state->RefFrameHeight[fh->ref_frame_idx[i]];
                    fh->RenderWidth  = state->RefRenderWidth[fh->ref_frame_idx[i]];
                    fh->RenderHeight = state->RefRenderHeight[fh->ref_frame_idx[i]];
                    break;
                }
            }
            if (fh->found_ref) {
                /* frame_size() */
                if (fh->frame_size_override_flag) {
                    uint8_t n = seq->frame_width_bits_minus_1 + 1;
                    _obp_br(fh->frame_width_minus_1, br, n);
                    n = seq->frame_height_bits_minus_1 + 1;
                    _obp_br(fh->frame_height_minus_1, br, n);
                    FrameWidth  = fh->frame_width_minus_1 + 1;
                    FrameHeight = fh->frame_height_minus_1 + 1;
                } else {
                    FrameWidth  = seq->max_frame_width_minus_1 + 1;
                    FrameHeight = seq->max_frame_height_minus_1 + 1;
                }
                /* superres_params() */
                uint32_t SuperresDenom;
                if (seq->enable_superres) {
                    _obp_br(fh->superres_params.use_superres, br, 1);
                } else {
                    fh->superres_params.use_superres = 0;
                }
                if (fh->superres_params.use_superres) {
                    _obp_br(fh->superres_params.coded_denom, br, 3);
                    SuperresDenom = fh->superres_params.coded_denom + 9;
                } else {
                    SuperresDenom = 8;
                }
                UpscaledWidth = FrameWidth;
                FrameWidth = (UpscaledWidth * 8 + (SuperresDenom / 2)) / SuperresDenom;
                /* compute_image_size() */
                MiCols = 2 * ((FrameWidth + 7) >> 3);
                MiRows = 2 * ((FrameHeight + 7) >> 3);
                /* render_size() */
                _obp_br(fh->render_and_frame_size_different, br, 1);
                if (fh->render_and_frame_size_different == 1) {
                    _obp_br(fh->render_width_minus_1, br, 16);
                    _obp_br(fh->render_height_minus_1, br, 16);
                    fh->RenderWidth  = fh->render_width_minus_1 + 1;
                    fh->RenderHeight = fh->render_height_minus_1 + 1;
                } else {
                    fh->RenderWidth  = UpscaledWidth;
                    fh->RenderHeight = FrameHeight;
                }
            } else {
                /* superres_params() */
                uint32_t SuperresDenom;
                if (seq->enable_superres) {
                    _obp_br(fh->superres_params.use_superres, br, 1);
                } else {
                    fh->superres_params.use_superres = 0;
                }
                if (fh->superres_params.use_superres) {
                    _obp_br(fh->superres_params.coded_denom, br, 3);
                    SuperresDenom = fh->superres_params.coded_denom + 9;
                } else {
                    SuperresDenom = 8;
                }
                UpscaledWidth = FrameWidth;
                FrameWidth = (UpscaledWidth * 8 + (SuperresDenom / 2)) / SuperresDenom;
                /* compute_image_size() */
                MiCols = 2 * ((FrameWidth + 7) >> 3);
                MiRows = 2 * ((FrameHeight + 7) >> 3);
            }
        } else {
        /* frame_size() */
            if (fh->frame_size_override_flag) {
                uint8_t n = seq->frame_width_bits_minus_1 + 1;
                _obp_br(fh->frame_width_minus_1, br, n);
                n = seq->frame_height_bits_minus_1 + 1;
                _obp_br(fh->frame_height_minus_1, br, n);
                FrameWidth  = fh->frame_width_minus_1 + 1;
                FrameHeight = fh->frame_height_minus_1 + 1;
            } else {
                FrameWidth  = seq->max_frame_width_minus_1 + 1;
                FrameHeight = seq->max_frame_height_minus_1 + 1;
            }
            /* superres_params() */
            uint32_t SuperresDenom;
            if (seq->enable_superres) {
                _obp_br(fh->superres_params.use_superres, br, 1);
            } else {
                fh->superres_params.use_superres = 0;
            }
            if (fh->superres_params.use_superres) {
                _obp_br(fh->superres_params.coded_denom, br, 3);
                SuperresDenom = fh->superres_params.coded_denom + 9;
            } else {
                SuperresDenom = 8;
            }
            UpscaledWidth = FrameWidth;
            FrameWidth = (UpscaledWidth * 8 + (SuperresDenom / 2)) / SuperresDenom;
            /* compute_image_size() */
            MiCols = 2 * ((FrameWidth + 7) >> 3);
            MiRows = 2 * ((FrameHeight + 7) >> 3);
            /* render_size() */
            _obp_br(fh->render_and_frame_size_different, br, 1);
            if (fh->render_and_frame_size_different == 1) {
                _obp_br(fh->render_width_minus_1, br, 16);
                _obp_br(fh->render_height_minus_1, br, 16);
                fh->RenderWidth  = fh->render_width_minus_1 + 1;
                fh->RenderHeight = fh->render_height_minus_1 + 1;
            } else {
                fh->RenderWidth  = UpscaledWidth;
                fh->RenderHeight = FrameHeight;
            }
        }
        if (fh->force_integer_mv) {
            fh->allow_high_precision_mv = 0;
        } else {
            _obp_br(fh->allow_high_precision_mv, br, 1);
        }
        /* read_interpolation_filer() */
        _obp_br(fh->interpolation_filter.is_filter_switchable, br, 1);
        if (fh->interpolation_filter.is_filter_switchable) {
            fh->interpolation_filter.interpolation_filter = 4;
        } else {
            _obp_br(fh->interpolation_filter.interpolation_filter, br, 2);
        }
        _obp_br(fh->is_motion_mode_switchable, br, 1);
        if (fh->error_resilient_mode && !seq->enable_ref_frame_mvs) {
            fh->use_ref_frame_mvs = 0;
        } else {
            _obp_br(fh->use_ref_frame_mvs, br, 1);
        }
        for (int i = 0; i < 7; i++) {
            int refFrame = 1 + i;
            uint8_t hint = state->RefOrderHint[fh->ref_frame_idx[i]];
            state->OrderHint[refFrame] = hint;
            if (!seq->enable_order_hint) {
                state->RefFrameSignBias[refFrame] = 0;
            } else {
                state->RefFrameSignBias[refFrame] = _obp_get_relative_dist((int32_t) hint, (int32_t) OrderHint, seq);
            }
        }
    }
    return 0;
}
