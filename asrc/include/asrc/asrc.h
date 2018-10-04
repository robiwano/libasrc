#ifndef __ASRC_ASRC_H__
#define __ASRC_ASRC_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef void* ASRC_HANDLE;

typedef enum ASRC_RESULT {
    ASRC_SUCCESS,
    ASRC_INVALID_ARGUMENT,
} ASRC_RESULT;

typedef enum ASRC_QUALITY {
    ASRC_SINC_BEST_QUALITY   = 0,
    ASRC_SINC_MEDIUM_QUALITY = 1,
    ASRC_SINC_FASTEST        = 2,
    ASRC_ZERO_ORDER_HOLD     = 3,
    ASRC_LINEAR              = 4,
} ASRC_QUALITY;

/**
 * asrc_create - Creates the ASRC
 *
 * @param handle            Pointer to handle
 * @param quality           Quality of SRC
 * @param channels          Number of channels to process
 * @param in_buffer_size    Input buffer size - must not change during lifetime of ASRC
 * @param out_buffer_size   Output buffer size - must not change during lifetime of ASRC
 * @param init_ratio        Set to fs_out / fs_in (f.i. 48000/44100)
 * @returns ASRC_RESULT
 */
ASRC_RESULT asrc_create(ASRC_HANDLE* handle,
                        ASRC_QUALITY quality,
                        int channels,
                        int in_buffer_size,
                        int out_buffer_size,
                        double init_ratio);

/**
 * asrc_adaption_speed - Set ASRC adaption speed
 *
 * @param handle    Handle to ASRC
 * @param speed     Adaption speed, lower is slower [0.001, 1.0]
 * @returns ASRC_RESULT
 */
ASRC_RESULT asrc_adaption_speed(ASRC_HANDLE handle, double speed);

/**
 * asrc_process_input - Process an input block. Number of channels and frames is specified when
 * creating the ASRC.
 *
 * @param handle    Handle to ASRC
 * @param inputs    Array of input buffers.
 * @returns ASRC_RESULT
 */
ASRC_RESULT asrc_process_input(ASRC_HANDLE handle, const float** inputs);

/**
 * asrc_process_output - Process an output block. Number of channels and frames is specified when
 * creating the ASRC.
 *
 * @param handle    Handle to ASRC
 * @param inputs    Array of output buffers.
 * @returns ASRC_RESULT
 */
ASRC_RESULT asrc_process_output(ASRC_HANDLE handle, float** outputs);

/**
 * asrc_destroy - Destroy ASRC
 *
 * @param handle    Handle to ASRC
 * @returns void
 */
void asrc_destroy(ASRC_HANDLE handle);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // __ASRC_ASRC_H__