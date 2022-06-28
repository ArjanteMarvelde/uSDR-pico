#ifndef __FIX_FFT_H__
#define __FIX_FFT_H__
/* 
 * fix_fft.h
 *
 * Created: Apr 2022
 * Author: Arjan te Marvelde
 *
 * See fix_fft.c for more information 
 */

#define FFT_SIZE	1024				// Use this for buffer allocations
#define FFT_ORDER	10					// FFT_SIZE = 1 << FFT_ORDER

int fix_fft(int16_t *fr, int16_t *fi, bool inverse);

#endif
