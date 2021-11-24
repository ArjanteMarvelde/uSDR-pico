#ifndef __HMI_H__
#define __HMI_H__
/* 
 * hmi.h
 *
 * Created: Apr 2021
 * Author: Arjan te Marvelde
 *
 * See hmi.c for more information 
 */

extern bool ptt_active;

void hmi_init(void);
void hmi_evaluate(void);

#endif
