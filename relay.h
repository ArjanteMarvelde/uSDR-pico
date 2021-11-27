#ifndef __RELAY_H__
#define __RELAY_H__
/* 
 * relay.h
 *
 * Created: Nov 2021
 * Author: Arjan te Marvelde
 *
 * See relay.c for more information 
 */

#define REL_LPF2	0x01
#define REL_BPF6	0x02
#define REL_BPF12	0x04
#define REL_BPF24	0x08
#define REL_BPF40	0x10

#define REL_ATT_30	0x03
#define REL_ATT_20	0x01
#define REL_ATT_10	0x02
#define REL_ATT_00	0x00
#define REL_PRE_10	0x04

extern void relay_setband(uint8_t val);
extern void relay_setattn(uint8_t val);
extern int relay_getband(void);
extern int relay_getattn(void);
extern void relay_init(void);

#endif
