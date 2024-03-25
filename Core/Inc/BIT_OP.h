/*
 * BIT_OP.h
 *
 *  Created on: Mar 25, 2024
 *      Author: quang
 */

#ifndef INC_BIT_OP_H_
#define INC_BIT_OP_H_

#include "STD_TYPE.h"

#define ACCESS_REG_8BIT(REG)        (*(volatide uint8 *)REG)
#define ACCESS_REG_16BIT(REG)       (*(volatide uint16 *)REG)
#define ACCESS_REG_32BIT(REG)       (*(volatide uint32 *)REG)

#define SET_BIT_REG(REG,POS)        (REG |= (1 << POS))

#define CLR_BIT_REG(REG,POS)        (REG &= ~(1 << POS))

#define TOGGLE_BIT(REG,POS)         (REG ^= (1 << POS))

#define GET_BIT(REG,POS)            ((REG >> POS) & 1 )

#define IS_BIT_SET(REG,POS)         (((REG >> POS) & 1 ) == 0)

#define IS_BIT_CLR(REG,POS)         (!(IS_BIT_SET(REG,BIT_NUM)))


#endif /* INC_BIT_OP_H_ */
