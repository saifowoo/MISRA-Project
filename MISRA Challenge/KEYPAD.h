/*
 * KEYPAD.h
 *
 * Created: 30/01/2016 07:01:25 م
 *  Author: hossam
 */ 


#ifndef KEYPAD_H_
#define KEYPAD_H_


void KPD_Init(void);
void KPD_ReadVal(unsigned char* ValuePtr);
void KPD_COL_INIT(void);
void KPD_ROW_INIT(void);
void KPD_COL_READ(u8 * VALPTR);
void KPD_ROW_WRITE(u8 DATA);

#endif /* KEYPAD_H_ */
