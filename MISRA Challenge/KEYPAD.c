/*
 * KEYPAD.c
 *
 * Created: 30/01/2016 06:38:37 م
 *  Author: hossam
 */ 
#include "DIO.h"
#include "KEYPAD.h"
/*Local Symbols*/
#define KPD_COL_PORT (PD)
#define KPD_ROW_PORT (PD)
#define KPD_COL_MASK (0x70u)
#define KPD_ROW_MASK (0x0fu)
#define KPD_COL_PIN_NUM (4u)
#define KPD_ROW_PIN_NUM (0u)
/**************************************************/

void KPD_COL_INIT(void)
{
    DIO_InitPortDirection(KPD_COL_PORT,KPD_COL_MASK,0x00u);
}

void KPD_ROW_INIT(void)
{
    DIO_InitPortDirection(KPD_ROW_PORT,KPD_ROW_MASK,0xffu);
    DIO_WritePort(KPD_ROW_PORT,KPD_ROW_MASK,0x00u);
}

void KPD_COL_READ(u8 * VALPTR)
{
    DIO_ReadPort(KPD_COL_PORT,KPD_COL_MASK);
    *(VALPTR) = (*(VALPTR)) >> KPD_COL_PIN_NUM;
}

void KPD_ROW_WRITE(u8 DATA)
{
    DIO_WritePort(KPD_ROW_PORT,KPD_ROW_MASK,((DATA) << KPD_ROW_PIN_NUM));
}


/*#define KPD_COL_INIT() DIO_InitPortDirection(KPD_COL_PORT,KPD_COL_MASK,0x00)
#define KPD_ROW_INIT() DIO_InitPortDirection(KPD_ROW_PORT,KPD_ROW_MASK,0xff); \
                       DIO_InitPortDirection(KPD_ROW_PORT,KPD_ROW_MASK,0x00)

#define KPD_COL_READ(VALPTR) DIO_vidReadPortData(KPD_COL_PORT,KPD_COL_MASK,(VALPTR));\
                             *(VALPTR) = (*(VALPTR)) >> KPD_COL_PIN_NUM


#define KPD_ROW_WRITE(DATA) DIO_vidWritePortData(KPD_ROW_PORT,KPD_ROW_MASK,((DATA) << KPD_ROW_PIN_NUM))
*/
void KPD_Init(void)
{
	KPD_COL_INIT();
	KPD_ROW_INIT();
	
}
void KPD_ReadVal(unsigned char* ValuePtr)
{
    static const unsigned char KeysLut[]= {'1' , '2' , '3' , '4' , '5' , '6','7' , '8' , '9','*' , '0' , '#'};
	unsigned char Rowdata;
	unsigned char ColData;
	unsigned char LoopTermnate = 0u;
	for(Rowdata = 0u ; (Rowdata < 4u) && (LoopTermnate == 0u) ; Rowdata ++)
	{
		KPD_ROW_WRITE((u8)((u8)1 << Rowdata));
		KPD_COL_READ(&ColData);
		if(ColData != 0u)
		{
			*ValuePtr = KeysLut[((Rowdata*3u) + (ColData/2u))];
			LoopTermnate = 1u;
		}
		else
		{
			*ValuePtr = (u8)'n' ;
		}
	}

	
	
}

