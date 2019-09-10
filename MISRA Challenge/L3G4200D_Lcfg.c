/*
 * L3G4200D_Lcfg.c
 *
 * Created: 27/09/2015 02:11:03 م
 *  Author: hossam
 */ 
#include "L3G4200D_Lcfg.h"
#include "Basic_Types.h"
#include "L3G4200D_Lcfg.h"
#include "L3G4200D.h"

void GYHD_INIT_SLAVE_SELECT(void)
{
    DIO_InitPortDirection(PB,(1u)<<4,(1u)<<4);
    DIO_WritePort(PB,(1u)<<4,(1u)<<4);
}

void GYHD_ACTIVATE_SLAVE_SELECT(void)
{
    u8 data = 1u;
    DIO_WritePort(PB,(((u8)(~data))<<4),(1u)<<4);
}

void GYHD_DEACTIVATE_SLAVE_SELECT(void)
{
    DIO_WritePort(PB,((1u)<<4),((1u)<<4));
}

void u8START_TIME_OUT_MS(u32 DELAY_MS,u8 * FLAG_PTR)
{
    _delay_ms(DELAY_MS);
    *(FLAG_PTR) = 0x01u;
}


const L3G4200D_CfgType L3G4200D_ConfigParam = { {u8FS_250},
                                                {u8X_AXIS_ACTIVE,u8Y_AXIS_ACTIVE,u8Z_AXIS_ACTIVE},
                                                {u8NO_FILTERS,u8NO_FILTERS},
                                                {0X2CA4,0x2CA4,0x2CA4,u8INT1_X,u8INT1_Y,u8INT1_Z,0x00,0x00}
};

