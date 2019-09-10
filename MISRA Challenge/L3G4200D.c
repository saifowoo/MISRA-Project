﻿/*
 * L3G4200D.c
 *
 * Created: 24/09/2015 09:13:31 م
 *  Author: hossam
 */ 

#include "L3G4200D.h"
#include "L3G4200D_Cfg.h"
#include "L3G4200D_Lcfg.h"
#include "SPI.h"
/*****************************************************************************************************/
/* Local Symbols */
/*****************************************************************************************************/
/*Registers addresses*/
#define WHO_AM_I_REG_ADD    0x0fu
#define CTRL_REG1_ADD       0x20u
#define CTRL_REG2_ADD       0x21u
#define CTRL_REG3_ADD       0x22u
#define CTRL_REG4_ADD       0x23u
#define CTRL_REG5_ADD       0x24u
#define REFERENCE_ADD       0x25u
#define OUT_TEMP_ADD        0x26u
#define STATUS_REG_ADD      0x27u
#define OUT_X_L_ADD         0x28u
#define OUT_X_H_ADD         0x29u
#define OUT_Y_L_ADD         0x2au
#define OUT_Y_H_ADD         0x2bu
#define OUT_Z_L_ADD         0x2cu
#define OUT_Z_H_ADD         0x2du
#define FIFO_CTRL_REG_ADD   0x2eu
#define FIFO_SRC_REG_ADD    0x2fu
#define INT1_CFG_ADD        0x30u
#define INT1_SRC_ADD        0x31u
#define INT1_TSH_XH_ADD     0x32u
#define INT1_TSH_XL_ADD     0x33u
#define INT1_TSH_YH_ADD     0x34u
#define INT1_TSH_YL_ADD     0x35u
#define INT1_TSH_ZH_ADD     0x36u
#define INT1_TSH_ZL_ADD     0x37u
#define INT1_DURATION_ADD   0x38u


void L3G4200D_REG_WRITE(u8 ADD,u8 DATA)
{
    GYHD_ACTIVATE_SLAVE_SELECT();
    SPI_u8TransfereByte(ADD);
    SPI_u8TransfereByte(DATA);
    GYHD_DEACTIVATE_SLAVE_SELECT();
}
void L3G4200D_REG_READ(u8 ADD,u8 *DATA_PTR)
{
    GYHD_ACTIVATE_SLAVE_SELECT();
    SPI_u8TransfereByte(ADD|0x80u);
    (*DATA_PTR) = SPI_u8TransfereByte(0x00u);
    GYHD_DEACTIVATE_SLAVE_SELECT();
}

/*****************************************************************************************************/
/*General Write Read Macros*/
/*
#define L3G4200D_REG_WRITE(ADD,DATA) GYHD_ACTIVATE_SLAVE_SELECT();\
                                     SPI_u8TransfereByte(ADD);\
									 SPI_u8TransfereByte(DATA);\
									 GYHD_DEACTIVATE_SLAVE_SELECT()

#define L3G4200D_REG_READ(ADD,DATA_PTR) GYHD_ACTIVATE_SLAVE_SELECT();\
                                     SPI_u8TransfereByte(ADD|0x80);\
									 (*DATA_PTR) = SPI_u8TransfereByte(0x00);\
									 GYHD_DEACTIVATE_SLAVE_SELECT()
 */
/*****************************************************************************************************/
/* MAcros To Axis Registers*/

void L3G4200D_READ_WHO_AM_I(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(WHO_AM_I_REG_ADD,DATA_PTR);
}

void L3G4200D_READ_CTRL_REG1(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(CTRL_REG1_ADD,DATA_PTR);
}
void L3G4200D_WRITE_CTRL_REG1(u8 DATA)
{
    L3G4200D_REG_WRITE(CTRL_REG1_ADD,DATA);
}
void L3G4200D_READ_CTRL_REG2(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(CTRL_REG2_ADD,DATA_PTR);
}
void L3G4200D_WRITE_CTRL_REG2(u8 DATA)
{
    L3G4200D_REG_WRITE(CTRL_REG2_ADD,DATA);
}
void L3G4200D_READ_CTRL_REG3(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(CTRL_REG3_ADD,DATA_PTR);
}
void L3G4200D_WRITE_CTRL_REG3(u8 DATA)
{
    L3G4200D_REG_WRITE(CTRL_REG3_ADD,DATA);
}
void L3G4200D_READ_CTRL_REG4(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(CTRL_REG4_ADD,DATA_PTR);
}
void L3G4200D_WRITE_CTRL_REG4(u8 DATA)
{
    L3G4200D_REG_WRITE(CTRL_REG4_ADD,DATA);
}
void L3G4200D_READ_CTRL_REG5(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(CTRL_REG5_ADD,DATA_PTR);
}
void L3G4200D_WRITE_CTRL_REG5(u8 DATA)
{
    L3G4200D_REG_WRITE(CTRL_REG5_ADD,DATA);
}
void L3G4200D_READ_REFERENCE(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(REFERENCE_ADD,DATA_PTR);
}
void L3G4200D_WRITE_REFERENCE(u8 DATA)
{
    L3G4200D_REG_WRITE(REFERENCE_ADD,DATA);
}
void L3G4200D_READ_OUT_TEMP(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(OUT_TEMP_ADD,DATA_PTR);
}
void L3G4200D_READ_STATUS_REG(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(STATUS_REG_ADD,DATA_PTR);
}
void L3G4200D_READ_OUT_X_L(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(OUT_X_L_ADD,DATA_PTR);
}
void L3G4200D_READ_OUT_X_H(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(OUT_X_H_ADD,DATA_PTR);
}
void L3G4200D_READ_OUT_Y_L(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(OUT_Y_L_ADD,DATA_PTR);
}
void L3G4200D_READ_OUT_Y_H(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(OUT_Y_H_ADD,DATA_PTR);
}
void L3G4200D_READ_OUT_Z_L(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(OUT_Z_L_ADD,DATA_PTR);
}
void L3G4200D_READ_OUT_Z_H(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(OUT_Z_H_ADD,DATA_PTR);
}
void L3G4200D_READ_FIFO_CTRL_REG(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(FIFO_CTRL_REG_ADD,DATA_PTR);
}
void L3G4200D_WRITE_FIFO_CTRL_REG(u8 DATA)
{
    L3G4200D_REG_WRITE(FIFO_CTRL_REG_ADD,DATA);
}
void L3G4200D_READ_FIFO_SRC_REG(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(FIFO_SRC_REG_ADD,DATA_PTR);
}
void L3G4200D_READ_INT1_CFG(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(INT1_CFG_ADD,DATA_PTR);
}
void L3G4200D_WRITE_INT1_CFG(u8 DATA)
{
    L3G4200D_REG_WRITE(INT1_CFG_ADD,DATA);
}
void L3G4200D_READ_INT1_SRC(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(INT1_SRC_ADD,DATA_PTR);
}
void L3G4200D_READ_INT1_TSH_XH(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(INT1_TSH_XH_ADD,DATA_PTR);
}
void L3G4200D_WRITE_INT1_TSH_XH(u8 DATA)
{
    L3G4200D_REG_WRITE(INT1_TSH_XH_ADD,DATA);
}
void L3G4200D_READ_INT1_TSH_XL(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(INT1_TSH_XL_ADD,DATA_PTR);
}
void L3G4200D_WRITE_INT1_TSH_XL(u8 DATA)
{
    L3G4200D_REG_WRITE(INT1_TSH_XL_ADD,DATA);
}
void L3G4200D_READ_INT1_TSH_YH(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(INT1_TSH_YH_ADD,DATA_PTR);
}
void L3G4200D_WRITE_INT1_TSH_YH(u8 DATA)
{
    L3G4200D_REG_WRITE(INT1_TSH_YH_ADD,DATA);
}
void L3G4200D_READ_INT1_TSH_YL(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(INT1_TSH_YL_ADD,DATA_PTR);
}
void L3G4200D_WRITE_INT1_TSH_YL(u8 DATA)
{
    L3G4200D_REG_WRITE(INT1_TSH_YL_ADD,DATA);
}
void L3G4200D_READ_INT1_TSH_ZH(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(INT1_TSH_ZH_ADD,DATA_PTR);
}
void L3G4200D_WRITE_INT1_TSH_ZH(u8 DATA)
{
    L3G4200D_REG_WRITE(INT1_TSH_ZH_ADD,DATA);
}
void L3G4200D_READ_INT1_TSH_ZL(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(INT1_TSH_ZL_ADD,DATA_PTR);
}
void L3G4200D_WRITE_INT1_TSH_ZL(u8 DATA)
{
    L3G4200D_REG_WRITE(INT1_TSH_ZL_ADD,DATA);
}
void L3G4200D_READ_INT1_DURATION(u8 * DATA_PTR)
{
    L3G4200D_REG_READ(INT1_DURATION_ADD,DATA_PTR);
}
void L3G4200D_WRITE_INT1_DURATION(u8 DATA)
{
    L3G4200D_REG_WRITE(INT1_DURATION_ADD,DATA);
}

/*
 #define L3G4200D_READ_WHO_AM_I(DATA_PTR)           L3G4200D_REG_READ(WHO_AM_I_REG_ADD,DATA_PTR)
#define L3G4200D_READ_CTRL_REG1(DATA_PTR)          L3G4200D_REG_READ(CTRL_REG1_ADD,DATA_PTR)
#define L3G4200D_WRITE_CTRL_REG1(DATA)             L3G4200D_REG_WRITE(CTRL_REG1_ADD,DATA)
#define L3G4200D_READ_CTRL_REG2(DATA_PTR)          L3G4200D_REG_READ(CTRL_REG2_ADD,DATA_PTR)
#define L3G4200D_WRITE_CTRL_REG2(DATA)             L3G4200D_REG_WRITE(CTRL_REG2_ADD,DATA)
#define L3G4200D_READ_CTRL_REG3(DATA_PTR)          L3G4200D_REG_READ(CTRL_REG3_ADD,DATA_PTR)
#define L3G4200D_WRITE_CTRL_REG3(DATA)             L3G4200D_REG_WRITE(CTRL_REG3_ADD,DATA)
#define L3G4200D_READ_CTRL_REG4(DATA_PTR)          L3G4200D_REG_READ(CTRL_REG4_ADD,DATA_PTR)
#define L3G4200D_WRITE_CTRL_REG4(DATA)             L3G4200D_REG_WRITE(CTRL_REG4_ADD,DATA)
#define L3G4200D_READ_CTRL_REG5(DATA_PTR)          L3G4200D_REG_READ(CTRL_REG5_ADD,DATA_PTR)
#define L3G4200D_WRITE_CTRL_REG5(DATA)             L3G4200D_REG_WRITE(CTRL_REG5_ADD,DATA)
#define L3G4200D_READ_REFERENCE(DATA_PTR)          L3G4200D_REG_READ(REFERENCE_ADD,DATA_PTR)
#define L3G4200D_WRITE_REFERENCE(DATA)             L3G4200D_REG_WRITE(REFERENCE_ADD,DATA)
#define L3G4200D_READ_OUT_TEMP(DATA_PTR)           L3G4200D_REG_READ(OUT_TEMP_ADD,DATA_PTR)
#define L3G4200D_READ_STATUS_REG(DATA_PTR)         L3G4200D_REG_READ(STATUS_REG_ADD,DATA_PTR)
#define L3G4200D_READ_OUT_X_L(DATA_PTR)            L3G4200D_REG_READ(OUT_X_L_ADD,DATA_PTR)
#define L3G4200D_READ_OUT_X_H(DATA_PTR)            L3G4200D_REG_READ(OUT_X_H_ADD,DATA_PTR)
#define L3G4200D_READ_OUT_Y_L(DATA_PTR)            L3G4200D_REG_READ(OUT_Y_L_ADD,DATA_PTR)
#define L3G4200D_READ_OUT_Y_H(DATA_PTR)            L3G4200D_REG_READ(OUT_Y_H_ADD,DATA_PTR)
#define L3G4200D_READ_OUT_Z_L(DATA_PTR)            L3G4200D_REG_READ(OUT_Z_L_ADD,DATA_PTR)
#define L3G4200D_READ_OUT_Z_H(DATA_PTR)            L3G4200D_REG_READ(OUT_Z_H_ADD,DATA_PTR)
#define L3G4200D_READ_FIFO_CTRL_REG(DATA_PTR)      L3G4200D_REG_READ(FIFO_CTRL_REG_ADD,DATA_PTR)
#define L3G4200D_WRITE_FIFO_CTRL_REG(DATA)         L3G4200D_REG_WRITE(FIFO_CTRL_REG_ADD,DATA)
#define L3G4200D_READ_FIFO_SRC_REG(DATA_PTR)       L3G4200D_REG_READ(FIFO_SRC_REG_ADD,DATA_PTR)
#define L3G4200D_READ_INT1_CFG(DATA_PTR)           L3G4200D_REG_READ(INT1_CFG_ADD,DATA_PTR)
#define L3G4200D_WRITE_INT1_CFG(DATA)              L3G4200D_REG_WRITE(INT1_CFG_ADD,DATA)
#define L3G4200D_READ_INT1_SRC(DATA_PTR)           L3G4200D_REG_READ(INT1_SRC_ADD,DATA_PTR)
#define L3G4200D_READ_INT1_TSH_XH(DATA_PTR)        L3G4200D_REG_READ(INT1_TSH_XH_ADD,DATA_PTR)
#define L3G4200D_WRITE_INT1_TSH_XH(DATA)           L3G4200D_REG_WRITE(INT1_TSH_XH_ADD,DATA)
#define L3G4200D_READ_INT1_TSH_XL(DATA_PTR)        L3G4200D_REG_READ(INT1_TSH_XL_ADD,DATA_PTR)
#define L3G4200D_WRITE_INT1_TSH_XL(DATA)           L3G4200D_REG_WRITE(INT1_TSH_XL_ADD,DATA)
#define L3G4200D_READ_INT1_TSH_YH(DATA_PTR)        L3G4200D_REG_READ(INT1_TSH_YH_ADD,DATA_PTR)
#define L3G4200D_WRITE_INT1_TSH_YH(DATA)           L3G4200D_REG_WRITE(INT1_TSH_YH_ADD,DATA)
#define L3G4200D_READ_INT1_TSH_YL(DATA_PTR)        L3G4200D_REG_READ(INT1_TSH_YL_ADD,DATA_PTR)
#define L3G4200D_WRITE_INT1_TSH_YL(DATA)           L3G4200D_REG_WRITE(INT1_TSH_YL_ADD,DATA)
#define L3G4200D_READ_INT1_TSH_ZH(DATA_PTR)        L3G4200D_REG_READ(INT1_TSH_ZH_ADD,DATA_PTR)
#define L3G4200D_WRITE_INT1_TSH_ZH(DATA)           L3G4200D_REG_WRITE(INT1_TSH_ZH_ADD,DATA)
#define L3G4200D_READ_INT1_TSH_ZL(DATA_PTR)        L3G4200D_REG_READ(INT1_TSH_ZL_ADD,DATA_PTR)
#define L3G4200D_WRITE_INT1_TSH_ZL(DATA)           L3G4200D_REG_WRITE(INT1_TSH_ZL_ADD,DATA)
#define L3G4200D_READ_INT1_DURATION(DATA_PTR)      L3G4200D_REG_READ(INT1_DURATION_ADD,DATA_PTR)
#define L3G4200D_WRITE_INT1_DURATION(DATA)         L3G4200D_REG_WRITE(INT1_DURATION_ADD,DATA)
 */

/*****************************************************************************************************/
/*Register Default Value*/
#define L3G4200D_WHO_AM_I_DEFAULT 0xD3u
/*****************************************************************************************************/
/*Operational Modes Management*/
#define u8PWR_DN_MOD 0x00u
#define u8SLEEP_MOD  0x08u
#define u8NORMAL_MOD 0x0fu
#define u8MOD_CLR_MASK 0xf0u
#define u8TO_SLEEP_MODE_DELAY_MS 0x05u
/*****************************************************************************************************/
/*CTRL_REG4 Initialization*/
#define u8BLOCK_DATA_UPDATE 0x00u
#define	u8BIG_LITTLE_ENDIAN 0x00u
#define u8SELF_TEST_MODE    0x00u
#define u8SPI_MODE          0x00u
#define u8CTRL_REG4_INIT (u8BLOCK_DATA_UPDATE | u8BIG_LITTLE_ENDIAN | u8SELF_TEST_MODE | u8SPI_MODE)
/*****************************************************************************************************/
/*Reading Operation*/
#define u8NEW_DATA_AVAILABEL_X_MASK 0x01u
#define u8NEW_DATA_AVAILABEL_Y_MASK 0x02u
#define u8NEW_DATA_AVAILABEL_Z_MASK 0x04u
#define u8DATA_OVER_WRITTEN_MASK    0x80u
#define u8AXIS_IS_ENABLED           0x00u
#define u8AXIS_IS_DISABLED          0x01u
/*****************************************************************************************************/
/*Filters Configuration*/
#define u8FILTERS_CLR_MASK 0xE0u
/*****************************************************************************************************/
/*Self Axis Movement Config*/
#if (u8SELF_AXIS_MOV == (ON))
#define u8INT1_ENABLE 0x80u
#endif
/*****************************************************************************************************/
/*Static Variables*/
/*****************************************************************************************************/
static u8 GYHD_u8LastPowerMode = u8PWR_DN_MOD;		
static u8 GYHD_u8TimeOutFlag;
static u8 GYHD_u8StartTimeoutFlag;	

/*****************************************************************************************************/
/*Private functions declarations*/
/*****************************************************************************************************/
static u8 u8CheckTimeOut(u8 u8TimeInMs);
static void vidManageOperationModes(u8 u8NewMode);
static u8 udtSelfTest(void);
static void vidInitCtrlReg4(void);
static void vidCheckForNewData(u8* pu8Status, u8 u8AxisMask);
static u16 u16ReadAxisData(u8 AxisMask);
static u8 u8CheckAxisIsEnabled(u8 u8AxisMask);
static void vidInitFilters(void);
#if (u8SELF_AXIS_MOV == ON)
static void vidInitSelectiveAxisMovement(void);
#endif
/*****************************************************************************************************/
/* Global Services */
/*****************************************************************************************************/
u8 GYHD_Init(void)
{
    u8 RetVal;
    GYHD_INIT_SLAVE_SELECT();
    SPI_vidInit();
    GYHD_u8TimeOutFlag = 0u;
    GYHD_u8StartTimeoutFlag = 0u;
    /*To Do: Initialized Module Registers with the Specific values*/
    /*Wait for Entering Power Down Mode*/
    do
    {
        u8START_TIME_OUT_MS((u32)250,&GYHD_u8TimeOutFlag);
    }while (GYHD_u8TimeOutFlag == 0u);
    GYHD_u8TimeOutFlag = 0u;

    /*Perform Sensor Self Test*/
    RetVal = udtSelfTest();
    if(RetVal == OK)
    {
        /*Init Power Mode*/
        vidManageOperationModes(u8PWR_DN_MOD);
        GYHD_u8LastPowerMode = u8PWR_DN_MOD;

        /*Init FullScale, Data Order, Spi Mode and block data update*/
        vidInitCtrlReg4();

        /*Init Filters*/
        vidInitFilters();
    }
    else
    {
        /* Self Test Failed, a Problem in Communication Found*/
    }
    return RetVal;
}
/*****************************************************************************************************/
void GYHD_WakeUpModule(u8* u8WakeUpStatePtr)
{
    vidManageOperationModes(u8NORMAL_MOD);
    if((GYHD_u8LastPowerMode == u8NORMAL_MOD) && (GYHD_u8StartTimeoutFlag == 0u))
    {
        *u8WakeUpStatePtr = u8WAKEUP_DN;
    }
    else
    {
        *u8WakeUpStatePtr = u8WAKEUP_IN_PROGRESS;
    }

}
/*****************************************************************************************************/
void GYHD_SleepModule(void)
{
    vidManageOperationModes(u8SLEEP_MOD);
}
/*****************************************************************************************************/
u8 GYHD_ReadXDirection(u16* pu16Data,u8* pu8ReadingStatus)
{
    u8 RetVar = NOK;
    u8 u8IsEnabled;
    u8 u8Axis_Mask = u8NEW_DATA_AVAILABEL_X_MASK;
    /*Check if Axis was enabled*/
    u8IsEnabled = u8CheckAxisIsEnabled(u8Axis_Mask);
    if(u8IsEnabled == u8AXIS_IS_ENABLED)
    {
        /*Check for Data Updated*/
        vidCheckForNewData(pu8ReadingStatus,u8Axis_Mask);
        if(*pu8ReadingStatus == u8READING_OK)
        {
            /*Read Data*/
            *pu16Data = u16ReadAxisData(u8Axis_Mask);
            RetVar = OK;
        }
        else
        {
            /*Data overwritten and Pointer Initialized*/
        }
    }
    else
    {
        /*Axis Disabled*/
        *pu8ReadingStatus = u8AXIS_DISABLED;
    }

    return RetVar;
}
/*****************************************************************************************************/
u8 GYHD_ReadYDirection(u16* pu16Data,u8* pu8ReadingStatus)
{
    u8 Ret_Var = NOK;
    u8 u8IsEnable;
    u8 u8AxisMask_ = u8NEW_DATA_AVAILABEL_Y_MASK;
    /*Check if Axis was enabled*/
    u8IsEnable = u8CheckAxisIsEnabled(u8AxisMask_);
    if(u8IsEnable == u8AXIS_IS_ENABLED)
    {
        /*Check for Data Updated*/
        vidCheckForNewData(pu8ReadingStatus,u8AxisMask_);
        if(*pu8ReadingStatus == u8READING_OK)
        {
            /*Read Data*/
            *pu16Data = u16ReadAxisData(u8AxisMask_);
            Ret_Var = OK;
        }
        else
        {
            /*Data overwritten and Pointer Initialized*/
        }
    }
    else
    {
        /*Axis Disabled*/
        *pu8ReadingStatus = u8AXIS_DISABLED;
    }

    return Ret_Var;
}
/*****************************************************************************************************/
u8 GYHD_ReadZDirection(u16* pu16Data,u8* pu8ReadingStatus)
{
    u8 RetVar_ = NOK;
    u8 u8IsEnabled_;
    u8 U8AxisMask = u8NEW_DATA_AVAILABEL_Z_MASK;
    /*Check if Axis was enabled*/
    u8IsEnabled_ = u8CheckAxisIsEnabled(U8AxisMask);
    if(u8IsEnabled_ == u8AXIS_IS_ENABLED)
    {
        /*Check for Data Updated*/
        vidCheckForNewData(pu8ReadingStatus,U8AxisMask);
        if(*pu8ReadingStatus == u8READING_OK)
        {
            /*Read Data*/
            *pu16Data = u16ReadAxisData(U8AxisMask);
            RetVar_ = OK;
        }
        else
        {
            /*Data overwritten and Pointer Initialized*/
        }
    }
    else
    {
        /*Axis Disabled*/
        *pu8ReadingStatus = u8AXIS_DISABLED;
    }

    return RetVar_;
}
/*****************************************************************************************************/
/*Private functions definitions*/
/*****************************************************************************************************/
static void vidCheckForNewData(u8* pu8Status, u8 u8AxisMask)
{
    u8 u8Status = 0u;
    L3G4200D_READ_STATUS_REG(&u8Status);
    /*Check for Data updated*/
    if((u8Status & u8AxisMask) == u8AxisMask)
    {
        /*Check for Data Overwritten*/
        if ((u8Status & u8DATA_OVER_WRITTEN_MASK) == u8DATA_OVER_WRITTEN_MASK)
        {
            *pu8Status = u8DATA_OVERWRITTEN;
        }
        else
        {
            *pu8Status = u8READING_OK;
        }
    }
    else
    {
        *pu8Status = u8NO_DATA_UPDATED;
    }

}
/*****************************************************************************************************/
static u8 u8CheckAxisIsEnabled(u8 u8AxisMask)
{
    u8 u8IsEnable_;
    u8 u8RegValue = 0u;
    /*Read Control Register*/
    L3G4200D_READ_CTRL_REG1(&u8RegValue);
    /*Check for access Status*/
    if((u8RegValue & u8AxisMask) == u8AxisMask)
    {
        u8IsEnable_ = u8AXIS_IS_ENABLED;
    }
    else
    {
        u8IsEnable_ = u8AXIS_IS_DISABLED;
    }
    return u8IsEnable_;
}
/*****************************************************************************************************/
static u16 u16ReadAxisData(u8 AxisMask)
{
    u16 u16AxisData;
    u8 u8RegData1 = 0u;
    u8 u8RegData2 = 0u;
    switch(AxisMask)
    {
    case u8NEW_DATA_AVAILABEL_X_MASK:
    {
        L3G4200D_READ_OUT_X_L(&u8RegData1);
        L3G4200D_READ_OUT_X_H(&u8RegData2);
        u16AxisData = u8RegData1 | ((u16)(((8u)(u8RegData2)<<8u)));
    }
    break;
    case u8NEW_DATA_AVAILABEL_Y_MASK:
    {
        L3G4200D_READ_OUT_Y_L(&u8RegData1);
        L3G4200D_READ_OUT_Y_H(&u8RegData2);
        u16AxisData = u8RegData1 | ((u16)(((8u)(u8RegData2)<<8u)));
    }
    break;
    case u8NEW_DATA_AVAILABEL_Z_MASK:
    {
        L3G4200D_READ_OUT_Z_L(&u8RegData1);
        L3G4200D_READ_OUT_Z_H(&u8RegData2);
        u16AxisData = u8RegData1 | ((u16)(((8u)(u8RegData2)<<8u)));
    }
    break;
    default:
        u16AxisData = 0x00u;
        break;
    }
    return u16AxisData;
}

/*****************************************************************************************************/
static void vidInitCtrlReg4(void)
{
    static u8 GYHD_u8FullScale;
    u8 u8InitParams = 0x00;
    GYHD_u8FullScale = L3G4200D_ConfigParam.u8FullScaleValue;
    u8InitParams|= (((u8)GYHD_u8FullScale) | ((u8)u8CTRL_REG4_INIT));
    L3G4200D_WRITE_CTRL_REG4(u8InitParams);
}	
/*****************************************************************************************************/
static void vidManageOperationModes(u8 u8NewMode)
{
    u8 u8RegData = 0u;
    switch(u8NewMode)
    {
    case u8PWR_DN_MOD:
    {
        /*Clear the Power Mode bits in the register and write the mode values*/
        L3G4200D_READ_CTRL_REG1(&u8RegData);
        u8RegData &= u8MOD_CLR_MASK;
        u8RegData |= u8NewMode;
        L3G4200D_WRITE_CTRL_REG1(u8RegData);
        GYHD_u8LastPowerMode = u8PWR_DN_MOD;
    }
    break;

    case u8SLEEP_MOD:
    {
        /*Clear the Power Mode bits in the register and write the mode values*/
        L3G4200D_READ_CTRL_REG1(&u8RegData);
        u8RegData &= u8MOD_CLR_MASK;
        u8RegData |= u8NewMode;
        L3G4200D_WRITE_CTRL_REG1(u8RegData);
        GYHD_u8LastPowerMode = u8SLEEP_MOD;
    }
    break;

    case u8NORMAL_MOD:
    {
        /*Wait for 250ms*/
        if(GYHD_u8StartTimeoutFlag == 0u)
        {
            /*Clear axis activation bits*/
            u8NewMode &= 0xF8u;
            /*Write Activation signals*/
            u8NewMode |= ((L3G4200D_ConfigParam.strAxisActivation.u8XAxisActivation) | (L3G4200D_ConfigParam.strAxisActivation.u8YAxisActivation) | (L3G4200D_ConfigParam.strAxisActivation.u8ZAxisActivation));
            GYHD_u8TimeOutFlag = 0u;
            /*Clear the Power Mode bits in the register and write the mode values*/
            L3G4200D_READ_CTRL_REG1(&u8RegData);
            u8RegData &= u8MOD_CLR_MASK;
            u8RegData |= u8NewMode;
            L3G4200D_WRITE_CTRL_REG1(u8RegData);
        }
        else
        {
            if(GYHD_u8LastPowerMode == u8SLEEP_MOD)
            {
                GYHD_u8TimeOutFlag = u8CheckTimeOut(u8TO_SLEEP_MODE_DELAY_MS);
            }
            else
            {
                GYHD_u8TimeOutFlag = u8CheckTimeOut(250u);
            }

            if(GYHD_u8TimeOutFlag == 1u)
            {
                GYHD_u8StartTimeoutFlag = 0u;
                GYHD_u8LastPowerMode = u8NORMAL_MOD;
            }
            else
            {
                /* Do nothing: The timeout didn't pass*/
            }
        }
    }
    break;
    default:
        break;
    }
}
/*****************************************************************************************************/
static u8 u8CheckTimeOut(u8 u8TimeInMs)
{
    u8 u8TimeOutFlag = 0u;
    u8START_TIME_OUT_MS(u8TO_SLEEP_MODE_DELAY_MS,&u8TimeOutFlag);
    GYHD_u8StartTimeoutFlag = 1u;
    return u8TimeOutFlag;
}
/*****************************************************************************************************/
static u8 udtSelfTest(void)
{
    u8 RetVal1;
    u8 RegData = 0u;
    /*Read the Who AM I? Register*/
    L3G4200D_READ_WHO_AM_I(&RegData);
    if(RegData == L3G4200D_WHO_AM_I_DEFAULT)
    {
        RetVal1 = OK;
    }
    else
    {
        RetVal1 = NOK;
    }
    return RetVal1;
}
/*****************************************************************************************************/
static void vidInitFilters(void)
{
    u8 u8RegData_1 = 0u;
    L3G4200D_READ_CTRL_REG5(&u8RegData_1);
    u8RegData_1 &= u8FILTERS_CLR_MASK;
    u8RegData_1 |= ((L3G4200D_ConfigParam.strFilterCfg.u8FilterOnData) | (L3G4200D_ConfigParam.strFilterCfg.u8FilterOnInterrupt));
    L3G4200D_WRITE_CTRL_REG5(u8RegData_1);
}
/*****************************************************************************************************/
#if(u8SELF_AXIS_MOV == ON)
static void vidInitSelectiveAxisMovement(void)
{
    u8 u8RegData_2 = 0u;
    u8 u8Threshold;
    /*Enable Interrupt 1*/
    L3G4200D_WRITE_CTRL_REG5(u8INT1_ENABLE);

    /*Write threshold values for each axis*/
    u8Threshold = (u8)L3G4200D_ConfigParam.strIntCfg.u16ThresholdX;
    L3G4200D_WRITE_INT1_TSH_XL(u8Threshold);
    u8Threshold = (u8)((u16)(L3G4200D_ConfigParam.strIntCfg.u16ThresholdX>>8U));
    L3G4200D_WRITE_INT1_TSH_XH(u8Threshold);

    u8Threshold = (u8)L3G4200D_ConfigParam.strIntCfg.u16ThresholdY;
    L3G4200D_WRITE_INT1_TSH_YL(u8Threshold);
    u8Threshold = (u8)((u16)(L3G4200D_ConfigParam.strIntCfg.u16ThresholdY>>8U));
    L3G4200D_WRITE_INT1_TSH_YH(u8Threshold);

    u8Threshold = (u8)L3G4200D_ConfigParam.strIntCfg.u16ThresholdZ;
    L3G4200D_WRITE_INT1_TSH_ZL(u8Threshold);
    u8Threshold = (u8)((u16)(L3G4200D_ConfigParam.strIntCfg.u16ThresholdZ>>8U));
    L3G4200D_WRITE_INT1_TSH_ZH(u8Threshold);

    /*Write Interrupt Duration*/
    L3G4200D_WRITE_INT1_DURATION(L3G4200D_ConfigParam.strIntCfg.u8IntDuration);

    /*Write Interrupt Activation*/
    u8RegData_2 =  ( ((u8)L3G4200D_ConfigParam.strIntCfg.u8XAxisInterrupt) | ((u8)L3G4200D_ConfigParam.strIntCfg.u16ThresholdY) | ((u8)L3G4200D_ConfigParam.strIntCfg.u16ThresholdZ) );
    L3G4200D_WRITE_INT1_CFG(u8RegData_2);
}
#endif
