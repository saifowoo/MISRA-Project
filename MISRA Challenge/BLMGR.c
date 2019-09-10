#include "Basic_Types.h"
#include "BLTD.h"
#include "BLMGR.h"
#include "CRC.h"
#include "DIO.h"
#include "UART_Drv.h"
#include "BLMGR_CFG.h"


void _delay_ms(u32 delay);
/*********************************************************************************/
/*Local Symbols*/
/*********************************************************************************/
/*Paring States*/
#define PAIRING_STATE_IDLE                  (0xffu)
#define PAIRING_STATE_INITIALIZING          (0x00u)
#define PAIRING_STATE_WAIT_INIT_RESP        (0x01u)
#define PAIRING_STATE_INQUIRE               (0x02u)
#define PAIRING_STATE_WAIT_INQUIRE_RESP     (0x03u)
#define PAIRING_STATE_WAIT_PAIR_REQ         (0x04u)
#define PAIRING_STATE_CONNECTED_DONE        (0x05u)
#define PAIRING_STATE_FAILED                (0x06u)
#define PAIRING_STATE_START_WAIT_PAIR_REQ   (0x07u)
/*********************************************************************************/
/*Handshaking states*/
#define HANDSHAKE_STATE_IDLE             (0xffu)
#define HANDSHAKE_STATE_SEND_ID_FRAME    (0x01u)
#define HANDSHAKE_STATE_RECV_ID_FRAME    (0x02u)
#define HANDSHAKE_STATE_SEND_VAL_FRMAE   (0x03u)
#define HANDSHAKE_STATE_RECV_VAL_FRAME   (0x04u)
#define HANDSHAKE_STATE_SEND_ERR_FRAME   (0x05u)
#define HANDSHAKE_STATE_HANDSHAKING_DONE (0x06u)
#define HANDSHAKE_STATE_FAILED           (0x07u)
/*********************************************************************************/
/*Communication states*/
#define COMM_STATE_IDLE             (0xffu)
#define COMM_STATE_SEND_DATA_FRAME  (0x01u)
#define COMM_STATE_RECV_DATA_FRAME  (0x02u)
#define COMM_STATE_SEND_ERROR_FRAME (0x03u)
#define COMM_STATE_FAILED           (0x04u)
/*********************************************************************************/
/*Bluetooth States*/
#define BLUETOOTH_STATE_STOPPED       (0xffu)
#define BLUETOOTH_STATE_DISCONNECTED  (0x00u)
#define BLUETOOTH_STATE_PAIRING       (0x01u)
#define BLUETOOTH_STATE_HANDSHAKING   (0x02u)
#define BLUETOOTH_STATE_COMMUNICATION (0x03u)
/*********************************************************************************/
/*Lengths Configuration*/
#define ID_FRAME_LENGTH        (18u)
#define VAL_FRAME_LENGTH       (18u)
#define DATA_FRAME_LENGTH      (18u)
#define ERROR_FRAME_LENGTH     (18u)
#define MAX_DEV_NAME_LENGTH    (6u)
#define MAX_DATA_BUFFER_LENGTH (18u)
/*********************************************************************************/
/*Timeout Counts Configuration*/
#define PAIRING_MAX_COUNTS      (1u)
#define HANDSHAKING_MAX_COUNTS  (10u)
#define COMM_MAX_COUNTS         (10u)
#define MAX_PAIRING_FAIL_REPT   (10u)
#define MAX_HANDSHAKE_FAIL_REP  (5u)
#define MAX_COMM_FAIL_REP       (20u)
#define MAX_DISCONNECTION_COUNT (2u)
/*********************************************************************************/
/*Error States*/
#define ERRH_TIMEOUT_ERROR             (0x00u)
#define ERRH_HANDSHAKE_ERROR           (0x01u)
#define ERRH_CHECKSUM_ERROR            (0x03u)
#define ERRH_INVALID_FRAME_ERROR       (0x04u)
#define ERRH_CRC_ERROR                 (0x05u)
#define ERRH_INVALID_RECEIVER_ERROR    (0x06u)
#define ERRH_WRONG_FRAME_ERROR         (0x07u)
#define ERRH_NO_ERROR                  (0xffu)
/*********************************************************************************/
/*Buffer Indices*/
#define FRAME_HEADER_IDX   (0x00u)
#define FRAME_SENDER_IDX   (0x02u)
#define FRAME_RECVER_IDX   (0x03u)
#define FRAME_TYPE_IDX     (0x04u)
#define PARAM_LENGTH_IDX   (0x05u)
#define OS_TYPE_IDX        (0x06u)
#define DEV_TYPE_IDX       (0x07u)
#define DEV_NAME_IDX       (0x08u)
#define FRAME_CRC_IDX      (0x0Eu)
#define FRAME_CHECKSUM_IDX (0x10u)
#define FRAME_TAIL_IDX     (0x11u)
#define FRAME_VAL_CRC_IDX  (0x06u)
#define BATT_LEVEL_IDX     (0x06u)
#define DIRECTION_IDX      (0x06u)
#define SPEED_DEGREE_IDX   (0x07u)
#define ERROR_TYPE_IDX     (0x06u)
/*********************************************************************************/
/*Default Values*/
#define TX_OS_TYPE       (0xffu)
#define TX_DEV_TYPE      (0x04u)
#define TX_FRAME_DEFUALT (0x00u)
#define TX_CRC_DEFAULT   (0xffu)
/*********************************************************************************/
/*Frame types*/
#define FRAME_TYPE_ID_FRAME    (0x01u)
#define FRAME_TYPE_VAL_FRAME   (0x02u)
#define FRAME_TYPE_DATA_FRAME  (0x03u)
#define FRAME_TYPE_ERROR_FRAME (0x04u)
/*********************************************************************************/
/*Error Types*/
#define ERROR_TYPE_RESEND_LAST_FRMAE     (0x01u)
#define ERROR_TYPE_START_NEW_HANDSHAKE   (0x02u)
#define ERROR_TYPE_UPDATE_UR_TRANSMITTER (0x03u)
/*********************************************************************************/
/*Private functions*/
/*********************************************************************************/
/*State machines*/
static void PairingInit(void);
static void PairingStateMachine(void);
static void ErrorHandlingStateMachine(void);
static void HandShakeInit(void);
static void HandShakingStateMachine(void);
static void CommunicationInit(void);
static void CommStateMachine(void);
static void DisconnectInit(void);
static void DisconnectStateMachine(void);
/*********************************************************************************/
/*Frames handlers*/
static void UpdateIdFrame(void);
static u8   CheckIdFrame(void);
static void UpdateValFrame(void);
static u8   CheckValFrame(void);
static void UpdateDataFrame(void);
static u8 CheckDataFrame(void);
static void UpdateErrorFrame(u8 ErrorType);
static void CheckErrorFrame(void);
/*********************************************************************************/
/*Utilities*/
static void RxBufferDnCallBackNotif(void);
static void MemCpy( u8* DesPtr, const u8* SrcPtr, u16 Length);
static void MemSet(u8* DesPtr, u8 ConstVal, u16 Length);
#if (COMM_CINFIG == SLAVE_COMM)
static u8 MemCmp(const u8* Src1Ptr,const u8* Src2Ptr,u16 Length);
static u8 GetCrcKey(void);
#endif
static u8 CalculateCheksum(u8* BufferPtr, u16 BufferLength);
static void BuzzerSound(void);
static u8 CheckCrc(void);
static void PowerBluetoothOn(void);
/*static void PowerBluetoothOff(void);*/
static void BuzzerInit(void);
static void PowerBlueToothInit(void);
static void BlueToothKeyInit(void);
/*static void InserBreakPoint(void);*/
/*********************************************************************************/
/*Static variables*/
/*********************************************************************************/
static u8  BLMGR_PairingState;
static u32 BLMGR_PairingTimeoutCounter;
static u8  BLMGR_HandShakeState;
static u8  BLMGR_HandShakeTimeOutCounter;
static u8  BLMGR_DataRxBuffer[MAX_DATA_BUFFER_LENGTH];
static u8  BLMGR_DataTxBuffer[MAX_DATA_BUFFER_LENGTH];
static u8  BLMGR_FrameReceivedNotificationFlag;
static u8  BLMGR_ErrorState;
static u8  BLMGR_RxOsType;
static u8  BLMGR_RxDeviceType;
static u8  BLMGR_RxDevicName[MAX_DEV_NAME_LENGTH];
static u8  BLMGR_TxDevicName[MAX_DEV_NAME_LENGTH];
static u8  BLMGR_RxDeviceNameLength;
static u8  BLMGR_TxDeviceNameLength;
static u8  BLMGR_TxFrameReceiver;
/*static u8  BLMGR_RxFrameSender;*/
static u32 BLMGR_CrcKey;
static u8  BLMGR_CommState;
static u8  BLMGR_CommTimeOutCounter;
static u8  BLMGR_TxBattLevel;
#if (COMM_CINFIG == SLAVE_COMM)
static u8  BLMGR_TxDirection;
static u8  BLMGR_TxSpeedDegree;
static u8  BLMGR_RxBattLevel;
#endif
static u8  BLMGR_BluetoothState;
static u8  BLMGR_BluetoothStartRequest;
static u8  BLMGR_StopDeviceRequest;
static u8  BLMGR_PairingFailRepetionCount;
static u8  BLMGR_HandshakeFailRepCount;
static u8  BLMGR_CommFailReptCount;
static u8  BLMGR_ExpectedReceivedFrame;
static u8  BLMGR_DisconectionTimeOut;
/*static u8 testflag = 0u;*/
static u8 BLMGR_DevicePaired;
/*********************************************************************************/
/*Global Services*/
/*********************************************************************************/
void BLMGR_StartDevice(void)
{
    BLMGR_BluetoothStartRequest = (u8)1;
}
/*********************************************************************************/
void BLMGR_Test(void)
{
    BuzzerSound();



}

/*********************************************************************************/
void BLMGR_BluetoothInit(void)
{
    /*Init UART*/
    UART_Init();
    /*Init State*/
    BLMGR_BluetoothState = (u8) BLUETOOTH_STATE_STOPPED;
    BLMGR_BluetoothStartRequest = (u8) 0;
    BLMGR_StopDeviceRequest = (u8) 0;
    BLMGR_DevicePaired = (u8) 0;
    /*Init Pairing*/
    PairingInit();
    /*Init Handshaking*/
    HandShakeInit();
    /*Init Communication*/
    CommunicationInit();
    /*Disconnection Init*/
    DisconnectInit();
    /*Init Buzzer*/
    BuzzerInit();
    /*Init Bluetooth Power Control*/
    PowerBlueToothInit();
    /*Init Key*/
    BlueToothKeyInit();

}
/*********************************************************************************/
void BLMGR_BluetoothStateMachine(void)
{
    switch(BLMGR_BluetoothState)
    {
        case ((u8)BLUETOOTH_STATE_STOPPED):
        {
            /*Check if application need to start bluetooth*/
            if(BLMGR_BluetoothStartRequest == ((u8)1))
            {
                /*Power On the module*/
                PowerBluetoothOn();
                PairingInit();
                BLMGR_BluetoothState = ((u8)BLUETOOTH_STATE_PAIRING);

            }
        }
        break;

        case ((u8)BLUETOOTH_STATE_PAIRING):
        {
            PairingStateMachine();
            if(BLMGR_PairingState == ((u8)PAIRING_STATE_CONNECTED_DONE))
            {
                /*BLMGR_Test();*/
                /*Pairing succeeded, start handshaking*/
                HandShakeInit();
                BLMGR_BluetoothState = ((u8)BLUETOOTH_STATE_HANDSHAKING);
            }
            else if(BLMGR_PairingState == ((u8)PAIRING_STATE_FAILED))
            {
                /*Pairing failed, disconnect the module*/
                PairingInit();
                BLMGR_BluetoothState = ((u8)BLUETOOTH_STATE_DISCONNECTED);
            }
            else
            {
                /*pairing is in progress*/
            }
        }
        break;
        case ((u8)BLUETOOTH_STATE_HANDSHAKING):
        {
            HandShakingStateMachine();
            if(BLMGR_HandShakeState == ((u8)HANDSHAKE_STATE_HANDSHAKING_DONE))
            {
                /*Handshake succeeded, start communication*/
                CommunicationInit();
                BLMGR_BluetoothState = ((u8)BLUETOOTH_STATE_COMMUNICATION);
                BuzzerSound();
            }
            else if (BLMGR_HandShakeState == ((u8)HANDSHAKE_STATE_FAILED))
            {
                /*handshake failed, disconnect the module*/
                HandShakeInit();
                BLMGR_BluetoothState = ((u8)BLUETOOTH_STATE_DISCONNECTED);
            }
            else
            {
                /*handshake still in progress*/
            }
        }
        break;
        case ((u8)BLUETOOTH_STATE_COMMUNICATION):
        {
            CommStateMachine();
            if(BLMGR_CommState == ((u8)COMM_STATE_FAILED))
            {
                /*Disconnect the module*/
                CommunicationInit();
                BLMGR_BluetoothState = ((u8)BLUETOOTH_STATE_DISCONNECTED);
            }
            else
            {
                /*Communication is in progress*/

            }
        }
        break;

        case ((u8)BLUETOOTH_STATE_DISCONNECTED):
        {
            DisconnectStateMachine();
            /*Check if application need to start bluetooth*/
            if(BLMGR_BluetoothStartRequest == ((u8)1))
            {
                /*Power On the module*/
                /*PowerBluetoothOn();*/
                PairingInit();
                /*PowerBluetoothOff();*/
                /*InserBreakPoint();*/
                BLMGR_PairingState = ((u8)PAIRING_STATE_WAIT_PAIR_REQ);
                if(BLMGR_DisconectionTimeOut > ((u8)MAX_DISCONNECTION_COUNT))
                {
                    BLMGR_BluetoothState = ((u8)BLUETOOTH_STATE_PAIRING);
                    BLMGR_DisconectionTimeOut = ((u8)0);

                }
                else
                {
                    /*BuzzerSound();*/
                    BLMGR_DisconectionTimeOut ++;
                }

            }
            else if (BLMGR_StopDeviceRequest == ((u8)1))
            {
                /*PowerBluetoothOff();*/
                DisconnectInit();
                BLMGR_BluetoothState = ((u8)BLUETOOTH_STATE_STOPPED);
            }
            else
            {
                /*Still disconnected*/
            }
        }
        break;

        default:
        break;
    }
}
/*********************************************************************************/
/*Private Funcions definitions*/
/*********************************************************************************/
static void PairingInit(void)
{
    BLMGR_PairingState = ((u8)PAIRING_STATE_IDLE);
    BLMGR_PairingTimeoutCounter = ((u8)0);
    BLMGR_PairingFailRepetionCount = ((u8)0);
    /*BLMGR_DevicePaired = 0;*/
}
/*********************************************************************************/
static void HandShakeInit(void)
{
    BLMGR_HandShakeState = ((u8)HANDSHAKE_STATE_IDLE);
    BLMGR_PairingTimeoutCounter = ((u8)0);
    BLMGR_FrameReceivedNotificationFlag = ((u8)0);
    BLMGR_HandshakeFailRepCount = ((u8)0);
}
/*********************************************************************************/
static void CommunicationInit(void)
{
    BLMGR_CommState = ((u8)COMM_STATE_IDLE);
    BLMGR_CommTimeOutCounter = ((u8)0);
    BLMGR_CommFailReptCount = ((u8)0);
}
/*********************************************************************************/

static void PairingStateMachine(void)
{
    u8 ResponseState;

    if(BLMGR_DevicePaired == ((u8)1))
    {
        /*InserBreakPoint();*/
        BLMGR_PairingState = (u8)PAIRING_STATE_START_WAIT_PAIR_REQ;
        BLMGR_DevicePaired = ((u8)0);
    }
    switch(BLMGR_PairingState)
    {
        case (u8)PAIRING_STATE_IDLE:
        {
            /*wait for 1 second for stabilization*/
            if(BLMGR_PairingTimeoutCounter > (u8)PAIRING_MAX_COUNTS)
            {
                BLMGR_PairingTimeoutCounter = ((u8)0);
                /*go to the init state*/
                BLMGR_PairingFailRepetionCount = ((u8)0);
                BLMGR_PairingState = (u8)PAIRING_STATE_INITIALIZING;


            }
            else
            {
                BLMGR_PairingTimeoutCounter ++;
            }
        }
        break;

        case (u8)PAIRING_STATE_INITIALIZING:
        {
            /*send init Command*/
            BLTD_SendInitCmd();
            /*Go to next state to read response*/
            BLMGR_PairingState = (u8)PAIRING_STATE_WAIT_INIT_RESP;
        }
        break;

        case (u8)PAIRING_STATE_WAIT_INIT_RESP:
        {
            u8 RespArray[4];
            RespArray[0] = (u8)'O';
            RespArray[1] = (u8)'K';
            RespArray[2] = (u8)0x0d;
            RespArray[3] = (u8)0x0a;

            ResponseState = BLTD_CheckForResponse(RespArray,(u16)4);
            switch(ResponseState)
            {
                case (u8)BLTD_RESP_STATUS_OK:
                BLMGR_PairingFailRepetionCount = ((u8)0);
                BLMGR_PairingTimeoutCounter = ((u8)0);
                /*Respnse recieved and go to send the inquire request*/
                BLMGR_PairingState = (u8)PAIRING_STATE_INQUIRE;

                break;

                case (u8)BLTD_RESP_STATUS_NOK:

                if(BLMGR_PairingFailRepetionCount <= (u8)MAX_PAIRING_FAIL_REPT)
                {
                    /*response received not ok so re send the command again*/
                    BLMGR_PairingState = (u8)PAIRING_STATE_INITIALIZING;
                    BLMGR_PairingFailRepetionCount ++;

                }
                else
                {
                    BLMGR_PairingFailRepetionCount = ((u8)0);
                    BLMGR_PairingState = (u8)PAIRING_STATE_INQUIRE;
                }

                break;

                case (u8)BLTD_RESP_STATUS_NON:

                /*response not received and wait until timeout*/
                BLMGR_PairingTimeoutCounter ++;
                if(BLMGR_PairingTimeoutCounter > (u8)PAIRING_MAX_COUNTS)
                {
                    if(BLMGR_PairingFailRepetionCount <= (u8)MAX_PAIRING_FAIL_REPT)
                    {
                        BLMGR_PairingFailRepetionCount ++;
                        BLMGR_PairingTimeoutCounter = ((u8)0);
                        BLMGR_PairingState = (u8)PAIRING_STATE_INITIALIZING;
                    }
                    else
                    {
                        BLMGR_PairingFailRepetionCount = ((u8)0);
                        BLMGR_PairingState = (u8)PAIRING_STATE_FAILED;
                    }
                }
                else
                {
                    BLMGR_PairingTimeoutCounter ++;
                    BLMGR_PairingState = (u8)PAIRING_STATE_WAIT_INIT_RESP;
                }
                break;

                default:
                    break;
            }
        }
        break;
        case (u8)PAIRING_STATE_INQUIRE:
        {

            /*Send Inquire Command*/
            BLTD_SendInquireCmd();
            /*wait for the Inquire Response*/
            BLMGR_PairingState = (u8)PAIRING_STATE_WAIT_INQUIRE_RESP;
        }
        break;
        case (u8)PAIRING_STATE_WAIT_INQUIRE_RESP:
        {
            u8 RespArray1[4];
            RespArray1[0] = (u8)'O';
            RespArray1[1] = (u8)'K';
            RespArray1[2] = (u8)0x0d;
            RespArray1[3] = (u8)0x0a;
            ResponseState = BLTD_CheckForResponse(RespArray1,(u16)4);
            switch(ResponseState)
            {
                case (u8)BLTD_RESP_STATUS_OK:
                BLMGR_PairingFailRepetionCount = ((u8)0);
                BLMGR_PairingTimeoutCounter = ((u8)0);
                /*Respnse recieved and go to send the inquire request*/
                BLMGR_PairingState = (u8)PAIRING_STATE_START_WAIT_PAIR_REQ;

                break;

                case (u8)BLTD_RESP_STATUS_NOK:
                if(BLMGR_PairingFailRepetionCount <= (u8)MAX_PAIRING_FAIL_REPT)
                {
                    BLMGR_PairingFailRepetionCount ++;
                    /*response received not ok so re send the command again*/
                    BLMGR_PairingState = (u8)PAIRING_STATE_WAIT_INQUIRE_RESP;
                }
                else
                {
                    BLMGR_PairingFailRepetionCount = ((u8)0);
                    BLMGR_PairingState = (u8)PAIRING_STATE_INITIALIZING;
                }
                break;

                case (u8)BLTD_RESP_STATUS_NON:
                /*response not received and wait until timeout*/
                BLMGR_PairingTimeoutCounter ++;
                if(BLMGR_PairingTimeoutCounter > (u32)PAIRING_MAX_COUNTS)
                {
                    if(BLMGR_PairingFailRepetionCount <= (u8)MAX_PAIRING_FAIL_REPT)
                    {
                        BLMGR_PairingFailRepetionCount ++;
                        BLMGR_PairingTimeoutCounter = ((u32)0);
                        BLMGR_PairingState = (u8)PAIRING_STATE_INQUIRE;
                    }
                    else
                    {
                        BLMGR_PairingFailRepetionCount = ((u8)0);
                        BLMGR_PairingState = (u8)PAIRING_STATE_FAILED;
                    }
                }
                else
                {
                    BLMGR_PairingTimeoutCounter ++;
                    BLMGR_PairingState = (u8)PAIRING_STATE_WAIT_INQUIRE_RESP;
                }
                break;

                default:
                    break;
            }
        }
        break;
        case (u8)PAIRING_STATE_START_WAIT_PAIR_REQ:
        BLTD_StartWaitPairing();
        BuzzerSound();
        BLMGR_PairingState = (u8)PAIRING_STATE_WAIT_PAIR_REQ;
        break;
        case (u8)PAIRING_STATE_WAIT_PAIR_REQ:
        {
            u8 RespArray2[4];
            RespArray2[0] = (u8)'O';
            RespArray2[1] = (u8)'K';
            RespArray2[2] = (u8)0x0d;
            RespArray2[3] = (u8)0x0a;
            ResponseState = BLTD_CheckForResponse(RespArray2,(u16)4);
            switch(ResponseState)
            {
                case (u8)BLTD_RESP_STATUS_OK:
                BLMGR_PairingFailRepetionCount = ((u8)0);
                BLMGR_PairingTimeoutCounter = ((u8)0);
                /*Respnse recieved and go to send the inquire request*/
                BLMGR_PairingState = (u8)PAIRING_STATE_CONNECTED_DONE;
                /*BuzzerSound();*/
                BLMGR_DevicePaired = ((u8)1);
                break;

                case (u8)BLTD_RESP_STATUS_NOK:
                if(BLMGR_PairingFailRepetionCount <= (u8)MAX_PAIRING_FAIL_REPT)
                {
                    BLMGR_PairingFailRepetionCount ++;
                    /*response received not ok so re send the command again*/
                    BLMGR_PairingState = (u8)PAIRING_STATE_INQUIRE;
                }
                else
                {
                    BLMGR_PairingFailRepetionCount = ((u8)0);
                    BLMGR_PairingState = (u8)PAIRING_STATE_FAILED;
                }
                break;

                case (u8)BLTD_RESP_STATUS_NON:
                /*response not received and wait until timeout*/
                BLMGR_PairingState = (u8)PAIRING_STATE_WAIT_PAIR_REQ;
                /*BuzzerSound();*/
                break;

                default:
                    break;
            }
        }
        break;
        default:
        break;
    }
}
/*********************************************************************************/
static void HandShakingStateMachine(void)
{
    u8 IsFrameValid;
    switch (BLMGR_HandShakeState)
    {
        case (u8)HANDSHAKE_STATE_IDLE:
        {
            /*Check for the Device Comm Mode*/
            #if(COMM_CINFIG == MSTER_COMM)
            /* the device will master the communication*/
            BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_SEND_ID_FRAME;
            #elif(COMM_CINFIG == SLAVE_COMM)
            /*the Device will be mastered by the other device*/
            BLTD_StartReceivingData(BLMGR_DataRxBuffer,ID_FRAME_LENGTH,RxBufferDnCallBackNotif);
            BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_RECV_ID_FRAME;
            #else
            /*Wrong Config, State in Idle*/
            /*To do: Managing dev Errors*/
            #endif
        }
        break;
        case (u8)HANDSHAKE_STATE_SEND_ID_FRAME:
        {
            /*Update the ID frame by  loading tx data buffer*/
            UpdateIdFrame();
            #if(COMM_CINFIG == MSTER_COMM)
            /*Start Receiving the slave response*/
            BLTD_StartReceivingData(BLMGR_DataRxBuffer,(u8)ID_FRAME_LENGTH,&RxBufferDnCallBackNotif);
            BLMGR_ExpectedReceivedFrame = (u8)FRAME_TYPE_ID_FRAME;
            BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_RECV_ID_FRAME;
            #elif(COMM_CINFIG == SLAVE_COMM)
            /*Start receiving Validation frame*/
            BLTD_StartReceivingData(BLMGR_DataRxBuffer,VAL_FRAME_LENGTH,RxBufferDnCallBackNotif);
            BLMGR_ExpectedReceivedFrame = FRAME_TYPE_VAL_FRAME;
            BLMGR_HandShakeState = HANDSHAKE_STATE_RECV_VAL_FRAME;
            #else
            /*Wrong Config, State in Idle*/
            /*To do: Managing dev Errors*/
            #endif
            /*Send the ID Frame*/
            BLTD_SendMessage(BLMGR_DataTxBuffer,(u16)ID_FRAME_LENGTH);
        }
        break;
        case (u8)HANDSHAKE_STATE_RECV_ID_FRAME:
        {
            /*Check that a frame was received*/
            if(BLMGR_FrameReceivedNotificationFlag == (u8)1)
            {
                BLMGR_FrameReceivedNotificationFlag = (u8)0;
                BLMGR_HandShakeTimeOutCounter = (u8)0;
                IsFrameValid = CheckIdFrame();
                if(IsFrameValid == (u8)1)
                {
                    BLMGR_HandshakeFailRepCount = (u8)0;
                    /*Frame is valid*/
                    #if(COMM_CINFIG == MSTER_COMM)
                    /*Send the Validation frame*/
                    BLTD_StartReceivingData(BLMGR_DataRxBuffer,(u8)VAL_FRAME_LENGTH,&RxBufferDnCallBackNotif);
                    BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_HANDSHAKING_DONE;
                    #elif(COMM_CINFIG == SLAVE_COMM)
                    /*Start receiving validation frame*/
                    BLTD_StartReceivingData(BLMGR_DataRxBuffer,VAL_FRAME_LENGTH,RxBufferDnCallBackNotif);
                    BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_RECV_VAL_FRAME;
                    #else
                    /*Wrong Config, State in Idle*/
                    /*To do: Managing dev Errors*/
                    #endif
                }
                else
                {
                    /*Frame is invalid*/
                    BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_SEND_ERR_FRAME;
                }
            }
            else
            {
                /*No frame Received, Check Timeout*/
                if(BLMGR_HandShakeTimeOutCounter > (u8)HANDSHAKING_MAX_COUNTS)
                {
                    /*Handle Timeout Error*/
                    BLMGR_HandShakeTimeOutCounter = (u8)0;
                    BLMGR_ErrorState = (u8)ERRH_TIMEOUT_ERROR;
                    BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_SEND_ERR_FRAME;
                }
                else
                {
                    BLMGR_HandShakeTimeOutCounter ++;
                }
            }
        }
        break;
        case (u8)HANDSHAKE_STATE_SEND_VAL_FRMAE:
        {
            /*Prepare Validation frame*/
            UpdateValFrame();
            /*Sending Validation frame*/
            BLTD_StartReceivingData(BLMGR_DataRxBuffer,(u16)VAL_FRAME_LENGTH,&RxBufferDnCallBackNotif);
            BLTD_SendMessage(BLMGR_DataTxBuffer,(u16)VAL_FRAME_LENGTH);
            #if(COMM_CINFIG == MSTER_COMM)
            BLMGR_ExpectedReceivedFrame = (u8)FRAME_TYPE_VAL_FRAME;
            BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_RECV_VAL_FRAME;
            #elif(COMM_CINFIG == SLAVE_COMM)
            BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_HANDSHAKING_DONE;
            #else
            /*Wrong Config, State in Idle*/
            /*To do: Managing dev Errors*/
            #endif
        }
        break;
        case (u8)HANDSHAKE_STATE_RECV_VAL_FRAME:
        {
            /*Check that a frame was received*/
            if(BLMGR_FrameReceivedNotificationFlag == (u8)1)
            {
                BLMGR_FrameReceivedNotificationFlag = (u8)0;
                BLMGR_HandShakeTimeOutCounter = (u8)0;
                IsFrameValid = CheckValFrame();
                if(IsFrameValid == (u8)1)
                {
                    BLMGR_HandshakeFailRepCount = (u8)0;
                    #if(COMM_CINFIG == MSTER_COMM)
                    /*Master the Communication phase*/
                    BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_HANDSHAKING_DONE;
                    #elif(COMM_CINFIG == SLAVE_COMM)
                    BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_SEND_VAL_FRMAE;
                    /*Start Receiving validation frame*/
                    BLTD_StartReceivingData(BLMGR_DataRxBuffer,VAL_FRAME_LENGTH,RxBufferDnCallBackNotif);
                    #else
                    /*Wrong Config, State in Idle*/
                    /*To do: Managing dev Errors*/
                    #endif
                }
                else
                {
                    /*Handle validation error*/
                    BLMGR_ErrorState = (u8)ERRH_HANDSHAKE_ERROR;
                    BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_SEND_ERR_FRAME;
                }
            }
            else
            {
                /*No frame Received, Check Timeout*/
                if(BLMGR_HandShakeTimeOutCounter > (u8)HANDSHAKING_MAX_COUNTS)
                {
                    /*Handle Timeout Error*/
                    BLMGR_HandShakeTimeOutCounter = (u8)0;
                    BLMGR_ErrorState = (u8)ERRH_TIMEOUT_ERROR;
                    BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_SEND_ERR_FRAME;
                }
                else
                {
                    BLMGR_HandShakeTimeOutCounter ++;
                }
            }
        }
        break;
        case (u8)HANDSHAKE_STATE_SEND_ERR_FRAME:
        {
            ErrorHandlingStateMachine();
        }
        break;
        default:
        break;
    }
}
/*********************************************************************************/
static void CommStateMachine(void)
{
    u8 IsFrameValid3;
    switch (BLMGR_CommState)
    {
        case (u8)COMM_STATE_IDLE:
        {
            #if(COMM_CINFIG == MSTER_COMM)
            /*Start Sending Data frame*/
            BLMGR_CommState = (u8)COMM_STATE_SEND_DATA_FRAME;
            #elif(COMM_CINFIG == SLAVE_COMM)
            /*Start Receiving data frame*/
            BLMGR_ExpectedReceivedFrame = FRAME_TYPE_DATA_FRAME;
            BLMGR_CommState = COMM_STATE_RECV_DATA_FRAME;
            BLTD_StartReceivingData(BLMGR_DataRxBuffer,DATA_FRAME_LENGTH,RxBufferDnCallBackNotif);
            #else
            /*Wrong Config, State in Idle*/
            /*To do: Managing dev Errors*/
            #endif
        }
        break;
        case (u8)COMM_STATE_SEND_DATA_FRAME:
        {
            /*Update Data Frame*/
            UpdateDataFrame();
            /*Start Receiving data frame*/
            BLTD_StartReceivingData(BLMGR_DataRxBuffer,(u16)DATA_FRAME_LENGTH,&RxBufferDnCallBackNotif);
            BLMGR_ExpectedReceivedFrame = (u8)FRAME_TYPE_DATA_FRAME;
            BLMGR_CommState = (u8)COMM_STATE_RECV_DATA_FRAME;
            /*Send the Data Frame*/
            BLTD_SendMessage(BLMGR_DataTxBuffer,(u16)DATA_FRAME_LENGTH);
        }
        break;
        case (u8)COMM_STATE_RECV_DATA_FRAME:
        {
            /*Check that a frame was received*/
            if(BLMGR_FrameReceivedNotificationFlag == (u8)1)
            {
                BLMGR_FrameReceivedNotificationFlag = (u8)0;
                BLMGR_HandShakeTimeOutCounter = (u8)0;
                BLMGR_CommTimeOutCounter = (u8)0;
                /*Check Received data frame*/
                IsFrameValid3 = CheckDataFrame();
                if(IsFrameValid3 == (u8)1)
                {
                    BLMGR_CommFailReptCount = (u8)0;
                    BLMGR_CommState = (u8)COMM_STATE_SEND_DATA_FRAME;
                }
                else
                {
                    BuzzerSound();
                    BLMGR_CommFailReptCount = (u8)0;
                    /*Handshaking failed*/
                    BLMGR_CommState = (u8)COMM_STATE_FAILED;
                }
            }
            else
            {

                if(BLMGR_CommTimeOutCounter > (u8)COMM_MAX_COUNTS)
                {

                    /*InserBreakPoint();*/
                    /*Handle Timeout Error*/
                    BLMGR_CommTimeOutCounter = (u8)0;
                    BLMGR_ErrorState = (u8)ERRH_TIMEOUT_ERROR;
                    BLMGR_CommState = (u8)COMM_STATE_SEND_ERROR_FRAME;
                }
                else
                {
                    BLMGR_CommTimeOutCounter ++;
                }
            }
        }
        break;
        case (u8)COMM_STATE_SEND_ERROR_FRAME:
        {

            ErrorHandlingStateMachine();
        }
        break;
        default:
        break;
    }

}

/*********************************************************************************/
static void BuzzerSound(void)
{
    u8 LoopIndex;
    for(LoopIndex = (u8)0; LoopIndex < (u8)2 ; LoopIndex ++)
    {
        DIO_WritePort(BuzzerConfig.Portname,BUZEER_ON,BuzzerConfig.Port_Mask);
        _delay_ms((u8)25);
        DIO_WritePort(BuzzerConfig.Portname,BUZEER_OFF,BuzzerConfig.Port_Mask);
        _delay_ms((u8)25);

    }

}
/*********************************************************************************/
static void RxBufferDnCallBackNotif(void)
{

    BLMGR_FrameReceivedNotificationFlag = (u8)1;

}
/*********************************************************************************/
void BLMGR_SetReceiver(u8 Receiver)
{
    BLMGR_TxFrameReceiver = Receiver;
}
/*********************************************************************************/
void BLMGR_SetDeviceName(u8 * DeviceName,u8 DeviceNameLength)
{
    u8 * DeviceName_ptr = DeviceName;
    MemCpy(BLMGR_TxDevicName,DeviceName_ptr,(u16)DeviceNameLength);
    BLMGR_TxDeviceNameLength = DeviceNameLength;
}
/*********************************************************************************/

static void UpdateIdFrame(void)
{
    /*Set Tx Frame to default values*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_HEADER_IDX],(u8)TX_FRAME_DEFUALT,(u16)MAX_DATA_BUFFER_LENGTH);
    /*Set Header of Frame*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_HEADER_IDX],(u8)0xaa,(u16)2);
    /*Set Device Sender*/
    BLMGR_DataTxBuffer[FRAME_SENDER_IDX] = (u8)DEVICE_ROLE;
    /*Set Device Receiver*/
    BLMGR_DataTxBuffer[FRAME_RECVER_IDX] = BLMGR_TxFrameReceiver;
    /*Set frame type*/
    BLMGR_DataTxBuffer[FRAME_TYPE_IDX] = (u8)FRAME_TYPE_ID_FRAME;
    /*Set paramter length*/
    BLMGR_DataTxBuffer[PARAM_LENGTH_IDX] = (u8)2 + BLMGR_TxDeviceNameLength;
    /*Update Os Type*/
    BLMGR_DataTxBuffer[OS_TYPE_IDX] = (u8)TX_OS_TYPE;
    /*Update Device Type*/
    BLMGR_DataTxBuffer[DEV_TYPE_IDX] = (u8)TX_DEV_TYPE;
    /*Update Device Name*/
    MemCpy(&BLMGR_DataTxBuffer[DEV_NAME_IDX],BLMGR_TxDevicName,(u16)BLMGR_TxDeviceNameLength);
    /*update Default CRC*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_CRC_IDX],(u8)TX_CRC_DEFAULT,(u16)2);
    /*update Frame CheckSum*/
    BLMGR_DataTxBuffer[FRAME_CHECKSUM_IDX] = CalculateCheksum(BLMGR_DataTxBuffer,(u16)(FRAME_CHECKSUM_IDX - 1u));
    /*update frame tail*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_TAIL_IDX],(u8)0x55,(u16)1);
}
/*********************************************************************************/
static u8 CheckIdFrame(void)
{
    u8 IsFrameValid4;
    u8 TempVar;
    /* Perform a Checksum on the frame*/
    TempVar = CalculateCheksum(BLMGR_DataRxBuffer,(u16)(FRAME_CHECKSUM_IDX -1u));

    if (TempVar == BLMGR_DataRxBuffer[FRAME_CHECKSUM_IDX])
    {

        /*Perform Start and end of frame validation*/
        if((BLMGR_DataRxBuffer[FRAME_HEADER_IDX] == (u8)0xaa) &&
        (BLMGR_DataRxBuffer[FRAME_HEADER_IDX + 1u] == (u8)0xaa) &&
        (BLMGR_DataRxBuffer[FRAME_TAIL_IDX] == (u8)0x55))
        {

            /*Validate Frame Type*/
            if(BLMGR_DataRxBuffer[FRAME_TYPE_IDX] == (u8)FRAME_TYPE_ID_FRAME)
            {

                /* Check CRC*/
                if((BLMGR_DataRxBuffer[FRAME_CRC_IDX] == (u8)TX_CRC_DEFAULT) &&
                (BLMGR_DataRxBuffer[FRAME_CRC_IDX] == (u8)TX_CRC_DEFAULT))
                {
                    /*Validate Frame Receiver*/
                    if(BLMGR_DataRxBuffer[FRAME_RECVER_IDX] == (u8)DEVICE_ROLE)
                    {
                        /*Validate Device Name*/
                        BLMGR_RxDeviceNameLength = (u8)8 - BLMGR_DataRxBuffer[PARAM_LENGTH_IDX];
                        if(BLMGR_RxDeviceNameLength <= (u8)MAX_DEV_NAME_LENGTH)
                        {

                            /*Update received paramters*/
                            /*Update Frame sender*/
                            /* BLMGR_RxFrameSender = BLMGR_DataRxBuffer[FRAME_SENDER_IDX];*/
                            /*Update OS Type*/
                            BLMGR_RxOsType = BLMGR_DataRxBuffer[OS_TYPE_IDX];
                            /*Update Device Type*/
                            BLMGR_RxDeviceType = BLMGR_DataRxBuffer[DEV_TYPE_IDX];
                            /*Update Device Name*/
                            MemCpy(BLMGR_RxDevicName,&BLMGR_DataRxBuffer[DEV_NAME_IDX],(u16)BLMGR_RxDeviceNameLength);
                            BLMGR_ErrorState = (u8)ERRH_NO_ERROR;
                            IsFrameValid4 = (u8)1;
                        }
                        else
                        {
                            /*Invalid Frame receiver*/
                            BLMGR_ErrorState = (u8)ERRH_INVALID_FRAME_ERROR;
                            IsFrameValid4 = (u8)0;
                        }
                    }
                    else
                    {
                        /*Invalid Frame receiver*/
                        BLMGR_ErrorState = (u8)ERRH_INVALID_RECEIVER_ERROR;
                        IsFrameValid4 = (u8)0;
                    }
                }
                else
                {
                    /*Crc Error Found*/
                    BLMGR_ErrorState = (u8)ERRH_CRC_ERROR;
                    IsFrameValid4 = (u8)0;
                }
            }
            else
            {
                /*Invalid Frame Type*/
                BLMGR_ErrorState = (u8)ERRH_WRONG_FRAME_ERROR;
                IsFrameValid4 = (u8)0;
            }
        }
        else
        {
            /*Invalid Frame Detected*/
            BLMGR_ErrorState = (u8)ERRH_INVALID_FRAME_ERROR;
            IsFrameValid4 = (u8)0;
        }
    }
    else
    {
        /*Checksum error*/
        BLMGR_ErrorState = (u8)ERRH_CHECKSUM_ERROR;
        IsFrameValid4 = (u8)0;
    }
    return IsFrameValid4;
}
/*********************************************************************************/
static void UpdateValFrame(void)
{
    u16 Crc = 0;
    u32 CrcKey = (u32)0;
    static u8 TempBuffer[MAX_DATA_BUFFER_LENGTH];
    /*Set Tx Frame to default values*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_HEADER_IDX],(u8)TX_FRAME_DEFUALT,(u16)MAX_DATA_BUFFER_LENGTH);
    /*Set Header of Frame*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_HEADER_IDX],(u8)0xaa,(u16)2);
    /*Set Device Sender*/
    BLMGR_DataTxBuffer[FRAME_SENDER_IDX] = (u8)DEVICE_ROLE;
    /*Set Device Receiver*/
    BLMGR_DataTxBuffer[FRAME_RECVER_IDX] = (u8)BLMGR_TxFrameReceiver;
    /*Set frame type*/
    BLMGR_DataTxBuffer[FRAME_TYPE_IDX] = (u8)FRAME_TYPE_VAL_FRAME;
    /*Set paramter length*/
    BLMGR_DataTxBuffer[PARAM_LENGTH_IDX] = (u8)2;
    #if(COMM_CINFIG == MSTER_COMM)
    /* Start Generating the Key for CRC*/
    SECR_CrcPolynomialGenerate(&CrcKey,(u8)16);
    BLMGR_CrcKey = CrcKey;
    #endif
    /*Calculate CRC*/
    /*Prepare Data*/
    TempBuffer[0x00] = BLMGR_RxOsType;
    TempBuffer[0x01] = BLMGR_RxDeviceType;
    MemCpy(&TempBuffer[0x02],BLMGR_RxDevicName,(u16)BLMGR_RxDeviceNameLength);
    SECR_GnerateCrc(TempBuffer,BLMGR_RxDeviceNameLength + (u16)2, &Crc,BLMGR_CrcKey);
    /*Update Crc*/
    BLMGR_DataTxBuffer[FRAME_VAL_CRC_IDX] = (u8)Crc;
    BLMGR_DataTxBuffer[FRAME_VAL_CRC_IDX + 1u] = (u8)(Crc >> 8);
    /*update Default CRC*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_CRC_IDX],(u8)TX_CRC_DEFAULT,(u16)2);
    /*update Frame CheckSum*/
    BLMGR_DataTxBuffer[FRAME_CHECKSUM_IDX] = CalculateCheksum(BLMGR_DataTxBuffer,(u16)(FRAME_CHECKSUM_IDX - 1u));
    /*update frame tail*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_TAIL_IDX],(u8)0x55,(u16)1);
}
/*********************************************************************************/
static u8 CheckValFrame(void)
{
    u8 IsFrameValid5;
    u8 TempVar2;
    /* Perform a Checksum on the frame*/
    TempVar2 = CalculateCheksum(BLMGR_DataRxBuffer,(u16)(FRAME_CHECKSUM_IDX -1u));
    if (TempVar2 == BLMGR_DataRxBuffer[FRAME_CHECKSUM_IDX])
    {
        /*Perform Start and end of frame validation*/
        if((BLMGR_DataRxBuffer[FRAME_HEADER_IDX] == (u8)0xaa) &&
        (BLMGR_DataRxBuffer[FRAME_HEADER_IDX + 1u] == (u8)0xaa) &&
        (BLMGR_DataRxBuffer[FRAME_TAIL_IDX] == (u8)0x55))
        {
            /*Validate Frame Type*/
            if(BLMGR_DataRxBuffer[FRAME_TYPE_IDX] == (u8)FRAME_TYPE_VAL_FRAME)
            {
                /* Check CRC*/
                if((BLMGR_DataRxBuffer[FRAME_CRC_IDX] == (u8)TX_CRC_DEFAULT) &&
                (BLMGR_DataRxBuffer[FRAME_CRC_IDX] == (u8)TX_CRC_DEFAULT))
                {
                    /*Validate Frame Receiver*/
                    if(BLMGR_DataRxBuffer[FRAME_RECVER_IDX] == (u8)DEVICE_ROLE)
                    {
                        #if(COMM_CINFIG == MSTER_COMM)
                        /*Validate CRC */
                        IsFrameValid5 = CheckCrc();


                        #elif(COMM_CINFIG == SLAVE_COMM)
                        /*Get the Crc Key*/
                        IsFrameValid5 = GetCrcKey();
                        #endif
                    }
                    else
                    {
                        /*Invalid Frame receiver*/
                        BLMGR_ErrorState = (u8)ERRH_INVALID_RECEIVER_ERROR;
                        IsFrameValid5 = (u8)0;
                    }

                }
                else
                {
                    /*Crc Error Found*/
                    BLMGR_ErrorState = (u8)ERRH_CRC_ERROR;
                    IsFrameValid5 = (u8)0;
                }
            }
            else
            {
                /*Invalid Frame Type*/
                BLMGR_ErrorState = (u8)ERRH_WRONG_FRAME_ERROR;
                IsFrameValid5 = (u8)0;
            }
        }
        else
        {
            /*Invalid Frame Detected*/
            BLMGR_ErrorState = (u8)ERRH_INVALID_FRAME_ERROR;
            IsFrameValid5 = (u8)0;
        }
    }
    else
    {
        /*Checksum error*/
        BLMGR_ErrorState = (u8)ERRH_CHECKSUM_ERROR;
        IsFrameValid5 = (u8)0;
    }
    return IsFrameValid5;
}
/*********************************************************************************/
void BLMGR_SetBattLevel(u8 BattLevel)
{
    BLMGR_TxBattLevel= BattLevel;
}

static void UpdateDataFrame(void)
{
    static u8 TempBuffer4[MAX_DATA_BUFFER_LENGTH];
    u16 Crc1 = 0;
    /*Set Tx Frame to default values*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_HEADER_IDX],(u8)TX_FRAME_DEFUALT,(u16)MAX_DATA_BUFFER_LENGTH);
    /*Set Header of Frame*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_HEADER_IDX],(u8)0xaa,(u16)2);
    /*Set Device Sender*/
    BLMGR_DataTxBuffer[FRAME_SENDER_IDX] = (u8)DEVICE_ROLE;
    /*Set Device Receiver*/
    BLMGR_DataTxBuffer[FRAME_RECVER_IDX] = BLMGR_TxFrameReceiver;
    /*Set frame type*/
    BLMGR_DataTxBuffer[FRAME_TYPE_IDX] = (u8)FRAME_TYPE_DATA_FRAME;
    #if(COMM_CINFIG == MSTER_COMM)
    /*Set paramter length*/
    BLMGR_DataTxBuffer[PARAM_LENGTH_IDX] = (u8)1;
    /*Set Batterly level*/
    BLMGR_DataTxBuffer[BATT_LEVEL_IDX] = BLMGR_TxBattLevel;
    /*Calculate CRC*/
    MemCpy(TempBuffer4,&BLMGR_DataTxBuffer[BATT_LEVEL_IDX],(u16)1);
    SECR_GnerateCrc(TempBuffer4,(u16)1, &Crc1,BLMGR_CrcKey);
    #elif(COMM_CINFIG == SLAVE_COMM)
    /*Set paramter length*/
    BLMGR_DataTxBuffer[PARAM_LENGTH_IDX] = 2u;
    /*Set Direction*/
    BLMGR_DataTxBuffer[DIRECTION_IDX]= BLMGR_TxDirection;
    /*Set Speed degree*/
    BLMGR_DataTxBuffer[SPEED_DEGREE_IDX]= BLMGR_TxSpeedDegree;
    /*Calculate CRC*/
    MemCpy(TempBuffer4,&BLMGR_DataTxBuffer[DIRECTION_IDX],2);
    SECR_GnerateCrc(TempBuffer,2u, &Crc,BLMGR_CrcKey);
    #else
    /*Wrong Config, State in Idle*/
    /*To do: Managing dev Errors*/
    #endif
    /*Update Crc*/
    BLMGR_DataTxBuffer[FRAME_CRC_IDX] = (u8)Crc1;
    BLMGR_DataTxBuffer[FRAME_CRC_IDX + 1u] = (u8)(Crc1 >> 8u);
    /*update Frame CheckSum*/
    BLMGR_DataTxBuffer[FRAME_CHECKSUM_IDX] = CalculateCheksum(BLMGR_DataTxBuffer,(u8)(FRAME_CHECKSUM_IDX -1u));
    /*update frame tail*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_TAIL_IDX],(u8)0x55,(u16)1);
}
/*********************************************************************************/
static u8 CheckDataFrame(void)
{
   /*
    static u8  BLMGR_RxDirection;
    static u8  BLMGR_RxSpeedDegree;
    */
    static u8 TempBuffer5[MAX_DATA_BUFFER_LENGTH];
    u8 IsFrameValid1;
    u8 TempVar1;
    u16 GenCrc = 0;
    u16 RecvdCrc;
    /* Perform a Checksum on the frame*/
    TempVar1 = CalculateCheksum(BLMGR_DataRxBuffer,(u8)(FRAME_CHECKSUM_IDX -1u));
    if (TempVar1 == BLMGR_DataRxBuffer[FRAME_CHECKSUM_IDX])
    {
        /*Perform Start and end of frame validation*/
        if((BLMGR_DataRxBuffer[FRAME_HEADER_IDX] == (u8)0xaa) &&
        (BLMGR_DataRxBuffer[FRAME_HEADER_IDX + 1u] == (u8)0xaa) &&
        (BLMGR_DataRxBuffer[FRAME_TAIL_IDX] == (u8)0x55))
        {
            /*Validate Frame Type*/
            if(BLMGR_DataRxBuffer[FRAME_TYPE_IDX] == (u8)FRAME_TYPE_DATA_FRAME)
            {
                /* Check CRC*/
                /*Calculate Crc from received data*/
                #if(COMM_CINFIG == MSTER_COMM)
                TempBuffer5[0x00] = BLMGR_DataRxBuffer[DIRECTION_IDX];
                TempBuffer5[0x01] = BLMGR_DataRxBuffer[SPEED_DEGREE_IDX];
                SECR_GnerateCrc(TempBuffer5, (u16)2, &GenCrc,BLMGR_CrcKey);
                #elif(COMM_CINFIG == SLAVE_COMM)
                TempBuffer[0x00] = BLMGR_DataRxBuffer[BATT_LEVEL_IDX];
                SECR_GnerateCrc(TempBuffer5, 1u, &GenCrc,BLMGR_CrcKey);
                #else
                /*Wrong Config, State in Idle*/
                /*To do: Managing dev Errors*/
                #endif
                /*Read Received CRC*/
                RecvdCrc = (u8)0x00;
                RecvdCrc = BLMGR_DataRxBuffer[FRAME_CRC_IDX];
                RecvdCrc |=(u16) ((((u16)(BLMGR_DataRxBuffer[FRAME_CRC_IDX + 1u])) << 8));

                /*Compare the Two Crcs*/
                /*if(GenCrc == RecvdCrc)*/
                {
                    /*Validate Frame Receiver*/
                    if(BLMGR_DataRxBuffer[FRAME_RECVER_IDX] == (u8)DEVICE_ROLE)
                    {
                        /*Update received paramters*/
                        /*Update Frame sender*/
                        /*BLMGR_RxFrameSender = BLMGR_DataRxBuffer[FRAME_SENDER_IDX];*/
                        #if(COMM_CINFIG == MSTER_COMM)
                        /*Update Direction*/
                      /*  BLMGR_RxDirection = BLMGR_DataRxBuffer[DIRECTION_IDX]; */
                        /*Update Speed degree*/
                      /*  BLMGR_RxSpeedDegree = BLMGR_DataRxBuffer[SPEED_DEGREE_IDX];*/
                        #elif(COMM_CINFIG == SLAVE_COMM)
                        BLMGR_RxBattLevel = BLMGR_DataRxBuffer[BATT_LEVEL_IDX];
                        #else
                        /*Wrong Config, State in Idle*/
                        /*To do: Managing dev Errors*/
                        #endif
                        /*Update error state*/
                        BLMGR_ErrorState = (u8)ERRH_NO_ERROR;
                        IsFrameValid1 = (u8)1;

                    }
                    else
                    {
                        /*Invalid Frame receiver*/
                        BLMGR_ErrorState = (u8)ERRH_INVALID_RECEIVER_ERROR;
                        IsFrameValid1 = (u8)0;
                    }

                }

            }
            else
            {
                /*Invalid Frame Type*/
                BLMGR_ErrorState = (u8)ERRH_WRONG_FRAME_ERROR;
                IsFrameValid1 = (u8)0;
            }
        }
        else
        {
            /*Invalid Frame Detected*/
            BLMGR_ErrorState = (u8)ERRH_INVALID_FRAME_ERROR;
            IsFrameValid1 = (u8)0;
        }
    }
    else
    {
        /*Checksum error*/
        BLMGR_ErrorState = (u8)ERRH_CHECKSUM_ERROR;
        IsFrameValid1 = (u8)0;
    }
    return IsFrameValid1;
}
/*********************************************************************************/
static void ErrorHandlingStateMachine(void)
{
    switch(BLMGR_ErrorState)
    {
        case (u8)ERRH_TIMEOUT_ERROR:
        {
            /*Check the Expected frame to be received*/
            switch(BLMGR_ExpectedReceivedFrame)
            {
                case (u8)FRAME_TYPE_ID_FRAME:
                {
                    if(BLMGR_HandshakeFailRepCount <= (u8)MAX_HANDSHAKE_FAIL_REP)
                    {
                        BLMGR_HandshakeFailRepCount ++;
                        BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_RECV_ID_FRAME;
                        /*Send Error frame*/
                        UpdateErrorFrame((u8)ERROR_TYPE_RESEND_LAST_FRMAE);
                        BLTD_StartReceivingData(BLMGR_DataRxBuffer,(u16)ID_FRAME_LENGTH,&RxBufferDnCallBackNotif);
                        BLTD_SendMessage(BLMGR_DataTxBuffer,(u16)ERROR_FRAME_LENGTH);
                    }
                    else
                    {
                        BLMGR_HandshakeFailRepCount = (u8)0;
                        /*Handshaking failed*/
                        BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_FAILED;
                    }
                }
                break;
                case (u8)FRAME_TYPE_VAL_FRAME:
                {
                    if(BLMGR_HandshakeFailRepCount <= (u8)MAX_HANDSHAKE_FAIL_REP)
                    {
                        BLMGR_HandshakeFailRepCount ++;
                        BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_RECV_VAL_FRAME;
                        /*Send Error frame*/
                        UpdateErrorFrame((u8)ERROR_TYPE_RESEND_LAST_FRMAE);
                        BLTD_StartReceivingData(BLMGR_DataRxBuffer,(u16)VAL_FRAME_LENGTH,&RxBufferDnCallBackNotif);
                        BLTD_SendMessage(BLMGR_DataTxBuffer,(u16)ERROR_FRAME_LENGTH);
                    }
                    else
                    {
                        BLMGR_HandshakeFailRepCount = (u8)0;
                        /*Handshaking failed*/
                        BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_FAILED;
                    }
                }
                break;
                case (u8)FRAME_TYPE_DATA_FRAME:
                {


                    if(BLMGR_CommFailReptCount <= (u8)MAX_COMM_FAIL_REP)
                    {
                        /*InserBreakPoint();*/
                        BLMGR_CommFailReptCount ++;
                        BLMGR_CommState = (u8)COMM_STATE_SEND_DATA_FRAME;
                        /*Send Error frame*/
                        /*UpdateErrorFrame(ERROR_TYPE_RESEND_LAST_FRMAE);*/
                        /*BLTD_StartReceivingData(BLMGR_DataRxBuffer,DATA_FRAME_LENGTH,RxBufferDnCallBackNotif);*/
                        /*BLTD_SendMessage(BLMGR_DataTxBuffer,ERROR_FRAME_LENGTH);*/
                    }
                    else
                    {
                        /*InserBreakPoint();*/
                        BuzzerSound();
                        BLMGR_CommFailReptCount = (u8)0;
                        /*Handshaking failed*/
                        BLMGR_CommState = (u8)COMM_STATE_FAILED;
                    }
                }
                break;

                default:
                    break;
            }
        }
        break;
        case (u8)ERRH_HANDSHAKE_ERROR:
        {
            if(BLMGR_HandshakeFailRepCount <= (u8)MAX_HANDSHAKE_FAIL_REP)
            {
                /*Start new handshaking*/
                BLMGR_HandshakeFailRepCount ++;
                BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_IDLE;
                /*Send Error frame*/
                UpdateErrorFrame((u8)ERROR_TYPE_START_NEW_HANDSHAKE);
                BLTD_StartReceivingData(BLMGR_DataRxBuffer,(u16)DATA_FRAME_LENGTH,&RxBufferDnCallBackNotif);
                BLTD_SendMessage(BLMGR_DataTxBuffer,(u16)ERROR_FRAME_LENGTH);
            }
            else
            {
                BLMGR_HandshakeFailRepCount = (u8)0;
                /*Handshaking failed*/
                BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_FAILED;
            }
        }
        break;
        case (u8)ERRH_CHECKSUM_ERROR:
        {
            /*Check the Expected frame to be received*/
            switch(BLMGR_ExpectedReceivedFrame)
            {
                case (u8)FRAME_TYPE_ID_FRAME:
                {
                    if(BLMGR_HandshakeFailRepCount <= (u8)MAX_HANDSHAKE_FAIL_REP)
                    {
                        BLMGR_HandshakeFailRepCount ++;
                        BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_RECV_ID_FRAME;
                        /*Send Error frame*/
                        UpdateErrorFrame((u8)ERROR_TYPE_RESEND_LAST_FRMAE);
                        BLTD_StartReceivingData(BLMGR_DataRxBuffer,(u16)ID_FRAME_LENGTH,&RxBufferDnCallBackNotif);
                        BLTD_SendMessage(BLMGR_DataTxBuffer,(u16)ERROR_FRAME_LENGTH);
                    }
                    else
                    {
                        BLMGR_HandshakeFailRepCount = (u8)0;
                        /*Handshaking failed*/
                        BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_FAILED;
                    }
                }
                break;
                case (u8)FRAME_TYPE_VAL_FRAME:
                {
                    if(BLMGR_HandshakeFailRepCount <= (u8)MAX_HANDSHAKE_FAIL_REP)
                    {
                        BLMGR_HandshakeFailRepCount ++;
                        BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_RECV_VAL_FRAME;
                        /*Send Error frame*/
                        UpdateErrorFrame((u8)ERROR_TYPE_RESEND_LAST_FRMAE);
                        BLTD_StartReceivingData(BLMGR_DataRxBuffer,(u16)VAL_FRAME_LENGTH,&RxBufferDnCallBackNotif);
                        BLTD_SendMessage(BLMGR_DataTxBuffer,(u16)ERROR_FRAME_LENGTH);
                    }
                    else
                    {
                        BLMGR_HandshakeFailRepCount = (u8)0;
                        /*Handshaking failed*/
                        BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_FAILED;
                    }
                }
                break;
                case (u8)FRAME_TYPE_DATA_FRAME:
                {
                    if(BLMGR_CommFailReptCount <= (u8)MAX_COMM_FAIL_REP)
                    {
                        BLMGR_CommFailReptCount ++;
                        BLMGR_CommState = (u8)COMM_STATE_RECV_DATA_FRAME;
                        /*Send Error frame*/
                        UpdateErrorFrame((u8)ERROR_TYPE_RESEND_LAST_FRMAE);
                        BLTD_StartReceivingData(BLMGR_DataRxBuffer,(u16)DATA_FRAME_LENGTH,&RxBufferDnCallBackNotif);
                        BLTD_SendMessage(BLMGR_DataTxBuffer,(u16)ERROR_FRAME_LENGTH);
                    }
                    else
                    {
                        BLMGR_CommFailReptCount = (u8)0;
                        /*Handshaking failed*/
                        BLMGR_CommState = (u8)COMM_STATE_FAILED;
                    }
                }
                break;

                default:
                    break;
            }
        }
        break;
        case (u8)ERRH_INVALID_FRAME_ERROR:
        {
            switch(BLMGR_ExpectedReceivedFrame)
            {
                case (u8)FRAME_TYPE_ID_FRAME:
                {
                    BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_FAILED;
                }
                break;
                case (u8)FRAME_TYPE_VAL_FRAME:
                {
                    BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_FAILED;
                }
                break;
                case (u8)FRAME_TYPE_DATA_FRAME:
                {
                    BLMGR_CommState = (u8)COMM_STATE_FAILED;
                }
                break;

                default:
                    break;
            }
        }
        break;
        case (u8)ERRH_CRC_ERROR:
        {
            switch(BLMGR_ExpectedReceivedFrame)
            {
                case (u8)FRAME_TYPE_ID_FRAME:
                {
                    BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_FAILED;
                }
                break;
                case (u8)FRAME_TYPE_VAL_FRAME:
                {
                    BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_FAILED;
                }
                break;
                case (u8)FRAME_TYPE_DATA_FRAME:
                {
                    BLMGR_CommState = (u8)COMM_STATE_FAILED;
                }
                break;

                default:
                    break;
            }
        }
        break;
        case (u8)ERRH_INVALID_RECEIVER_ERROR:
        {
            /*Check the Expected frame to be received*/
            switch(BLMGR_ExpectedReceivedFrame)
            {
                case (u8)FRAME_TYPE_ID_FRAME:
                {
                    if(BLMGR_HandshakeFailRepCount <= (u8)MAX_HANDSHAKE_FAIL_REP)
                    {
                        BLMGR_HandshakeFailRepCount ++;
                        BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_RECV_ID_FRAME;
                        /*Send Error frame*/
                        UpdateErrorFrame((u8)ERROR_TYPE_UPDATE_UR_TRANSMITTER);
                        BLTD_StartReceivingData(BLMGR_DataRxBuffer,(u16)ID_FRAME_LENGTH,&RxBufferDnCallBackNotif);
                        BLTD_SendMessage(BLMGR_DataTxBuffer,(u16)ERROR_FRAME_LENGTH);
                    }
                    else
                    {
                        BLMGR_HandshakeFailRepCount = (u8)0;
                        /*Handshaking failed*/
                        BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_FAILED;
                    }
                }
                break;
                case (u8)FRAME_TYPE_VAL_FRAME:
                {
                    if(BLMGR_HandshakeFailRepCount <= (u8)MAX_HANDSHAKE_FAIL_REP)
                    {
                        BLMGR_HandshakeFailRepCount ++;
                        BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_RECV_VAL_FRAME;
                        /*Send Error frame*/
                        UpdateErrorFrame((u8)ERROR_TYPE_UPDATE_UR_TRANSMITTER);
                        BLTD_StartReceivingData(BLMGR_DataRxBuffer,(u16)VAL_FRAME_LENGTH,&RxBufferDnCallBackNotif);
                        BLTD_SendMessage(BLMGR_DataTxBuffer,(u16)ERROR_FRAME_LENGTH);
                    }
                    else
                    {
                        BLMGR_HandshakeFailRepCount = (u8)0;
                        /*Handshaking failed*/
                        BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_FAILED;
                    }
                }
                break;
                case (u8)FRAME_TYPE_DATA_FRAME:
                {
                    if(BLMGR_CommFailReptCount <= (u8)MAX_COMM_FAIL_REP)
                    {
                        BLMGR_CommFailReptCount ++;
                        BLMGR_CommState = (u8)COMM_STATE_RECV_DATA_FRAME;
                        /*Send Error frame*/
                        UpdateErrorFrame((u8)ERROR_TYPE_UPDATE_UR_TRANSMITTER);
                        BLTD_StartReceivingData(BLMGR_DataRxBuffer,(u8)DATA_FRAME_LENGTH,&RxBufferDnCallBackNotif);
                        BLTD_SendMessage(BLMGR_DataTxBuffer,(u16)ERROR_FRAME_LENGTH);
                    }
                    else
                    {
                        BLMGR_CommFailReptCount = (u8)0;
                        /*Handshaking failed*/
                        BLMGR_CommState = (u8)COMM_STATE_FAILED;
                    }
                }
                break;

                default:
                    break;
            }
        }
        break;
        case (u8)ERRH_WRONG_FRAME_ERROR:
        {
            if(BLMGR_DataRxBuffer[FRAME_TYPE_IDX] == (u8)FRAME_TYPE_ERROR_FRAME)
            {
                /*Handle Error frame*/
                CheckErrorFrame();
            }
            else
            {
                switch(BLMGR_ExpectedReceivedFrame)
                {
                    case (u8)FRAME_TYPE_ID_FRAME:
                    {
                        BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_FAILED;
                    }
                    break;
                    case (u8)FRAME_TYPE_VAL_FRAME:
                    {
                        BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_FAILED;
                    }
                    break;
                    case (u8)FRAME_TYPE_DATA_FRAME:
                    {
                        BLMGR_CommState = (u8)COMM_STATE_FAILED;
                    }
                    break;

                    default:
                        break;
                }
            }
        }
        break;

        default:
            break;
    }
}
/*********************************************************************************/
static void UpdateErrorFrame(u8 ErrorType)
{
    /*Set Tx Frame to default values*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_HEADER_IDX],(u8)TX_FRAME_DEFUALT,(u16)MAX_DATA_BUFFER_LENGTH);
    /*Set Header of Frame*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_HEADER_IDX],(u8)0xaa,(u16)2);
    /*Set Device Sender*/
    BLMGR_DataTxBuffer[FRAME_SENDER_IDX] = (u8)DEVICE_ROLE;
    /*Set Device Receiver*/
    BLMGR_DataTxBuffer[FRAME_RECVER_IDX] = (u8)BLMGR_TxFrameReceiver;
    /*Set frame type*/
    BLMGR_DataTxBuffer[FRAME_TYPE_IDX] = (u8)FRAME_TYPE_ERROR_FRAME;
    /*Set paramter length*/
    BLMGR_DataTxBuffer[PARAM_LENGTH_IDX] = (u8)1;
    /*Set Error type*/
    BLMGR_DataTxBuffer[ERROR_TYPE_IDX] = (u8)ErrorType;
    /*Update Crc*/
    BLMGR_DataTxBuffer[FRAME_CRC_IDX] = (u8)TX_CRC_DEFAULT;
    BLMGR_DataTxBuffer[FRAME_CRC_IDX + 1u] = (u8)TX_CRC_DEFAULT;
    /*update Frame CheckSum*/
    BLMGR_DataTxBuffer[FRAME_CHECKSUM_IDX] = CalculateCheksum(BLMGR_DataTxBuffer,(u8)(FRAME_CHECKSUM_IDX -1u));
    /*update frame tail*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_TAIL_IDX],(u8)0x55,(u8)1);
}
/*********************************************************************************/
static void CheckErrorFrame(void)
{
    u8 ErrorType1;
    ErrorType1 = BLMGR_DataRxBuffer[ERROR_TYPE_IDX];
    switch(ErrorType1)
    {
        case (u8)ERROR_TYPE_RESEND_LAST_FRMAE:
        {
            switch(BLMGR_ExpectedReceivedFrame)
            {
                case (u8)FRAME_TYPE_ID_FRAME:
                {
                    BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_SEND_ID_FRAME;
                }
                break;
                case (u8)FRAME_TYPE_VAL_FRAME:
                {
                    BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_SEND_VAL_FRMAE;
                }
                break;
                case (u8)FRAME_TYPE_DATA_FRAME:
                {
                    BLMGR_CommState = (u8)COMM_STATE_SEND_DATA_FRAME;
                }
                break;

                default:
                    break;
            }
        }
        break;
        case (u8)ERROR_TYPE_START_NEW_HANDSHAKE:
        {
            BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_IDLE;
        }
        break;
        case (u8)ERROR_TYPE_UPDATE_UR_TRANSMITTER:
        {
            switch(BLMGR_ExpectedReceivedFrame)
            {
                case (u8)FRAME_TYPE_ID_FRAME:
                {
                    BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_SEND_ID_FRAME;
                }
                break;
                case (u8)FRAME_TYPE_VAL_FRAME:
                {
                    BLMGR_HandShakeState = (u8)HANDSHAKE_STATE_SEND_VAL_FRMAE;
                }
                break;
                case (u8)FRAME_TYPE_DATA_FRAME:
                {
                    BLMGR_CommState = (u8)COMM_STATE_SEND_DATA_FRAME;
                }
                break;

                default:
                    break;
            }
        }
        break;

        default:
            break;
    }
}
/*********************************************************************************/
static void MemCpy( u8* DesPtr, const u8* SrcPtr, u16 Length)
{
    u16 LoopIndex1;
    u8 * DesPtr1 = DesPtr;
    for(LoopIndex1 = (u16)0; LoopIndex1 < Length; LoopIndex1 ++)
    {
        *(DesPtr1 += LoopIndex1) = *(SrcPtr += LoopIndex1);
    }
    DesPtr1 -= LoopIndex1;
}
/*********************************************************************************/
static void MemSet(u8* DesPtr, u8 ConstVal, u16 Length)
{
    u16 LoopIndex2;
    u8 * DesPtr2 = DesPtr;
    for(LoopIndex2 = (u16)0; LoopIndex2 < Length; LoopIndex2 ++)
    {
        *(DesPtr2 += LoopIndex2) = ConstVal;
    }
    DesPtr2 -= LoopIndex2;
}
/*********************************************************************************/
#if (COMM_CINFIG == SLAVE_COMM)
static u8 MemCmp(const u8* Src1Ptr,const u8* Src2Ptr,u16 Length)
{
    u8 IsEqual = 1;
    u8 LoopIndex;
    for (LoopIndex = 0; (LoopIndex < Length) && (IsEqual == 1) ; LoopIndex ++)
    {
        if(*(Src1Ptr + LoopIndex) != *(Src2Ptr + LoopIndex))
        {
            IsEqual = 0;
        }
    }
    return IsEqual;
}
#endif
/*********************************************************************************/
static u8 CalculateCheksum(u8 * BufferPtr, u16 BufferLength)
{
    u32 Checksum = 0x00u;
    u16 LoopIndex3 = 0u;
    u8 * BufferPtr3 = BufferPtr;
    u8 Checksum3;
    for (LoopIndex3 = (u16)0; LoopIndex3 <= BufferLength; LoopIndex3 ++)
    {
        Checksum3 = (*(BufferPtr3 += (u32)LoopIndex3));
        Checksum = Checksum + (u32)Checksum3;
    }
    BufferPtr3 -= (u32)LoopIndex3;
    Checksum %= 256u;
    return (u8)Checksum;
}
/*********************************************************************************/
static u8 CheckCrc(void)
{
    u16 RxCrc;
    u16 GenCrc3 = 0;
    u8 TempBuffer6[MAX_DATA_BUFFER_LENGTH];
    u8 IsFrameValid6;
    RxCrc = (u16)0x00;
    RxCrc = BLMGR_DataRxBuffer[FRAME_VAL_CRC_IDX];
    RxCrc |=(u16) (((u16)(BLMGR_DataRxBuffer[FRAME_VAL_CRC_IDX +1u])) << 8);
    TempBuffer6[0x00] = (u8)TX_OS_TYPE;
    TempBuffer6[0x01] = (u8)TX_DEV_TYPE;
    MemCpy(&TempBuffer6[0x02],BLMGR_TxDevicName,(u16)BLMGR_TxDeviceNameLength);
    SECR_GnerateCrc(TempBuffer6,BLMGR_TxDeviceNameLength + (u16)2, &GenCrc3,BLMGR_CrcKey);
    if(GenCrc3 == RxCrc)
    {
        BLMGR_ErrorState = (u8)ERRH_NO_ERROR;
        IsFrameValid6 = (u8)1;
    }
    else
    {
        /*Crc Error Found*/
        BLMGR_ErrorState = (u8)ERRH_CRC_ERROR;
        IsFrameValid6 = (u8)0;
    }
    return IsFrameValid6;
}
/*********************************************************************************/
#if (COMM_CINFIG == SLAVE_COMM)
static u8 GetCrcKey(void)
{
    u8 IsFrameValid;
    u16 RxCrc;
    u16 GenCrc;
    static u8 TempBuffer[MAX_DATA_BUFFER_LENGTH];
    u8 LoopTerminated;
    u32 LoopIndex;
    RxCrc = 0x00;
    RxCrc = BLMGR_DataRxBuffer[FRAME_VAL_CRC_IDX];
    RxCrc |= ((u16)BLMGR_DataRxBuffer[FRAME_VAL_CRC_IDX +1]) << 8;
    TempBuffer[0x00] = TX_OS_TYPE;
    TempBuffer[0x01] = TX_DEV_TYPE;
    MemCpy(&TempBuffer[0x02],BLMGR_TxDevicName,BLMGR_TxDeviceNameLength);
    LoopTerminated = 0;
    for (LoopIndex = 0; (LoopIndex < 0xffff) && (LoopTerminated == 0); LoopIndex ++)
    {
        SECR_GnerateCrc(TempBuffer,BLMGR_TxDeviceNameLength + 2, &GenCrc,(LoopIndex | 0x10000));
        if(GenCrc == RxCrc)
        {
            BLMGR_CrcKey = LoopIndex;
            LoopTerminated = 1;
        }
    }
    if(LoopTerminated == 1)
    {
        /*Done and CRC Key Found*/
        BLMGR_ErrorState = ERRH_NO_ERROR;
        IsFrameValid = 1;
    }
    else
    {
        /*Crc Error Found*/
        BLMGR_ErrorState = ERRH_CRC_ERROR;
        IsFrameValid = 0;
    }
    return IsFrameValid;
}
#endif
/*********************************************************************************/
static void PowerBluetoothOn(void)
{
    DIO_WritePort(BlueToothPwrConfig.Portname,(u8)BLOUETOOTH_ON,BlueToothPwrConfig.Port_Mask);
}

/*********************************************************************************/
/*
static void PowerBluetoothOff(void)
{
    DIO_WritePort(BlueToothPwrConfig.Portname,(u8)!BLOUETOOTH_ON,BlueToothPwrConfig.Port_Mask);
}
*/
/*********************************************************************************/
static void DisconnectStateMachine(void)
{
}
/*********************************************************************************/
static void DisconnectInit(void)
{
    BLMGR_DisconectionTimeOut = (u8)0;
}
/*********************************************************************************/
static void BuzzerInit(void)
{
    DIO_InitPortDirection(BuzzerConfig.Portname,(u8)0xff,BuzzerConfig.Port_Mask);
    DIO_WritePort(BuzzerConfig.Portname,(u8)0x00,BuzzerConfig.Port_Mask);
}
/*********************************************************************************/
static void PowerBlueToothInit(void)
{
    DIO_InitPortDirection(BlueToothPwrConfig.Portname,(u8)0xff,BlueToothPwrConfig.Port_Mask);
    DIO_WritePort(BlueToothPwrConfig.Portname,(u8)0x00,BlueToothPwrConfig.Port_Mask);
}
/*********************************************************************************/
static void BlueToothKeyInit(void)
{
    DIO_InitPortDirection(BluetoothKeyConfig.Portname,(u8)0xff,BluetoothKeyConfig.Port_Mask);
    DIO_WritePort(BluetoothKeyConfig.Portname,(u8)0xff,BluetoothKeyConfig.Port_Mask);
}
/*
static void InserBreakPoint(void)
{
    DIO_WritePort(BuzzerConfig.Portname,(u8)0xff,BuzzerConfig.Port_Mask);
    while(1)
    {
        ;
    }
}
*/
