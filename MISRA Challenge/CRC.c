﻿#include "CRC.h"
#include<stdlib.h>

/***************************************************************************************/
static u32 GetPower(u32 Base,u32 Pow);
static u16 gen_crc16(const u8 *data, u16 size, u32 CRC16);
/***************************************************************************************/
void SECR_CrcPolynomialGenerate(u32* PolynomialPtr,u8 CrcLengthInBits)
{
    u32 DevisorValue;
    DevisorValue = (u32)(GetPower((u32)2,(u32)CrcLengthInBits)) - 1u;
    *PolynomialPtr = ((u32)(rand()) % DevisorValue) + (u32)0x10000;
}
/***************************************************************************************/
void SECR_GnerateCrc(const u8* PayloadPtr,u16 PayloadLength, u16* CrcPtr, u32 CrcPoly)
{
    u16 LoopIndex = 0u;
    static u8 InternalBuffer[8];
    /*Copying data to internal buffer*/
    for (LoopIndex = 0u; LoopIndex < PayloadLength; LoopIndex++)
    {
        InternalBuffer[LoopIndex] = *(PayloadPtr+=LoopIndex);
    }
    PayloadPtr -= LoopIndex;
    /*perform bit wise invert on the data*/
    for (LoopIndex = 0u; LoopIndex < PayloadLength; LoopIndex++)
    {
        InternalBuffer[LoopIndex]  ^= (0xffu);
    }
    /*Generate CRC*/
    *CrcPtr = gen_crc16(InternalBuffer,PayloadLength*8u,0x18005u);
}
/***************************************************************************************/
static u32 GetPower(u32 Base,u32 Pow)
{
    u32 result = 1u;
    u32 Loop_Index;
    for (Loop_Index = 0u; Loop_Index < Pow; Loop_Index ++)
    {
        result *= Base;
    }
    return result;
}
/***************************************************************************************/
static u16 gen_crc16(const u8 *data, u16 size, u32 CRC16)
{
    u16 out = 0u;
    u16 bits_read = 0u, bit_flag;
    u16 i;
    u16 crc = 0u;
    u16 j = 0x0001u;
    /* Sanity check: */
    if(data == 0u)
    {
        crc = 0u;
        /*return 0*/
    }

    while(size > 0u)
    {
        bit_flag = out >> 15;

        /* Get next bit: */
        out <<= 1u;
        out |= ((((*data) >> (bits_read))) & 1u); /* item a) work from the least significant bits*/

        /* Increment bit counter: */
        bits_read++;
        if(bits_read > 7u)
        {
            bits_read = 0u;
            data+=1;
            size--;
        }

        /* Cycle check: */
        if(bit_flag)
        {
            out ^= CRC16;
        }

    }

    /* item b) "push out" the last 16 bits*/

    for (i = 0u; i < 16u; ++i) {
        bit_flag = out >> 15u;
        out <<= 1u;
        if(bit_flag)
        {
            out ^= CRC16;
        }
    }

    /*item c) reverse the bits*/

    i = 0x8000u;

    for (i = 0x8000u ; i != 0u; i >>=1u)
    {

            if (i & out)
            {
                crc |= j;
            }
            j <<= 1u;

    }

    return crc;
}
/***************************************************************************************/
