


#include "Basic_Types.h"
#include "BLMGR.h"

#include "PWM.h"
#include "GPT.h"

void _delay_ms(u32 delay);
s32 main(void);
void sei(void);
void Cyclic30ms(void);

void Cyclic30ms(void)
{
	static u8 TimeoutCounter = 0u;
	TimeoutCounter ++;
	if(TimeoutCounter == 10u)
	{
		BLMGR_BluetoothStateMachine();
		TimeoutCounter = 0u;
		
		
	}
}
s32 main(void)
{
	s32 Count= 0;
	s32 start = 0;
	s32 Count2 = 0;
 


GPT_Timer30msInit(&Cyclic30ms);
	
	BLMGR_BluetoothInit();
	BLMGR_SetReceiver(ROLE_MAPP);
    
							
	PWM_Init();
	sei();
	PWM_SetSpeed(25u);

	
	while(1)
	{
		Count2 = (Count2 +1) % 20;
		BLMGR_SetBattLevel((u8)Count2 / 4u);
		
		_delay_ms(100u);
		Count ++;
		if(start == 0)
		{
					if(Count > 5)
					{
						BLMGR_StartDevice();

						start = 1;
					}
					
		}

	    
		
		
	}
	
	
}
