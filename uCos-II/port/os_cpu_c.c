#define  OS_CPU_GLOBALS
#include "ucos_ii.h"
#include "stm32f4xx.h"


#if OS_TMR_EN > 0
static  INT16U  OSTmrCtr;
#endif


#if OS_CPU_HOOKS_EN > 0 && OS_VERSION > 203
void  OSInitHookBegin (void)
{	
#if OS_TMR_EN > 0
    OSTmrCtr = 0;
#endif
}
#endif


#if OS_CPU_HOOKS_EN > 0 && OS_VERSION > 203
void  OSInitHookEnd (void)
{
}
#endif

/*
*********************************************************************************************************
*                                          TASK CREATION HOOK
*
* Description: This function is called when a task is created.
*
* Arguments  : ptcb   is a pointer to the task control block of the task being created.
*
* Note(s)    : 1) Interrupts are disabled during this call.
*********************************************************************************************************
*/
#if OS_CPU_HOOKS_EN > 0
void  OSTaskCreateHook (OS_TCB *ptcb)
{
#if OS_APP_HOOKS_EN > 0
    App_TaskCreateHook(ptcb);
#else
    (void)ptcb;                                  /* Prevent compiler warning                           */
#endif
}
#endif



#if OS_CPU_HOOKS_EN > 0
void  OSTaskDelHook (OS_TCB *ptcb)
{
#if OS_APP_HOOKS_EN > 0
    App_TaskDelHook(ptcb);
#else
    (void)ptcb;                                  /* Prevent compiler warning                           */
#endif
}
#endif

#if OS_CPU_HOOKS_EN > 0 && OS_VERSION >= 251
void  OSTaskIdleHook (void)
{
#if OS_APP_HOOKS_EN > 0
    App_TaskIdleHook();
#endif
}
#endif

/************************************Return Hook***********************/
#if OS_CPU_HOOKS_EN > 0
void OSTaskReturnHook (OS_TCB *ptcb)
{
	(void)ptcb;
}
#endif



#if OS_CPU_HOOKS_EN > 0
void  OSTaskStatHook (void)
{
#if OS_APP_HOOKS_EN > 0
    App_TaskStatHook();
#endif
}
#endif



OS_STK *OSTaskStkInit (void (*task)(void *p_arg), void *p_arg, OS_STK *ptos, INT16U opt)
{
	OS_STK *stk;
    (void)opt;    
    stk       = ptos;  
#if (__FPU_PRESENT==1)&&(__FPU_USED==1)	
	*(--stk) = (INT32U)0x00000000L; //No Name Register  
	*(--stk) = (INT32U)0x00001000L; //FPSCR
	*(--stk) = (INT32U)0x00000015L; //s15
	*(--stk) = (INT32U)0x00000014L; //s14
	*(--stk) = (INT32U)0x00000013L; //s13
	*(--stk) = (INT32U)0x00000012L; //s12
	*(--stk) = (INT32U)0x00000011L; //s11
	*(--stk) = (INT32U)0x00000010L; //s10
	*(--stk) = (INT32U)0x00000009L; //s9
	*(--stk) = (INT32U)0x00000008L; //s8
	*(--stk) = (INT32U)0x00000007L; //s7
	*(--stk) = (INT32U)0x00000006L; //s6
	*(--stk) = (INT32U)0x00000005L; //s5
	*(--stk) = (INT32U)0x00000004L; //s4
	*(--stk) = (INT32U)0x00000003L; //s3
	*(--stk) = (INT32U)0x00000002L; //s2
	*(--stk) = (INT32U)0x00000001L; //s1
	*(--stk) = (INT32U)0x00000000L; //s0
#endif
    *(stk)    = (INT32U)0x01000000L;             /* xPSR                                               */
    *(--stk)  = (INT32U)task;                    /* Entry Point                                        */
    *(--stk)  = (INT32U)OS_TaskReturn;           /* R14 (LR) (init value will cause fault if ever used)*/
    *(--stk)  = (INT32U)0x12121212L;             /* R12                                                */
    *(--stk)  = (INT32U)0x03030303L;             /* R3                                                 */
    *(--stk)  = (INT32U)0x02020202L;             /* R2                                                 */
    *(--stk)  = (INT32U)0x01010101L;             /* R1                                                 */
    *(--stk)  = (INT32U)p_arg;                   /* R0 : argument                                      */

#if (__FPU_PRESENT==1)&&(__FPU_USED==1)	
	*(--stk) = (INT32U)0x00000031L; //s31
	*(--stk) = (INT32U)0x00000030L; //s30
	*(--stk) = (INT32U)0x00000029L; //s29
	*(--stk) = (INT32U)0x00000028L; //s28
	*(--stk) = (INT32U)0x00000027L; //s27
	*(--stk) = (INT32U)0x00000026L; //s26	
	*(--stk) = (INT32U)0x00000025L; //s25
	*(--stk) = (INT32U)0x00000024L; //s24
	*(--stk) = (INT32U)0x00000023L; //s23
	*(--stk) = (INT32U)0x00000022L; //s22
	*(--stk) = (INT32U)0x00000021L; //s21
	*(--stk) = (INT32U)0x00000020L; //s20
	*(--stk) = (INT32U)0x00000019L; //s19
	*(--stk) = (INT32U)0x00000018L; //s18
	*(--stk) = (INT32U)0x00000017L; //s17
	*(--stk) = (INT32U)0x00000016L; //s16
#endif
    *(--stk)  = (INT32U)0x11111111L;             /* R11                                                */
    *(--stk)  = (INT32U)0x10101010L;             /* R10                                                */
    *(--stk)  = (INT32U)0x09090909L;             /* R9                                                 */
    *(--stk)  = (INT32U)0x08080808L;             /* R8                                                 */
    *(--stk)  = (INT32U)0x07070707L;             /* R7                                                 */
    *(--stk)  = (INT32U)0x06060606L;             /* R6                                                 */
    *(--stk)  = (INT32U)0x05050505L;             /* R5                                                 */
    *(--stk)  = (INT32U)0x04040404L;             /* R4                                                 */

    return (stk);

}


#if (OS_CPU_HOOKS_EN > 0) && (OS_TASK_SW_HOOK_EN > 0)
void  OSTaskSwHook (void)
{
#if OS_APP_HOOKS_EN > 0
    App_TaskSwHook();
#endif
}
#endif


#if OS_CPU_HOOKS_EN > 0 && OS_VERSION > 203
void  OSTCBInitHook (OS_TCB *ptcb)
{
#if OS_APP_HOOKS_EN > 0
    App_TCBInitHook(ptcb);
#else
    (void)ptcb;                                  /* Prevent compiler warning                           */
#endif
}
#endif



#if (OS_CPU_HOOKS_EN > 0) && (OS_TIME_TICK_HOOK_EN > 0)
void  OSTimeTickHook (void)
{
#if OS_APP_HOOKS_EN > 0
    App_TimeTickHook();
#endif

#if OS_TMR_EN > 0
    OSTmrCtr++;
    if (OSTmrCtr >= (OS_TICKS_PER_SEC / OS_TMR_CFG_TICKS_PER_SEC)) {
        OSTmrCtr = 0;
        OSTmrSignal();
    }
#endif
}
#endif



void  OS_CPU_SysTickInit (void)
{
    INT32U  cnts;
	RCC_ClocksTypeDef  rcc_clocks;

	RCC_GetClocksFreq(&rcc_clocks);	//���ϵͳʱ��Ƶ�ʡ�
    cnts = rcc_clocks.HCLK_Frequency / OS_TICKS_PER_SEC;

	SysTick_Config(cnts);
}


#if OS_APP_HOOKS_EN > 0
void App_TaskCreateHook(OS_TCB *ptcb)
{
	(void) ptcb;
}

void App_TaskDelHook(OS_TCB *ptcb)
{
	(void) ptcb;
}

void App_TaskIdleHook(void)
{

}

void App_TaskStatHook(void)
{

}

#if OS_TASK_SW_HOOK_EN > 0
void App_TaskSwHook(void)
{

}
#endif

void App_TCBInitHook(OS_TCB *ptcb)
{
	(void)ptcb;
}

#if OS_TIME_TICK_HOOK_EN > 0
void App_TimeTickHook(void)
{

}
#endif


#endif /* OS_APP_HOOKS_EN */
