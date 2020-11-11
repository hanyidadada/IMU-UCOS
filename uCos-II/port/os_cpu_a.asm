
    EXTERN  OSRunning                                           ; External references
    EXTERN  OSPrioCur
    EXTERN  OSPrioHighRdy
    EXTERN  OSTCBCur
    EXTERN  OSTCBHighRdy
    EXTERN  OSIntNesting
    EXTERN  OSIntExit
    EXTERN  OSTaskSwHook

    EXPORT  OS_CPU_SR_Save                                      ; Functions declared in this file
    EXPORT  OS_CPU_SR_Restore
    EXPORT  OSStartHighRdy
    EXPORT  OSCtxSw
    EXPORT  OSIntCtxSw
    EXPORT  PendSV_Handler


NVIC_INT_CTRL   EQU     0xE000ED04                              ; Interrupt control state register.
NVIC_SYSPRI14   EQU     0xE000ED22                              ; System priority register (priority 14).
NVIC_PENDSV_PRI EQU           0xFF                              ; PendSV priority value (lowest).
NVIC_PENDSVSET  EQU     0x10000000                              ; Value to trigger PendSV exception.


    AREA |.text|, CODE, READONLY, ALIGN=2
    THUMB
    REQUIRE8
    PRESERVE8


OS_CPU_SR_Save
    MRS     R0, PRIMASK                                         ; Set prio int mask to mask all (except faults)
    CPSID   I
    BX      LR

OS_CPU_SR_Restore
    MSR     PRIMASK, R0
    BX      LR


OSStartHighRdy
    LDR     R0, =NVIC_SYSPRI14                                  ; Set the PendSV exception priority
    LDR     R1, =NVIC_PENDSV_PRI
    STRB    R1, [R0]

    MOVS    R0, #0                                              ; Set the PSP to 0 for initial context switch call
    MSR     PSP, R0
	
    LDR     R0, =OSRunning                                      ; OSRunning = TRUE
    MOVS    R1, #1
    STRB    R1, [R0]
	
    LDR     R0, =NVIC_INT_CTRL                                  ; Trigger the PendSV exception (causes context switch)
    LDR     R1, =NVIC_PENDSVSET
    STR     R1, [R0]

    CPSIE   I                                                   ; Enable interrupts at processor level

OSStartHang
    B       OSStartHang                                         ; Should never get here




OSCtxSw
    LDR     R0, =NVIC_INT_CTRL                                  ; Trigger the PendSV exception (causes context switch)
    LDR     R1, =NVIC_PENDSVSET
    STR     R1, [R0]
    BX      LR


OSIntCtxSw
    LDR     R0, =NVIC_INT_CTRL                                  ; Trigger the PendSV exception (causes context switch)
    LDR     R1, =NVIC_PENDSVSET
    STR     R1, [R0]
    BX      LR


PendSV_Handler
    CPSID   I                                                   ; Prevent interruption during context switch
	AND		LR, LR, #0xFFFFFFE0 								; Use FPU, MSP.
	ORR		LR, LR, #0x9
	
	MRS     R0, PSP                                             ; PSP is process stack pointer
    CBZ     R0, PendSVHandler_nosave                     ; Skip register save the first time
	
	;Is the task using the FPU context? If so, push high vfp registers.
	tst r14, #0x10
	it eq
	vstmdbeq r0!, {s16-s31}
	
	SUBS    R0, R0, #0x20
    STM     R0, {R4-R11}                                       ; Save remaining regs r4-11 on process stack    

    LDR     R1, =OSTCBCur                                       ; OSTCBCur->OSTCBStkPtr = SP;
    LDR     R1, [R1]
    STR     R0, [R1]                                            ; R0 is SP of process being switched out
                                                                ; At this point, entire context of process has been saved
PendSVHandler_nosave
    PUSH    {R14}                                               ; Save LR exc_return value
    LDR     R0, =OSTaskSwHook                                   ; OSTaskSwHook();
    BLX     R0
    POP     {R14}

    LDR     R0, =OSPrioCur                                      ; OSPrioCur = OSPrioHighRdy;
    LDR     R1, =OSPrioHighRdy
    LDRB    R2, [R1]
    STRB    R2, [R0]

    LDR     R0, =OSTCBCur                                       ; OSTCBCur  = OSTCBHighRdy;
    LDR     R1, =OSTCBHighRdy
    LDR     R2, [R1]
    STR     R2, [R0]

    LDR     R0, [R2]                                            ; R0 is new process SP; SP = OSTCBHighRdy->OSTCBStkPtr;
    LDM     R0, {R4-R11}                                        ; Restore r4-11 from new process stack
	ADDS    R0, R0, #0x20
	
	;Is the task using the FPU context? If so, pop the high vfp registers too.
	tst r14, #0x10
	it eq
	vldmiaeq r0!, {s16-s31}
	
    MSR     PSP, R0                                             ; Load PSP with new process SP
	ORR     LR, LR, #0x04                                       ; Ensure exception return uses process stack-FPU, PSP.
    CPSIE   I
    BX      LR                                                  ; Exception return will restore remaining context
    END

