PROCESSOR 16F627A
; PIC16F627A Configuration Bit Settings

; Assembly source line config statements

  
; CONFIG
  CONFIG  FOSC = INTOSCCLK      ; Oscillator Selection bits (INTOSC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled)
  CONFIG  PWRTE = OFF           ; Power-up Timer Enable bit (PWRT disabled)
  CONFIG  MCLRE = OFF           ; RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is digital input, MCLR internally tied to VDD)
  CONFIG  BOREN = OFF           ; Brown-out Detect Enable bit (BOD disabled)
  CONFIG  LVP = OFF             ; Low-Voltage Programming Enable bit (RB4/PGM pin has digital I/O function, HV on MCLR must be used for programming)
  CONFIG  CPD = OFF             ; Data EE Memory Code Protection bit (Data memory code protection off)
  CONFIG  CP = OFF              ; Flash Program Memory Code Protection bit (Code protection off)

// config statements should precede project file includes.
#include <xc.inc>

#define _XTAL_FREQ 4000000

#define RS PORTA, 0          ;  RA0 is RS line of LCD
#define E  PORTA, 1          ;  RA1 is E line of LCD
                                  ;  RB0-RB3 are D4-D7 of LCD
; === Macros ===
Set_Cursor MACRO addr
    banksel PORTA
    movlw   addr
    call    SendCommand
    ENDM

Write_Char MACRO char
    banksel PORTA
    movlw   char
    call    SendData
    ENDM


EStrobe MACRO void                  ;  Strobe the "E" Bit
    bsf    E
    nop
    nop
    bcf    E
    ENDM


Temp		equ 0x20     ; temporary variables	  
BYTE		equ 0x21 
Counter		equ 0x22 
Integer_RH	equ 0x23
Decimal_RH	equ 0x24
Integer_T	equ 0x25
Decimal_T	equ 0x26
Checksum	equ 0x27
Dec_Hun		equ 0x28
Dec_Ten		equ 0x29
Dec_One		equ 0x2A
RH_Hun		equ 0x2B
RH_Ten		equ 0x2C
RH_One		equ 0x2D
T_Hun		equ 0x2E
T_Ten		equ 0x2F
T_One		equ 0x30

d1		equ 0x31
d2		equ 0x32
d3		equ 0x33
Temp_1          equ 0x34
Temp_2		equ 0x35

		
; Strings stored in program memory
String_Temp:
    retlw   'T'
    retlw   'e'
    retlw   'm'
    retlw   'p'
    retlw   ':'
    retlw   0   ; Null terminator

String_Humi:
    retlw   'H'
    retlw   'u'
    retlw   'm'
    retlw   'i'
    retlw   ':'
    retlw   0   ; Null terminator
		
		
PSECT resetVec,class=CODE,delta=2
; ------------------------------------------------------------------------------
; resetVec - entry point
; ------------------------------------------------------------------------------
resetVec:
    goto main

  
psect   intVec,class=CODE,delta=2 ; PIC10/12/16
intVec:   
    retfie

;====================================================================
; main - single initialization, then loop forever
;====================================================================
main:
    ; -------------------------------------------------
    ; 1) Do one-time init
    ; -------------------------------------------------
    call    init        ; Set up ports, etc.
    call    init_LCD    ; Initialize LCD to 4-bit mode, etc.

    ; -------------------------------------------------
    ; 2) Print static labels:
    ;    "Temp:" on row 1 (address 0x80),
    ;    "Humi:" on row 2 (address 0xC0).
    ; -------------------------------------------------
    
    ; Print "Temp:" on row 1
;    movlw   0x80        ; DDRAM address for row 1, column 0
;    call    SetCursor
;    movlw   HIGH(String_Temp)   ; Load high byte of String_Temp address
;    movwf   PCLATH
;    movlw   LOW(String_Temp)    ; Load low byte of String_Temp address
;    call    WriteString         ; Write the string to the LCD
;
;    ; Print "Humi:" on row 2
;    movlw   0xC0        ; DDRAM address for row 2, column 0
;    call    SetCursor
;    movlw   HIGH(String_Humi)   ; Load high byte of String_Humi address
;    movwf   PCLATH
;    movlw   LOW(String_Humi)    ; Load low byte of String_Humi address
;    call    WriteString         ; Write the string to the LCD

    ; Print "Temp:" on row 1
    Set_Cursor 0x80      ; Set cursor to row 1, column 0
    Write_Char 'T'
    Write_Char 'e'
    Write_Char 'm'
    Write_Char 'p'
    Write_Char ':'
    Set_Cursor 0x8A
    Write_Char 0xDF
    Write_Char 'C'

    ; Print "Humi:" on row 2
    Set_Cursor 0xC0      ; Set cursor to row 2, column 0
    Write_Char 'H'
    Write_Char 'u'
    Write_Char 'm'
    Write_Char 'i'
    Write_Char ':'
    Set_Cursor 0xCA
    Write_Char '%'


main_loop:   
   
    ; -------------------------------------------------
    ; 3) Send DHT start signal, read data
    ; -------------------------------------------------

    call    start_signal
    call    delay_50_us
    call    delay_50_us
    call    delay_50_us
    call    delay_10_us

    call    Read_Integer_RH
    call    Read_Decimal_RH
    call    Read_Integer_T
    call    Read_Decimal_T
    call    Read_Checksum

    ; Convert integer portions to ASCII for printing
    call    Integer_RH_to_ASCII
    call    Integer_T_to_ASCII

    ; -------------------------------------------------
    ; 4) Write Temperature to LCD
    ;    Assume we want to place it at row1, col5 => 0x80 + 5 = 0x85
    ; -------------------------------------------------
    banksel PORTA
    movlw   0x86
    call    SendCommand
    
    movf    T_Hun,W
    call    SendData
    
    movf    T_Ten,W
    call    SendData

    movf    T_One,W
    call    SendData


    ; -------------------------------------------------
    ; 5) Write Humidity to LCD
    ;    Assume we want to place it at row2, col5 => 0xC0 + 5 = 0xC5
    ; -------------------------------------------------
    movlw   0xC6
    call    SendCommand

    movf    RH_Hun,W
    call    SendData

    movf    RH_Ten,W
    call    SendData

    movf    RH_One,W
    call    SendData


    ; -------------------------------------------------
    ; 6) (Optional) delay before next reading. I set a 2 second delay
    ; -------------------------------------------------
    banksel	    TRISA

    MOVLW	    0x00		;set data direction of to output 
    MOVWF	    TRISA
    call            delay_500_ms
    call            delay_500_ms
    call            delay_500_ms
    call            delay_500_ms

    ; or something smaller, e.g.:
    ; call    delay_20_ms

    ; -------------------------------------------------
    ; 7) Loop forever
    ; -------------------------------------------------
    goto    main_loop

; === Subroutines ===
; Subroutine to set cursor position
; Input: W register = DDRAM address (e.g., 0x80 for row 1, 0xC0 for row 2)
SetCursor:
    banksel PORTA
    call    SendCommand ; Send the DDRAM address
    bsf     RS          ; RS = 1 (data mode)
    return
    
; Subroutine to write a string from program memory to the LCD
; Input: W register = low byte of string address
;        PCLATH = high byte of string address
WriteString:
    banksel PORTA
    call    GetChar         ; Call the string's starting address
WriteString_Loop:
    iorlw   0               ; Check if the character is null (end of string)
    btfsc   STATUS, 2       ; If null, exit
    return
    call    SendData    ; Send the character to the LCD
    call    GetChar         ; Get the next character
    goto    WriteString_Loop

; Subroutine to get the next character from the string
GetChar:
    movwf   PCL             ; Jump to the address in PCLATH:PCL
    
    
; ---------------------------------------------------------
; CheckBusyFlag
;   - Continuously reads the LCD busy flag (BF) in 4-bit mode
;   - Waits until BF == 0
;   - Returns when the LCD is no longer busy
; ---------------------------------------------------------
; ---------------------------------------------------------
; CheckBusyFlag
;   - Checks the LCD busy flag (BF) in 4-bit mode
;   - Waits until BF == 0
; ---------------------------------------------------------
CheckBusyFlag:
    banksel PORTA
    bcf     PORTA, 0      ; RS = 0 (command register)
    bsf     PORTA, 3      ; R/W = 1 (read mode)

    ; Set RB0-RB3 as inputs
    banksel TRISB
    movlw   0b00001111    ; RB0-RB3 = inputs, RB4-RB7 = outputs
    movwf   TRISB

CheckBusyLoop:
    ; Read high nibble (BF is bit 7 of the status byte)
    banksel PORTA
    bsf     PORTA, 1      ; E = 1
    nop
    nop
    banksel PORTB
    movf    PORTB, W      ; Read high nibble into W
    banksel PORTA
    bcf     PORTA, 1      ; E = 0

    ; Save W to a temporary register to test BF
    movwf   Temp_1          ; Temp = high nibble
    btfsc   Temp_1, 3       ; Test bit 3 (BF in 4-bit mode)
    goto    StillBusy     ; BF=1 => LCD is busy

    ; Read low nibble (to complete the status read)
    bsf     PORTA, 1      ; E = 1
    nop
    nop
    banksel PORTB
    movf    PORTB, W      ; Read low nibble (ignore value)
    banksel PORTA
    bcf     PORTA, 1      ; E = 0

    ; Restore RB0-RB3 as outputs
    banksel TRISB
    clrf    TRISB         ; RB0-RB3 = outputs
    banksel PORTA
    bcf     PORTA, 3      ; R/W = 0 (write mode)
    return

StillBusy:
    ; Read low nibble (to complete the status read)
    bsf     PORTA, 1      ; E = 1
    nop
    nop
    banksel PORTB
    movf    PORTB, W      ; Read low nibble (ignore value)
    banksel PORTA
    bcf     PORTA, 1      ; E = 0
    goto    CheckBusyLoop ; Loop back to check BF again
    
    
;---------------------------
; Sends W as 4-bit data:
;   - High nibble first
;   - Low nibble second
;   - Nibbles go onto RB0..RB3
;---------------------------
; Send a command to the LCD (RS=0)
; Send a command to the LCD (RS=0)
SendCommand:
    movwf   Temp_2          ; Save the command
    call    CheckBusyFlag ; Wait until LCD is ready
    banksel PORTA
    bcf     PORTA, 0      ; RS = 0 (command mode)
    movf    Temp_2, W       ; Restore the command to W
    goto    SendNibbles   ; Send the command

; Send data to the LCD (RS=1)
SendData:
    movwf   Temp_2          ; Save the data
    call    CheckBusyFlag ; Wait until LCD is ready
    banksel PORTA
    bsf     PORTA, 0      ; RS = 1 (data mode)
    movf    Temp_2, W       ; Restore the data to W
    ; Fall through to SendNibbles

; Common nibble-sending logic
SendNibbles:
    banksel PORTB
    movwf   Temp_2          ; Save the byte again (optional)
    
    ; Send high nibble
    swapf   Temp_2, W       ; Swap nibbles (high nibble in W<3:0>)
    andlw   0x0F          ; Mask out upper nibble
    movwf   PORTB         ; Send to LCD data pins (RB0-RB3)
    banksel PORTA
    bsf     PORTA, 1      ; E = 1
    nop                   ; Strobe delay
    nop
    bcf     PORTA, 1      ; E = 0

    ; Send low nibble
    movf    Temp_2, W       ; Reload original byte
    andlw   0x0F          ; Mask out upper nibble
    movwf   PORTB         ; Send to LCD data pins (RB0-RB3)
    banksel PORTA
    bsf     PORTA, 1      ; E = 1
    nop                   ; Strobe delay
    nop
    bcf     PORTA, 1      ; E = 0

    ; Optional delay for slow instructions
    ; call    delay_50_us
    return
    
    
init:
		banksel     PORTA
		CLRF	    PORTA		;Initialize PORTA setting output data latches
		MOVLW	    0x07		;Turn comparators off and
		MOVWF	    CMCON		;enable pins for I/O functions
		
		CLRF	    PORTB		;Initialize PORTB setting output data latches
		
		banksel	    TRISA

		MOVLW	    0x00		;Value used to initialize
						;   data direction
		MOVWF	    TRISA
		
		MOVLW	    0x00		;Value used to initialize;data direction
		MOVWF	    TRISB

		call	    delay_500_ms
		call	    delay_500_ms
		return
init_LCD:
                banksel	    PORTA
		movlw	    0x03
	        call	    SendCommand
		nop
		nop
		movlw	    0x03
	        call	    SendCommand
		nop
		nop
		movlw	    0x03
	        call	    SendCommand
		nop
		nop
		movlw	    0x28                     ;  4 bit, 2 Line, 5x7 font
	        call	    SendCommand
		nop
		nop
		movlw	    0x0C
	        call	    SendCommand
		nop
		nop
		movlw	    0x06
	        call	    SendCommand
		

		movlw	    0x01
	        call	    SendCommand
		call	    delay_500_ms      
		return
start_signal:
		
					    ; set DHT pin as output during init so no need to do it again 
		banksel	    PORTA
		bsf	    PORTA,2	    ; drive it high

		call	    delay_500_ms
		;call	    delay_500_ms    ;I want to make sure it starts high 
					      
					    ;(depending on your device, might need banksel)
					    ; e.g. if DHT pin is on RA2, then TRISA,2 = 0

		bcf	    PORTA,2	    ; drive it low
		call	    delay_20_ms
		call	    delay_10_ms


		bsf	    PORTA,2	    ; drive it high
		call	    delay_30_us
		;call	    delay_10_us
		
		banksel	    TRISA

					    ; set DHT pin as input
		bsf	    TRISA,2
		return					
	
Read_BYTE:
                banksel	     BYTE
		CLRF        BYTE                    ;BYTE=00000000
		MOVLW       8                    ;Check_bit 8 times, each time rotating left and updating the LSB
		MOVWF       Counter
		RLF         BYTE,f
		CALL        Check_bit
		DECFSZ      Counter,f
		GOTO        $-3
		RETURN

Check_bit:
		; 1) Clear TMR1
		clrf	    TMR1H
		clrf	    TMR1L

    ; 2) Wait for pin to go HIGH
WaitHigh:
		btfss	    PORTA,2
		goto	    WaitHigh

		; 3) Start timer
		bsf	    T1CON, 0

		; 4) Wait for pin to go LOW
WaitLow:
		btfsc	    PORTA,2
		goto	    WaitLow

		; 5) Stop timer
		bcf	    T1CON,0

		; 6) Compare TMR1L with 40
		banksel	    TMR1L
		movf	    TMR1L,w
		banksel	    STATUS   ; or same bank if needed
		movlw	    40
		subwf	    TMR1L,w  ; W = TMR1L - 40
		btfsc	    STATUS,0 ; if carry=1 => TMR1L >= 40
		bsf	    BYTE,0   ; set bit as '1'
		BCF         STATUS,0                ;clears STATUS,C

		return


; Input in W, store in BYTE
Convert_to_ASCII:
		banksel	    BYTE
		movwf	    BYTE
		clrf	    Dec_Hun
		clrf	    Dec_Ten
		clrf	    Dec_One

loop_100:
		movlw	    100
		subwf	    BYTE,f         ; BYTE -= 100
		btfsc	    STATUS,0       ; if carry=1 => Byte >= 100 => success
		goto	    inc_hun        ; so increment Dec_Hun & keep looping
		; else Byte < 100 => we borrowed, so add 100 back
		movlw	    100
		addwf	    BYTE,f
		goto	    tens_digits

inc_hun:
		incf	    Dec_Hun,f
		goto	    loop_100

tens_digits:
loop_10:
		movlw	    10
		subwf	    BYTE,f
		btfsc	    STATUS,0
		goto	    inc_ten
		movlw	    10
		addwf	    BYTE,f
		goto	    ones_digit

inc_ten:
		incf	    Dec_Ten,f
		goto	    loop_10

ones_digit:
		; Now BYTE < 10
		movf	    BYTE,w
		movwf	    Dec_One

		; Convert each digit to ASCII
		movlw	    0x30
		addwf	    Dec_Hun,f
		movlw	    0x30
		addwf	    Dec_Ten,f
		movlw	    0x30
		addwf	    Dec_One,f
		return
	    


Read_Integer_RH:
		banksel	     BYTE
		CALL        Read_BYTE
		MOVF        BYTE,W
		MOVWF       Integer_RH              ;Move BYTE to Integer_RH
		RETURN

Read_Decimal_RH:
		banksel	     BYTE
		CALL        Read_BYTE
		MOVF        BYTE,W
		MOVWF       Decimal_RH              ;Move BYTE to Decimal_RH
		RETURN

Read_Integer_T:
		banksel	     BYTE
		CALL        Read_BYTE
		MOVF        BYTE,W
		MOVWF       Integer_T              ;Move BYTE to Integer_T
		RETURN

Read_Decimal_T:
		banksel	     BYTE
		CALL        Read_BYTE
		MOVF        BYTE,W
		MOVWF       Decimal_T              ;Move BYTE to Decimal_T
		RETURN

Read_Checksum:
		banksel	     BYTE
		CALL        Read_BYTE
		MOVF        BYTE,W
		MOVWF       Checksum              ;Move BYTE to Checksum
		RETURN


;CHECKSUM

;CONVERT BINARY VALUES TO ASCII CHARACTERS TO SEND TO LCD
Integer_RH_to_ASCII:
		banksel	    Integer_RH
		MOVF        Integer_RH,W
		CALL        Convert_to_ASCII
		MOVF        Dec_Hun,W
		MOVWF       RH_Hun
		MOVF        Dec_Ten,W
		MOVWF       RH_Ten
		MOVF        Dec_One,W
		MOVWF       RH_One
		return

Integer_T_to_ASCII:
		banksel	    Integer_T
		MOVF        Integer_T,W
		CALL        Convert_to_ASCII
		MOVF        Dec_Hun,W
		MOVWF       T_Hun
		MOVF        Dec_Ten,W
		MOVWF       T_Ten
		MOVF        Dec_One,W
		MOVWF       T_One
		return


	   
    
delay_500_ms:
			;499994 cycles
	movlw	0x03
	movwf	d1
	movlw	0x18
	movwf	d2
	movlw	0x02
	movwf	d3
	
delay_500_ms_0:
	decfsz	d1, f
	goto	$+2
	decfsz	d2, f
	goto	$+2
	decfsz	d3, f
	goto	delay_500_ms_0

			;2 cycles
	goto	$+1
	
	                ; 4 cycles (including call)
	return
	
delay_500_us:
				;499 cycles
	movlw	0xA6
	movwf	d1
delay_500_us_0:
	decfsz	d1, f
	goto	delay_500_us_0

			;1 cycle
	nop
	return
	
delay_50_us:
				;49 cycles
	movlw	0x0F
	movwf	d1
delay_50_us_0:
	decfsz	d1, f
	goto	delay_50_us_0

			;1 cycle
	
	return
	
delay_20_us:
				;19 cycles
	movlw	0x05
	movwf	d1
delay_20_us_0:
	decfsz	d1, f
	goto	delay_20_us_0

			;1 cycle
	return
	
delay_10_us:
    			;10 cycles
	goto	$+1
	goto	$+1
	goto	$+1


	return
    
delay_30_us:
				; cycles
	movlw	0x08
	movwf	d1
delay_30_us_0:
	decfsz	d1, f
	goto	delay_30_us_0
	
	nop
			;1 cycle
	return	
delay_20_ms:	
			;19998 cycles
	movlw	0x9F
	movwf	d1
	movlw	0x10
	movwf	d2
delay_20_ms_0:
	decfsz	d1, f
	goto	$+2
	decfsz	d2, f
	goto	delay_20_ms_0

			;2 cycles
	goto	$+1
	return
	

delay_10_ms:	
			;9998 cycles
	movlw	0xCF
	movwf	d1
	movlw	0x08
	movwf	d2
delay_10_ms_0:
	decfsz	d1, f
	goto	$+2
	decfsz	d2, f
	goto	delay_10_ms_0

			;2 cycles
	goto	$+1
	return
	
	END 