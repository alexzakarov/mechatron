; Arduino Blink Firmware for ATmega328P
; LED on PB5 (Arduino pin 13)
; Assembled with avr-gcc

#include <avr/io.h>

; Reset vector jumps to main
    jmp main
    reti  ; INT0
    reti  ; INT1
    reti  ; PCINT0
    reti  ; PCINT1
    reti  ; PCINT2
    reti  ; WDT
    reti  ; TIMER2 COMPA
    reti  ; TIMER2 COMPB
    reti  ; TIMER2 OVF
    reti  ; TIMER1 CAPT
    reti  ; TIMER1 COMPA
    reti  ; TIMER1 COMPB
    reti  ; TIMER1 OVF
    reti  ; TIMER0 COMPA
    reti  ; TIMER0 COMPB
    reti  ; TIMER0 OVF
    reti  ; SPI STC
    reti  ; USART RX
    reti  ; USART UDRE
    reti  ; USART TX
    reti  ; ADC
    reti  ; EEPROM READY
    reti  ; ANALOG COMP
    reti  ; TWI
    reti  ; SPM READY

main:
    ; Set PB5 (Arduino pin 13) as output
    ser r24           ; R24 = 0xFF
    out DDRB, r24     ; All PORTB pins as output

blink_loop:
    ; Turn LED ON (PB5 = 1)
    ldi r24, 0x20     ; PB5 mask
    out PORTB, r24    ; LED on

    ; Call delay
    rcall delay

    ; Turn LED OFF (PB5 = 0)
    ldi r24, 0x00     ; Clear PB5
    out PORTB, r24    ; LED off

    ; Call delay again
    rcall delay

    ; Loop forever
    rjmp blink_loop

; Simple delay subroutine - nested loops
delay:
    ldi r24, 0xFF     ; Outer counter
delay_outer:
    ldi r25, 0xFF     ; Middle counter
delay_middle:
    ldi r26, 0xFF     ; Inner counter
delay_inner:
    dec r26
    brne delay_inner

    dec r25
    brne delay_middle

    dec r24
    brne delay_outer

    ret
