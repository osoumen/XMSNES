;------------------------------------------------------------------------------------------------------------------------
; Copyright (c) 2007, Mukunda Johnson
; 
; All rights reserved.
; 
; Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
; 
;     * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
;     * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
;     * Neither the name of the <ORGANIZATION> nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
;
;  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
; "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
; LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
; A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
; CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
; EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
; PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
; LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
; NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
; SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;-----------------------------------------------------------------------------------

;-----------------------
; XMSNES SPC DRIVER   /
; by eKid            /
;-- - - - - - - - --

; tabstop = 4

; UNCOMMENT FOR XM-SPC CONVERSION IMAGE
#define xmspc_mode

; UNCOMMENT FOR SAMPLE START FIX
#define xms_ssfix

; EFFECT DEFINITIONS (comment to exclude)
#define FX_INCLUDE_TREMOR
#define FX_INCLUDE_TREMOLO
#define FX_INCLUDE_ENVPOS
#define VFX_INCLUDE_EXTENDED


; SOME DEFINITIONS
c_xmsoffseth	=$21
c_xmsoffset		=$2100

ramp_size		=10  ; tweak with noise level

plimitl_lo			=1600
plimitl_hi			=7728 ;7744

plimita_lo			=112
plimita_hi			=8190 ;7744

c_inst_nsamps		=0
c_inst_noise		=1
c_inst_nvpoints		=2
c_inst_nppoints		=3
c_inst_envflags		=4
c_inst_volsus		=5
c_inst_volloopsx	=6
c_inst_volloopsp	=7
c_inst_volloopsv	=8
c_inst_volloopsy	=9
c_inst_volloope		=10
c_inst_pansus		=11
c_inst_panloopsx	=12
c_inst_panloopsp	=13
c_inst_panloopsv	=14
c_inst_panloopsy	=15
c_inst_panloope		=16
c_inst_fadeout		=17
c_inst_vibtype		=19
c_inst_vibsweep		=20
c_inst_vibdepth		=22
c_inst_vibrate		=23
c_inst_volenv		=24
c_inst_panenv		=48
c_inst_sampmap		=72

xmso_flen			=$0
xmso_len			=$2
xmso_chn			=$3
xmso_ins			=$4
xmso_smp			=$5
xmso_pat			=$6
xmso_spd			=$7
xmso_bpm			=$8

xmso_foff			=$A

xmso_pn				=$D
xmso_freqmode		=$9
xmso_restart		=$C
xmso_orders			=$1E

xmso_patto			=$16
xmso_insto			=$18
xmso_sampo			=$1A
xmso_loopo			=$1C


.include "dsp.inc"

.org $0000					; VARIABLES IN DIRECT PAGE 0

comm_write:					; writing position for transfers
	.word
comm_write2:				; writing position for transfers
	.word
comm_buffer:
	.word
data_length:				; length of transfer
	.word
comm_mode:					; current mode
	.word					; 0 = ready, 1 = getting samples

int1:						; General purpose
	.word
int2:						; General purpose
	.word
int3:						; General purpose
	.word
int4:						; General purpose
	.word
int6:						; General purpose
	.word
int7:						; General purpose
	.word
byte1:						; General purpose
	.byte

channel_bit:				; Contains channel bit for the channel currently being updated
	.byte
fx_return:					; Holds return address
	.word

cur_inst:					; Instrument of current channel being updated
	.word

; channel struct:
;   MAIN				byte[7]
;   0 period			word		16 bytes per channel
;   2 instrument		byte
;   3 volume fx			byte
;   4 effect			byte
;   5 param				byte
;	6 volume			byte
;	SUB					byte[9]
;	7 note				byte		; used for glissando aswell
;	8 AV_SWEEP_H		byte		-- auto-vibrato sweep
;	9 PANNING!			byte		
;   10 sample			byte		
;   11 counter			byte		; used for various effects
;   12 porta_speed		byte		; FOR PORTAMENTO
;	13 AV_SINEPOS		byte		
;	14 sine_pos			byte		; FOR VIBRATO
;	15 retrig/trem data	byte		; OMG RETRIGGER & TREMOR COUNTERS
;	etc

; channel struct offsets:
cso_period				=0
cso_inst				=2
cso_volfx				=3
cso_fx					=4
cso_fxparam				=5
cso_vol					=6
cso_note				=7
cso_avsweep				=8
cso_panning				=9
cso_sample				=10
cso_counter				=11
cso_portaspd			=12
cso_avsine				=13
cso_sinepos				=14
cso_rtdata				=15

; channel other info				( secondary channel buffer )
;  0 FADEOUT			word
;  2 ENV VOL			word		; hi 7 bits = y, lo bits = x		WORD
;  4 ENV PAN			word		; hi 7 bits = y, lo bits = x		WORD
;  6 FLAGS				byte		; kxttxvvg
									; g = glissando control
									; v = vibrato table
									; t = tremolo table
									; k = key on
;  7 AV_SWEEP_L

;  8 perioda_lo			used for vibrato
;  9 perioda_hi			used for vibrato
;  A envelope point PAN used for envelopes, duh
;  B envelope point VOL used for envelopes, duh
;  C envelope opt var
;  D envelope opt var
;  E volume addition
;  F note delay
coso_pa					=$8 ; for vibrato, period addition
coso_epv				=$A ; envelope optimization memory
coso_epp				=$B ;
coso_eyv				=$C ;
coso_eyp				=$D ;
coso_vadd				=$E ; volume addition
coso_ndelay				=$F ;

; oh and some notes about the counter on RETRIGGER NOTE
; lo 4 bits keeps the speed
; hi 4 bits are the ticker

xms_channels:		; Main channel buffer
	.block 128

xms_tremor_flags:
	.byte			; 1 bit per channel

xms_fxfirst:
	.byte

; channels are edited AFTER ticks
; variables to set data:
xms_cc_flags:
	.byte			; bits: 76543210
					; 0: start note
					; 1: set pitch
					; 2: set volume
					; 3: set panning
					; 4: set source
					; 5: start note with offset (FOR 9xx)
					; 6: reset all volumes (for global things) (will overwrite bit 2)
					; 7: perioda +/-

ccflag_keyon		=%1
ccflag_pitch		=%10
ccflag_volume		=%100
ccflag_panning		=%1000
ccflag_source		=%10000
ccflag_offset		=%100000
ccflag_volumereset	=%1000000

row_flags:
	.word

xms_cc_period:			; permanent period
	.word
xms_cc_perioda:			; USED FOR AUTO VIBRATO. UPDATED *EVERY* TICK
	.word
xms_cc_volume:			; volume for DSP
	.byte
xms_cc_panning:			; panning for DSP
	.byte
xms_cc_source:			; source for DSP
	.byte
xms_cc_sampoff:			; sample offset for DSP
	.byte

xms_samp_head:			; current sample header
	.byte
	.byte
	.byte
	.byte
	.byte

xsh_vol					=0 ; volume
xsh_ft					=1 ; finetune ( word )
xsh_pan					=3 ; panning
xsh_note				=4 ; relative note

xms_volume:				; XMS playback volume
	.byte
xms_speed:				; XMS speed
	.byte
xms_bpm:				; XMS bpm
	.byte
xms_position:			; XMS order table position
	.byte
xms_row:				; Current row being decoded
	.byte
xms_tick:				; Current tick
	.byte
xms_pattern_length:		; Length of pattern
	.byte
xms_playing:			; If player is running
	.byte
xms_pattern_read:		; Current XMS pattern read address
	.word
xms_pattern_plus:		; pattern secondary read offset
	.byte				
xms_pattern_repeat:		; pattern repetition variables
	.byte
	.byte
	.byte
	.byte
	.byte
	.byte
	.byte
	.byte
xms_patt_delay:			; set to zero at startup
	.byte				; pattern delay

xms_patt_jump:			; pattern jump position
	.word				
xms_patt_jump_enable:	; pattern jump enable
	.byte				; boolean
xms_patt_jump_offset:	; orderlist entry
	.word

xms_global_vol:		; global volume
	.byte
xms_patt_loop_pos:
	.word			; IF NOT ZERO, JUMP TO THIS ADDRESS WITH XMS_PATTERN_READ AND RESET
xms_patt_loop:
	.byte			; lo-4 bits = counter, hi 4-bits = jump boolean
xms_patt_loop_row:
	.byte			; set row to this

xms_peek:			; peek ahead data
	.byte

xms_clock:			; timer tick accumulator
	.byte
xms_clock_ticks:	; number of ticks required by timer to equal BPM
	.byte

xms_pclock:			; accumulator for pre-timer
	.byte
xms_pclock_ticks:
	.byte

xms_final_volume:	; final volume( for ramping )
	.block 8

dvar:
	.word

xms_last_message:
	.byte

; SNES->SPC
comm_validate:		; validation byte
	.byte

comm_samplewrite:	; for getting song samples
	.byte

; Sample playback
samp_playing:
	.byte
	.byte
	.byte
	.byte
	.byte
	.byte
	.byte
	.byte

samp_dirty:
	.byte

samp_usage:
	.byte

samp_play_next:
	.byte

;---------------------------------------------------------------------------------------------------------------------------------------------------------------------
; XMSNES PROGRAM CODE
;---------------------------------------------------------------------------------------------------------------------------------------------------------------------

.org $0600			; CODE AREA $600, continues to XMS offset

;------------------------------------------------------------------------------------------
; MAIN ROUTINE
;------------------------------------------------------------------------------------------

spx_main:
	mov ($F1), #%00110000			; clear input ports
	mov $F4, #$ED					; send ready signal
	mov $F5, #$FE					;

	mov (comm_validate), #0			; reset validation counter
	
	call SOUND_RESET				; reset sound
	call SOUND_ON					; turn on sound
	mov a, #50						; default volume
	mov y, a						;
	call SOUND_VOLUME				;
	mov (xms_volume), #255			; max xms playback vol
	
;---- XM-SPC CONVERSION ROUTINE --------.
#ifdef xmspc_mode						;
	mov a, (c_xmsoffset+xmso_freqmode)	;
	bne _xmsspc_rout_notamiga			;
	call cmsfm_amiga					;
_xmsspc_rout_notamiga:					;
	call XMS_BuildDirectory				;
	call XMS_Play						;
#endif									;
;---------------------------------------'

	mov (comm_write+1), #c_xmsoffseth
	mov (comm_write), #$00
	
_program_loop:						; main program loop
	call comm_routine				; ? Communications Routine
	call XMS_Player					; ? XMS Player Routine
	call XMS_VOLUMERAMP
	call XMS_CHECKSFX
	jmp _program_loop				; 3

;-------------------------------------------------------------------------------------------
; MAIN DSP FUNCTIONS
;-------------------------------------------------------------------------------------------
SOUND_RESET:
	; Soft Reset
	mov ($F2), #DSP_FLG
	mov ($F3), $%01100000
	; Reset other DSP registers
	call RESET_DSP_FILTER
	call RESET_DSP_VOICES
	mov  ($F2), #DSP_PMON
	mov  ($F3), #0
	ret
SOUND_ON:
	mov ($F2), #DSP_FLG				; 5 Turn off MUTE		{ 35 cycles
	mov ($F3), #$20					; 5 -------------
	mov ($F2), #DSP_NON				; 5 Turn off NOISE
	mov ($F3), #$00					; 5 --------------
	mov ($F2), #DSP_DIR				; 5 Setup Directory
	mov ($F3), #$02					; 5 ---------------
	ret								; 5 return				}
SOUND_OFF:
	mov ($F2), #DSP_FLG				; 5 Turn on MUTE		{ 15 cycles
	or  ($F3), #%01000000			; 5 ------------
	ret								; 5 return				}
SOUND_VOLUME:
	; a = left
	; y = right
	mov  ($F2), #DSP_MVOLL			; 5 Set VOL(L)			{ 23 cycles
	movz ($F3), a					; 4
	mov  ($F2), #DSP_MVOLR			; 5 Set VOL(R)
	movz ($F3), y					; 4						
	ret								; 5						}

;-------------------------------------------------------------------------------------------
; COMMUNICATIONS
;-------------------------------------------------------------------------------------------

COMM_MESSAGE_TABLE:					; COMMUNCIATION CODE
.word COMM_MSG_TRANSFERSAMP			; 14
.word COMM_MSG_TRANSFERB			; 15

.word COMM_MSG_REGISTER_SAMP		; 16
.word COMM_MSG_SENDSAMPLES			; 17

.word COMM_MSG_XMS_VOLUME			; 18
.word COMM_MSG_MAIN_VOLUME			; 19

.word COMM_MSG_TRANSFER				; 1A
.word COMM_MSG_BUILDDIR				; 1B
.word COMM_MSG_SETFREQMODE			; 1C
.word COMM_MSG_XMS_RESET			; 1D
.word COMM_MSG_XMS_PLAY				; 1E
.word COMM_MSG_XMS_PAUSE			; 1F

.word COMM_MSG_PLAYSAMPLE			; 20
.word COMM_MSG_SETSFXPARAM			; 21

COMM_MODES:
.word COMM_MODE0
;.word COMM_MODE2

;TRANSFER_EXITS:
;.word _comm_exit2

comm_routine:
	; routine to handle data transfers
	movz a, ($F7)							; 3 Check if SNES sent message
	cmpz a, (comm_validate)					; 3 
	bne _comm_getmessage					; 2/4
	
	ret
	
_comm_getmessage:
	nop					; extra delay :)
	nop
	movz a, ($F7)		; re-fetch value (incase old value was corrupted)
	movz (comm_validate), a					; 4 save validation
	
	movz a, (comm_mode)
	mov x, a
	jmp [COMM_MODES+x]
	
COMM_MODE0:
	
	movz a, ($F6)							; 3 fetch message #
	
	setc
	sbc a, #$14
	asl a
	mov x, a

	jmp [COMM_MESSAGE_TABLE+x]

_comm_exit:
	mov ($F7), (comm_validate)
	ret										; return from subroutine
	
	
_cmt_begin_transfer:
	
	mov (comm_write2), (comm_write)
	mov (comm_write2+1), (comm_write+1)
	decw (comm_write)							; 6	prepare transfer							14 cycles
	movz a, (comm_validate)						; 3
	movz ($F7), a								; 4 validate message
	mov  x, a									; 2 
	mov  y, #1									; 2
	
;	jmp _comm_transfer							; x start transfer
	
_comm_transfer:
	cmpz x, ($F7)											; 3	check for data			} 5 cycles
	beq _comm_transfer										; 2/4						}
	mov x, ($F7)											; 3	...	
															;
	movz a, ($F4)											; 3	fetch data			51 cycles...
	mov [comm_write]+y, a									; 7 store..
	
	movz a, ($F5)											; 3 fetch more data( hurry before snes overwrites )
	mov [comm_write2]+y, a									; 7
	
	movz a, ($F6)											; 3
	mov ($F7), x											; 4		snes is now active
	inc y
	inc y
	mov [comm_write]+y, a									; 7
	inc y
	beq _cmt_y_eq3											; 2/4
_cmt_y_ret3:
	
	cmp x, #0												; 2
	bne _comm_transfer										; 2/4  jump is taken
	
_comm_transfer_finish:
	mov a, y
	clrc
	adcz a, (comm_write)
	movz (comm_write), a
	adc  (comm_write+1), #0
	movz (comm_validate), x								; 4 save validation
	jmp _comm_exit										; 3
	
_cmt_y_eq3:
	mov y, #1					; 2 reset Y
	incz (comm_write+1)			; 4 increment writepos
	decw (comm_write)			; 6
	incz (comm_write2+1)
	decw (comm_write2)
	b _cmt_y_ret3				; 4
	
;----------------------------------------------------------------------------------------------
; COMMUNICATION MESSAGES
;----------------------------------------------------------------------------------------------
; a = message

COMM_MSG_TRANSFERSAMP:
	mov a, #0
	mov y, #0
	cmpw ya, $F4
	beq COMM_MSG_TRANSFERSAMP_REL
	movw ya, ($F4)
	movw (comm_write), ya
COMM_MSG_TRANSFERSAMP_REL:
	
	mov (int1+0), #$00
	mov (int1+1), #$02
	
	movz a, (comm_samplewrite)		; get next sample#
	asl a							; *4
	asl a
	mov y, a						; set index
	push y							; save value
	clrc							; 
	movz a, (comm_write)			; preserve comm_write and set directory pointer to comm_write+6 (skipping sample header):
	movz (int7), a					; comm_write preservation
	adc a, #6						; +6
	mov [int1]+y, a					; directory setting
	movz a, (comm_write+1)			; do hi byte
	movz (int7+1), a
	adc a, #0
	inc y
	mov [int1]+y, a					; done
	
	call COMM_MSG_TRANSFERB			; transfer data
	
	; setup loop start
	mov y, #3						; 3 = XMB.LOOP, hi byte
	mov a, [int7]+y					; load byte
	cmp a, #255
	beq _ts_noloop
	mov x, a						; preserve byte
	dec y							; next value
	mov a, [int7]+y					; load byte
	pop y							; restore directory index
	adci a, [int1]+y				; add loop start to directory offset
	inc y							; edit pointer
	inc y
	mov [int1]+y, a					; save value
	dec y							; get hi byte
	mov a, x
	adci a, [int1]+y				; add hi bytes
	inc y
	inc y
	mov [int1]+y, a					; save value
	b _ts_hasloop
_ts_noloop:
	pop y
	inc y
	inc y
	mov [int1]+y, a
	inc y
	mov [int1]+y, a
_ts_hasloop:
	
	
	
	
	movz a, (comm_validate)
_cmts_wl:
	cmpz a, ($F7)
	beq _cmts_wl
	mov  (comm_validate), ($F7)
	mov  ($F4), (comm_samplewrite)
	mov	 ($F7), (comm_validate)

	incz (comm_samplewrite)
	
	ret

COMM_MSG_REGISTER_SAMP:
	jmp _comm_exit
	
COMM_MSG_SENDSAMPLES:
	mov ($F6), #$9B
	;mov ($F7), (comm_validate)
	
	mov (comm_samplewrite), #0
	
	mov (int1+1), #$02		; int1 = $0200
	mov (int1),   #$00
	
	clrc
	
	mov  a, (c_xmsoffset+xmso_loopo)				; int2 = sample# directory
	mov  y, (c_xmsoffset+xmso_loopo+1)
	movw (int2), ya
	movw (int3), ya
	incw (int3)
	
	mov a, #0
	
cmss_samploop:
	
	movz (int4), a
	asl a
	mov y, a
	
	mov  a, [int3]+y		; get sample#
	movz (int4+1), a
	mov  a, [int2]+y		;
	movz (int6), a
	
	
cmss_loopto0:
	dec y					; search back to 0
	dec y
	
	bmi cmss_dloope			; 
cmss_dloop:					;
	cmpi a, [int2]+y			; see if sample exists
	beq cmss_match1			;

	jmp cmss_loopto0

cmss_dloope:
	
	movz a, (int6)
	movz ($F4), a
	movz a, (int4+1)
	movz ($F5), a
	movz a, (comm_validate)
	
	movz ($F7), a
	movz (comm_validate), a
	; receive transfer message
	
cmss_dloopwm:
	cmpz a, ($F7)
	beq cmss_dloopwm
	mov (comm_validate), ($F7)
	
	call COMM_MSG_TRANSFERSAMP_REL

	movz a, (comm_validate)		; sync with snes
cmss_dloopwm2:
	cmpz a, ($F7)
	beq cmss_dloopwm2
	mov (comm_validate), ($F7)
	
cmss_next:
	movz a, (int4)
	inc a
	cmp a, (c_xmsoffset+xmso_smp)
	bne cmss_samploop
cmss_exit:
	mov $F6, #$9C
	mov $F7, (comm_validate)
	ret
	
cmss_match1:				; check hi byte
	movz a, (int4+1)
	cmpi a, [int3]+y			;
	beq cmss_match2			;
	movz a, (int6)
	jmp cmss_loopto0
	
cmss_match2:
	
	mov a, y
	asl a
	mov y, a
	
	mov a, [int1]+y
	push a
	inc y
	mov a, [int1]+y
	push a
	inc y
	mov a, [int1]+y
	push a
	inc y
	mov a, [int1]+y
	push a
	
	mov a, (int4)
	asl a
	asl a
	adc a, #3
	mov y, a
	pop a
	mov [int1]+y, a
	pop a
	dec y
	mov [int1]+y, a
	pop a
	dec y
	mov [int1]+y, a
	pop a
	dec y
	mov [int1]+y, a
	
	jmp cmss_next
	
	
COMM_MSG_TRANSFER:
	mov (comm_write), $F4						; 5 set write offset
	mov (comm_write+1), $F5						; 5
COMM_MSG_TRANSFERB:
	jmp _cmt_begin_transfer						; 3 jump to transfer routine
	
COMM_MSG_BUILDDIR:
	call XMS_BuildDirectory
	jmp _comm_exit
COMM_MSG_SETFREQMODE:

	cmp ($F5), #1
	beq cmsfm_amiga
cmsfm_linear:
	
	mov		(xms_samp_head+xsh_ft+1), #0
	; reset mods
	mov		a, #0
	mov		XMS_PERIOD2FREQ,   a
	mov		XMS_PERIOD2FREQ+1, a
	mov		XMS_PERIOD2FREQ+2, a
	
	mov		XMS_PeriodClip,   a
	mov		XMS_PeriodClip+1, a
	
	mov		XMS_GetPeriod,   a
	mov		XMS_GetPeriod+1, a
	
	mov		_xmsfxa_modspace, a
	mov		_xmsfxa_modspace+1, a
	jmp		_comm_exit
	
cmsfm_amiga:
	
	mov		(xms_samp_head+xsh_ft+1), #$03
	
	; modify code
	mov		a, #$5F
	mov		(XMS_PERIOD2FREQ+0), a
	mov		a, #(XMS_PERIOD2FREQ_AMIGA & 255)
	mov		(XMS_PERIOD2FREQ+1), a
	mov		a, #(XMS_PERIOD2FREQ_AMIGA >> 8)
	mov		(XMS_PERIOD2FREQ+2), a
	
	mov		a, #$2F
	mov		(XMS_PeriodClip),   a
	mov		(XMS_GetPeriod),    a
	mov		(_xmsfxa_modspace), a
	
	mov		a, #(XMS_GetAmigaPeriod-(XMS_GetPeriod+2))
	mov		(XMS_GetPeriod+1), a
	
	mov		a, #(XMS_PeriodClipA-(XMS_PeriodClip+2))
	mov		(XMS_PeriodClip+1), a
	
	mov		a, #(_xmsfxa_amiga-(_xmsfxa_modspace+2))
	mov		(_xmsfxa_modspace+1), a
	
	jmp		_comm_exit
	
COMM_MSG_XMS_RESET:
	call XMS_Stop
	jmp _comm_exit
COMM_MSG_XMS_PLAY:
	call XMS_Play
	jmp _comm_exit
COMM_MSG_XMS_PAUSE:
	; not implemented
	jmp _comm_exit
COMM_MSG_XMS_VOLUME:
	movz a, ($F4)
	movz (xms_volume), a
	jmp _comm_exit
COMM_MSG_MAIN_VOLUME:
	movw ya, ($F4)
	mov  $F2, #DSP_MVOLL
	movz $F3, a
	mov  $F2, #DSP_MVOLR
	movz $F3, y
	jmp _comm_exit
	
COMM_MSG_PLAYSAMPLE:
	; find free channel
	mov (int1), #128
	mov (int1+1), #7
	mov a, $F7				; load 'zz' (see protocolsX.txt)
	xcn a					; xCCxxxxx - xxxxxCCx mask priority
	lsr						; xxxxxCCx - xxxxxxCC
	and a, #3				; xxxxxxCC - 000000CC
	inc a
	mov y, #8				; loop through channels
_cmps_findchannel:
	dec y
	cmpz y, (samp_usage)	; check if the following channels are reserved
	beq _cmps_exit			; exit if so
	cmp a, (samp_playing)+y	; check if priority is higher
	bcs _cmps_foundchan		; exit loop or continue search
	dec y
	b _cmps_findchannel
	
_cmps_foundchan:
	; y = channel#
	mov x, (samp_playing)+y
	beq _cmps_found0
	cmp x, (int1)
	bcs _cmps_findchannel
	movz (int1), x
	movz (int1+1), y
	b _cmps_findchannel
_cmps_found0:
	movz (int1+1), y
_cmps_searchcomplete:
	; a = prio
	movz y, (int1+1)
	mov (samp_playing)+y, a
;	movz a, (int1+1)	; get channel#
	mov  a, y
	xcn  a
	mov  x, a

	call XMS_CHANNEL_INTERRUPT

	; play sample....
	or   a, #$07	; turn off volume
	mov  y, #0
	movw ($F2), ya

	
	movz a, ($F4)		; set volume (left)
	and  a, #$F0
	bpl  _cmps_vl_good
	dec  a
_cmps_vl_good:
	movz ($F2), x
	movz ($F3), a
	incz ($F2)
	
	movz a, ($F4)		; set volume (right)
	and  a, #$0F
	xcn  a
	bpl  _cmps_vr_good
	dec  a
_cmps_vr_good:
	movz ($F3), a
	incz ($F2)			; set period lo

	mov  ($F3), #0
	incz ($F2)
	
	movz a, $F7			; set period hi
	and a, #%00011111
	asl a
	movz ($F3), a

	incz ($F2)			; set source#
	movz a, ($F5)
	movz ($F3), a
	mov  a, x			; set adsr/gain
	or   a, #$06
	mov  y, #%11100000
	movw $F2, ya
	dec  a
	mov  y, #%10001111
	movw $F2, ya
;	or 	 a, #$06
;	mov  y, #0
;	movw ($F2), ya
;	inc  a
;	mov  y, #127
;	movw ($F2), ya
	
	movz y, (int1+1)			; get channel#
	mov  a, (XMS_TABLE_BITS)+y	; translate to bit array
	mov  y, a
	mov  a, #$4C
	movw ($F2), ya		; key on
	mov  a, y
	orz  a, (samp_dirty)
	movz (samp_dirty), a
	
_cmps_realexit:

	jmp _comm_exit

_cmps_exit:
	mov a, (int1)
	bmi _cmps_realexit
	b _cmps_searchcomplete

_cmsfxp_params:
.byte samp_usage

COMM_MSG_SETSFXPARAM:
	movz y, ($F4)
	mov  a, (_cmsfxp_params)+y
	mov  x, a
	movz a, ($F5)
	movz ($00)+x, a

	jmp _comm_exit

COMM_MSG_UNDEFINED:
	jmp _comm_exit

;-----------------------------------------------------------------------------------------------
; XMS Initialization
;-----------------------------------------------------------------------------------------------
XMS_BuildDirectory:
	
	mov a, c_xmsoffset+xmso_loopo
	mov y, c_xmsoffset+xmso_loopo+1
	movw (int3), ya
	
	mov a, c_xmsoffset+xmso_sampo
	mov y, c_xmsoffset+xmso_sampo+1
	movw (int4), ya
												;
	mov y, #0									; 2 prepare to build directory
												;
	mov x, (c_xmsoffset+xmso_smp)				; 4
												;
	mov (int1), #$00							; 5
	mov (int1+1), #$02							; 5
												;
	decw (int3)									; 6
	decw (int3)									; 6
												;
	clrc										; 2 just in case :\					}
	
	; SKIP IN XMR MODE:
	
_bd_convert_loop:

	mov a, [int4]+y							; 6 $200+y = samp_offsets[y] + 5		{ 47 cycles		{ 96 cycles * nsamples
	adc a, #5								; 2
	mov [int1]+y, a							; 7
											;
	inc y									; 2
											;
	mov a, [int4]+y							; 6
	adc a, #0
	mov [int1]+y, a							; 7
											;
	inc y									; 2
											;
	mov a, [int3]+y							; 6 $202+y = sampl_offsets[y]
	mov [int1]+y, a							; 7										}
											;
	inc y									; 2										{ 49 cycles
											;
	mov a, [int3]+y							; 6
	mov [int1]+y, a							; 7
											;
	inc y									; 2
											;
	decw (int4)								; 6
	decw (int4)								; 6
	decw (int3)								; 6
	decw (int3)								; 6
											;
	dec x									; 2
											;	
	bne _bd_convert_loop					; 2/4									}				}

	mov a, c_xmsoffset+xmso_sampo		;; EDITED!!!! ---------------------------------------------------------------------
	mov y, c_xmsoffset+xmso_sampo+1
	movw (int3), ya

	mov a, #0								; 2										} 4 cycles
	mov x, #0								; 2										}
_bd_duplicate_check:						; check for sample copies

	push a									; 4 preserve sample number				{ 59 cycles
	asl a									; 2 fetch sample address
	adcz a, (int3)							; 3					; EDITED------------
	movz (int1), a							; 4
	mov (int1+1), (int3+1)					; 5					; EDITED------------
	adc (int1+1), #0						; 5
	mov y, #1								; 2
	mov a, [int1]+y							; 6
	movz (int4+1), a						; 4
	dec y									; 2
	mov a, [int1]+y							; 6
	movz (int4), a							; 4
	
	mov y, #4								; 2 fetch 'duplicate' byte
	mov a, [int4]+y							; 6 check and branch if original sample
	beq _bd_original						; 2/4									}

	dec a							; 2 Get offset address of original	} 8 cycles
	asl a							; 2									}
	asl a							; 2									}
	mov y, a						; 2									}
	
	
	mov a, $200+y					; 4 Copy						{ 36 cycles
	mov $200+x, a					; 5
	mov a, $201+y					; 4
	mov $201+x, a					; 5
	mov a, $202+y					; 4
	mov $202+x, a					; 5
	mov a, $203+y					; 4
	mov $203+x, a					; 5								}
_bd_original:
	
_bd_modified:
	inc x									; 2								} 8 cycles
	inc x									; 2								}
	inc x									; 2								}
	inc x									; 2								}
	
	pop a									; 4 restore sample number		} 14 cycles
											;								}
	inc a									; 2								}
	cmp a, (c_xmsoffset+xmso_smp)			; 4								}
	bcc _bd_duplicate_check					; 2/4							}
	
	ret										; 5
;--------------------------------------------------------------------------------------------------------------------------------------------------------
; XMS PLAYBACK CODE FOLLOWS
;--------------------------------------------------------------------------------------------------------------------------------------------------------
XMS_PERIOD2FREQ:
	nop
	nop							; <----------- RESERVED FOR MODIFICATION (LONG)
	nop

XMS_PERIOD2FREQ_LINEAR:											;											|
	; parameter:												;											| Optimization Priority: High
	; int1 = xm period - 1600									;											| Optimization Level: Medium/High
	; return:													;											| HEAVY LOAD
	; int3 = dsp frequency										;											|

	setc
	sbc (int1), #$40							; 5
	sbc (int1+1), #$06							; 5
																;
	; formula = ((freqtable[period >> 4] * (16 - (period & 15))) >> 4) + ((freqtable[(period >> 4) + 1] * (period & 15)) >> 4)
	; freqtable = $0300
	
	mov (int2), #$02		; int2 = $0300					5				} 12 cycles		{ 84 cycles
	mov (int2+1), #$03		;								5				}
	
	movz a, int1			; 3	ya = (period >> 4) * 2						} 26 cycles
	lsrz int1+1				; 4												}
	ror a					; 2												}
	lsrz int1+1				; 4												}
	ror a					; 2												}
	lsrz int1+1				; 4												}
	ror a					; 2												}
	movz y, (int1+1)		; 3												}
	and a, #254				; 2												}
	
	addw ya, (int2)			; int2 += (period>>4) * 2		5				} 14 cycles
	movw (int2), ya			;								4				}
							;												}
	movz a, (int1)			;								3				}
	and a, #15				;								2				}
							;
	; a = period & 15		;
	mov x, #0				;								2				} 6 cycles
							;												}
	cmp a, #0				;								2				}
	bne _p2f_irregular		;								4				}				}
	
	decw int2
	decw int2
	
	mov a, [int2+x]			; SHORTCUT		6					} 43 cycles
	movz (int3), a			;				4					}
	incw (int2)				;				6					}
	mov a, [int2+x]			;				6					}
	lsr a					;				2					}
	rorz (int3)				;				4					}
	lsr a					;				2					 }
	rorz (int3)				;				4					   }
	movz (int3+1), a		;				4						}
							;										}
	
	ret						;				5 -- 115 cycles regular	}
_p2f_irregular:
	
	; a = interpolation
	; int2 = table address
	; get initial freq and subtract next freq
	eor a, #15				;						2	{ 57 cycles
	inc a					;						2
	movz (byte1), a			;						4
							;
	mov  y, #3				;						2
	mov  a, [int2]+y		; get freq2				6
	movz (int3+1), a		;						4
	dec  y					;						2
	mov  a, [int2]+y		;						6
	movz (int3), a			;						4
	dec  y					;						2
	mov  a, [int2]+y		; get freq1				6
	mov  y, a				;						2
	mov  a, [int2+x]		;						6
	movw (int1), ya			;						4
	subw ya, (int3)			; subtract freq2		5	}
	
	;------------------------------------------------------
	push y					; 4 mul freqa * interpol	{ 45 cycles
	movz y, (byte1)			; 3
	mul ya					; 9
	movw (int2), ya			; 4
	pop a					; 4
	movz y, (byte1)			; 3
	mul ya					; 9
	mov y, a				; 2
	mov a, #0				; 2
	addw ya, (int2)			; 5							}
		
	xcn a					; 5 freqb >> 4				{ 35 cycles
	and a, #15				; 2
	movz (int2), a			; 4
	mov a, y				; 2
	xcn a					; 5
	push a					; 4
	and a, #$F				; 2
	mov y, a				; 2
	pop a					; 4
	and a, #$F0				; 2
	orz a, (int2)			; 3							}
	
	addw ya, (int1)			; 5 add initial and >> 2	{ 25 cycles
	movz (int3+1), y		; 4
	lsrz (int3+1)			; 4
	ror a					; 2
	lsrz (int3+1)			; 4
	ror a					; 2
	movz (int3), a			; 4							}
	
	ret						; 5							251 cycles total

XMS_PERIOD2FREQ_AMIGA:
	; AMIGA period->snes frequency
	; calculate 1832634/period
	; int1 = period
	; $F000 = amiga freq table
	
	movz	a, (int1+1)				; 3	{ ? cycles
	setc
	sbc		(int1), #112			; 5
	sbc		a, #0
	
	lsr		a						; 2
	rorz	(int1)					; 4
	and		(int1), #$FE

	clrc							; 2
	adc		a, #$F0					; 2
	movz	(int1+1), a				; 4
	mov		y, #0					; 2
	mov		a, [int1]+y				; 6
	movz	(int3), a				; 4
	inc		y						; 2
	mov		a, [int1]+y				; 6
	movz	(int3+1),a				; 4

	ret								; 5 }
	
;---------------------------------------------------------------------------------------------------
	
RESET_DSP_FILTER:
	mov $F2, #DSP_EVOLL		; 5	Reset Volume	{ 65 cycles
	mov $F3, #0				; 5
	mov $F2, #DSP_EVOLR		; 5
	mov $F3, #0				; 5
	mov $F2, #DSP_EVOLL		; 5
	mov $F3, #0				; 5
	
	mov $F2, #DSP_EDL		; 5 
	mov $F3, #0				; 5
	
	mov $F2, #DSP_ESA		; 5
	mov $F3, #$FF			; 5
	
	mov $F2, #DSP_EON		; 5
	mov $F3, #0				; 5
	
	ret						; 5					}
	
RESET_DSP_VOICES:
	
	mov $F2, #($70 | DSP_ADSR)
	setc
rdv_loop:
	mov $F3, #0
	incz $F2
	incz $F2
	mov $F3, #0
	sbc $F2, #$12
	bpl rdv_loop
	ret
	
XMS_Play:
	cmp (xms_playing), #1
	bne _xmsp_notplaying
	call XMS_Stop
_xmsp_notplaying:
	mov a, (c_xmsoffset+xmso_spd); 4
	movz (xms_speed), a			; 4
	mov (xms_position), #0		; 5

	; setup initial panning & reset repeat bytes
	mov y, #8
	mov x, #112
_xmsp_ip_loop:
	mov a, (c_xmsoffset+xmso_pn-1)+y
	lsr a
	lsr a
	movz (xms_channels+cso_panning)+x, a
	mov a, x
	setc
	sbc a, #16
	mov x, a
	dec y
	bne _xmsp_ip_loop
	
	; setup first pattern		;
	mov a, (c_xmsoffset + xmso_orders)	; 4
								;
	call XMS_SetPattern			; ?
								;
	mov a, (c_xmsoffset+xmso_bpm)		; 4
	call XMS_SetBPM				; 135 cycles
								;
	mov (xms_global_vol), #64	; 5
								;
	mov (xms_cc_flags), #0		; 5
								;
	mov (xms_playing), #1		; 5
	
	call XMS_ResetSamps
	
	ret							; 5
	
XMS_Stop:
	call RESET_DSP_VOICES
	mov (xms_playing), #0
	mov a, #$77
	mov y, #$00
	setc
_xmss_loop:
	movw ($F2), ya
	sbc a, #$10
	bcs _xmss_loop
	call XMS_ClearChannels
	ret

XMS_ClearChannels:
	mov x, #127
	mov a, #0
_xmscc_loop:
	mov (xms_channels)+x, a
	dec x
	bpl _xmscc_loop

	mov x, #7
	mov a, #0
_xmscc_vloop:
	mov (xms_final_volume)+x, a
	dec x
	bpl _xmscc_vloop
	
	ret

XMS_ResetSamps:
	mov a, #$70 + cso_sample
	mov y, #255
	setc
_xmscc_loop2:
	mov x, a
	movz (xms_channels)+x, y
	sbc a, #$10
	bcs _xmscc_loop2
	ret
	
XMS_SetBPM:						;[124 cycles]
	; a = bpm					; 
	; 32 = minimum				; 
	movz (xms_bpm), a			; 4				{ 42 cycles
	; setup timer				;
	
	
	mov $F1, #%00000000
	;          r-ba-210
	
	; compute 20000/bpm			; 
	push a						; 4
	mov y, a					; 2
	mov a, #0					; 2
								; 
	movw (int1), ya				; 4
	mov a, #(20000 & 255)		; 2
	mov y, #(20000 >> 8)		; 2
	mov (byte1), #0				; 5
_xms_sbpm_divprep:				; 
	cmpw ya, (int1)				; 4
	bcc _xms_sbpm_divready		; 2/4
	subw ya, (int1)				;  5
	incz (byte1)				;  4
	b _xms_sbpm_divprep			;  4			}
_xms_sbpm_divready:
	
	pop x						; 4				{ 82 cycles
	div ya, x					; 12
	movz y, (byte1)				; 3
	; ya = 20000/bpm
	movw (int2), ya

	call SPC_GetTimerValues01
	movz ($FA), a
	mov  (xms_clock), #0
	movz (xms_clock_ticks), y
	
	movw ya, (int2)
	setc
	sbc  a, #ramp_size
	bcs  _xmssb_mr_good
	dec  y
_xmssb_mr_good:
	call SPC_GetTimerValues01
	movz ($FB), a
	mov  (xms_pclock), #0
	movz (xms_pclock_ticks), y

	mov $F1, #%00000011
	;          r-ba-210
	
	ret							; 5				}

SPC_GetTimerValues01:			;[51 cycles]
	; ya = milliseconds*8		;
	;returns:					;
	; y = countup				;
	; a = timer value			;
	mov (int1+1), #1			; 5	reset counter			{ 13 cycles
	cmp y, #0					; 2 check if 8-bit value
	beq _spcst_exit				; 2/4						  nojump
_spcst_slow:					; ----------------------
	movz (int1), y				; 4 move HI byte to ram		}
_spcst_inc_loop:				; 
	lsrz (int1)					; 4 shift					{ 30 cycles (1 loop)
	beq _spcst_no_inc			; 2/4 check
	ror a						; 2 timer/2
	aslz (int1+1)				; 4 counter*2
	b _spcst_inc_loop			; 4 loop
_spcst_no_inc:					; 
	ror a						; 2 timer/2
	aslz (int1+1)				; 4 counter*2				}
_spcst_exit:					; 
	movz y, (int1+1)			; 3 load countup			{ 8 cycles
	ret							; 5 return					}
	
XMS_SetPattern:
	; a = pattern
	mov y, c_xmsoffset+xmso_patto
	movz (int1), y
	mov y, c_xmsoffset+xmso_patto+1
	movz (int1+1), y
	
;	mov (int1), (pattern_offsets)			; 5		{ 90 cycles
;	mov (int1+1), (pattern_offsets+1)		; 5
											; 
	asl a									; 2
	adc (int1+1), #0						; 5
	adcz a, (int1)							; 3
	adc (int1+1), #0						; 5
	movz (int1), a							; 4
											; 
	mov y, #0								; 2
											; 
	mov a, [int1]+y							; 6
	movz (xms_pattern_read), a				; 4
	inc y									; 2
	mov a, [int1]+y							; 6
	movz (xms_pattern_read+1), a			; 4
											; 
	mov y, #0								; 2
	mov a, [xms_pattern_read]+y				; 6
	movz (xms_pattern_length), a			; 4
											; 
	incw (xms_pattern_read)
											; 
	mov (xms_row), #0						; 5
	mov (xms_tick), #0						; 5
											; 
	mov (xms_patt_loop), #0					; 5
	mov (xms_patt_loop_row), #0

	movw ya, (xms_pattern_read)
	movw (xms_patt_loop_pos), ya

	; clear REPEAT data
	mov a, #0
	mov y, #4
_xmssp_clearrep:
	mov (xms_pattern_repeat-1)+y, a
	mov (xms_pattern_repeat+3)+y, a
	dec y
	bne _xmssp_clearrep
											; 
	ret										; 5		}
	
XMS_VRamp:							;[100 cycles]
	mov y, #%10011111				; 2		{ 2 cycles }
	bbc0 (xms_peek), _xmsvr_1		; 5/7	{ 88 cycles
	mov a, #$07						; 2
	movw ($F2), ya					; 4
_xmsvr_1:							;
	bbc1 (xms_peek), _xmsvr_2		; 5/7
	mov a, #$17						; 2
	movw ($F2), ya					; 4`
_xmsvr_2:							;
	bbc2 (xms_peek), _xmsvr_3		; 5/7
	mov a, #$27						; 2
	movw ($F2), ya					; 4
_xmsvr_3:							;
	bbc3 (xms_peek), _xmsvr_4		; 5/7
	mov a, #$37						; 2
	movw ($F2), ya					; 4
_xmsvr_4:							;
	bbc4 (xms_peek), _xmsvr_5		; 5/7
	mov a, #$47						; 2
	movw ($F2), ya					; 4
_xmsvr_5:							;
	bbc5 (xms_peek), _xmsvr_6		; 5/7
	mov a, #$57						; 2
	movw ($F2), ya					; 4
_xmsvr_6:							;
	bbc6 (xms_peek), _xmsvr_7		; 5/7
	mov a, #$67						; 2
	movw ($F2), ya					; 4
_xmsvr_7:							;
	bbc7 (xms_peek), _xmsvr_8		; 5/7
	mov a, #$77						; 2
	movw ($F2), ya					; 4
_xmsvr_8:							;
	mov (xms_peek), #0				; 5		{ 10 cycles
	jmp	_VR_return					; 5		}
	
;------------------------------------------------------------------------------------------------------
; ACTIVE ROUTINES
;------------------------------------------------------------------------------------------------------
XMS_Player:									; [ first/other]
	cmp (xms_playing), #1					; 5 check if XMS is playing				{ 33 cycles
	beq _xmsp_continue						; 2/4
	ret										;   5 return if not
_xmsp_continue:								;
	
	clrc
	movz a, ($FE)
	adcz a, (xms_pclock)
	
	cmpz a, (xms_pclock_ticks)
	bcc _xmsp_pcontinue
	mov (xms_pclock), #0
	movz a, (xms_peek)
	beq _xmsp_pskipr
	jmp XMS_VRamp
_xmsp_pcontinue:
	movz (xms_pclock), a
_xmsp_pskipr:
_VR_return:
	
	clrc									; 2 check timer
	movz a, ($FD)							; 3
	adcz a, (xms_clock)						; 3
	cmpz a, (xms_clock_ticks)				; 3
	bcs _xmsp_continue2						; 2/4
	movz (xms_clock), a						;   4
	ret										;   5 return if a tick hasn't passed
_xmsp_continue2:
	
	mov $F1, #%00000001	; reset
	;          r-ba-210
	mov $F1, #%00000011 ; set timers
	;          r-ba-210
	
	setc									; 2 edit timer values
	sbcz a, (xms_clock_ticks)				; 3
	movz (xms_clock), a						; 4										}
	; a tick has passed
;	mov ($F4), xms_tick						; DEBUG, DELETE THIS
;	mov ($F5), xms_row						; DEBUG, DELETE THIS
;	mov $F5, xms_pattern_read
;	mov $F4, xms_pattern_read+1
	
	cmp (xms_tick), #0						; 5										{ 9 cycles (first tick)
	beq _xmsp_tick_first					; 2/4
	jmp _xmsp_tick_between					; 3		LONG JUMP						}
_xmsp_tick_first:
	
	; tick is zero, get new data
	
	; PATTERN DELAY HACK
	cmp (xms_patt_delay), #0				; 5										{ 14 cycles
	mov (xms_fxfirst), #0
	beq _xmsp_no_delay						; 2/4
	decz (xms_patt_delay)					;  4 pattern delay != 0, repeat other ticks
	jmp _xmsp_tick_between					;  3									}
_xmsp_no_delay:
	
	mov x, #0								; 2 reset channel index					{ 12 cycles
	mov y, #1								; 2 reset pattern read offset, skipping peek byte
	
	; loop through channels
	mov (channel_bit), #1					; 5 reset 
	mov a, (c_xmsoffset+xmso_chn)			; 3										}
	
_xmsp_channel_loop:							; ENTERING LOOP (8x)												[2498 cycle peak]
	push a									; 4									{ 37 cycles
	
	; check repeat byte
	push x
	mov  a, x
	xcn  a
	mov  x, a
	movz a, (xms_pattern_repeat)+x
	beq  _xmsp_read_normal
	bmi  _xmsp_repeat_empty
_xmsp_repeat_norm:

	mov (row_flags+1), #0					; 5
	and (xms_fxfirst), #%10					; 5

	dec  a

	movz (xms_pattern_repeat)+x, a
	pop x
	
	; repeat stuff... DOESN'T APPLY FOR NOTE/INST
	
	jmp _xmsp_patt_repeated
_xmsp_repeat_empty:
	dec a
	cmp a, #$80
	bne _xmsp_repeate_normally
	mov a, #0
_xmsp_repeate_normally:

	mov (row_flags+1), #0					; 5
	and (xms_fxfirst), #%10					; 5

	movz (xms_pattern_repeat)+x, a
	pop x
	; empty things...
	mov a, #0
	movz (xms_channels+cso_volfx)+x, a
	movz (xms_channels+cso_fx)+x, a
	movz (xms_channels+cso_fxparam)+x, a
	jmp _xmsp_patt_repeated
	
_xmsp_read_normal:
	pop x
	
	mov a, [xms_pattern_read]+y				; 6
_xmsp_channel_continue:						
											;
	inc y									; 2
	cmp a, #0								; 2		check if MSB is set
	bmi _xmsp_row_small						; 2/4
	mov (row_flags), #%10101101				;         all follow
	dec y									; 2
	b _xmsp_row_large						; 4		  skip
_xmsp_row_small:
	movz (row_flags), a						; 4		  row is compressed
_xmsp_row_large:							;
	
	; row_flags hi-flags:					;
	; 1 = set pitch							;
	; 2 = set sample						;
	; 3 = set volume						;
	mov (row_flags+1), #0					; 5
	and (xms_fxfirst), #%10					; 5									}
	
	bbc1 (row_flags), _xmsp_no_instr
_xmsp_yes_instr:
	bbs2 (row_flags), _xmsp_noter
	or (row_flags+1), #%110
	call _xmsp_notecheck
	jmp _xmsp_noteinst_finish

_xmsp_no_instr:
	call _xmsp_notecheck
	bbc2 (row_flags), _xmsp_no_instrument
	mov a, [xms_pattern_read]+y
	inc y
	dec a
	cmp a, (c_xmsoffset+xmso_ins)
	bcc _xmsp_inst_valid
	mov a, #255
_xmsp_inst_valid:
	inc a
	movz (xms_channels+cso_inst)+x, a
	or   (row_flags+1), #%110
_xmsp_no_instrument:

	jmp _xmsp_noteinst_finish

_xmsp_notecheck:
	bbc0 (row_flags), _xmsp_notecheck_exit
	mov a, [xms_pattern_read]+y
	inc y
	dec a
	cmp a, #96
	bne _xmsp_note_NOTKEYOFF
	call XMS_CHANNEL_ENV_KEYOFF
	ret
_xmsp_note_NOTKEYOFF:
	movz (xms_channels+cso_note)+x, a
	or   (row_flags+1), #%1001
	or   (xms_fxfirst), #%11
_xmsp_notecheck_exit:
	ret

_xmsp_noter:
	bbc0 (row_flags), _xmsp_noter_i

	mov a, [xms_pattern_read]+y					; WASTE
	inc y										; WASTE
	dec a										; WASTE
	cmp a, (c_xmsoffset+xmso_ins)				; WASTE
	bcc _xmsp_inst_valid2						; WASTE
	mov a, #255									; WASTE
_xmsp_inst_valid2:								; WASTE
	inc a										; WASTE
	movz (xms_channels+cso_inst)+x, a			; WASTE
	
_xmsp_noter_i:
	or (row_flags+1), #%1111
	
_xmsp_noteinst_finish:
	
	bbc3 (row_flags), _xmsp_no_volume		; 5/7   check for volume fx			{ 27 cycles
	mov  a, [xms_pattern_read]+y			; 6		  read
	inc  y									; 2
	movz (xms_channels+3)+x, a				; 5		  store
	or   (row_flags+1), #%100				; 5
	b    _xmsp_is_volume					; 4
_xmsp_no_volume:							;
	mov  a, #0								; 2		  store zero
	movz (xms_channels+3)+x, a				; 5									}
_xmsp_is_volume:							;
	
	
	bbc4 (row_flags), _xmsp_fx_00
_xmsp_fx_01:
	bbs5 (row_flags), _xmsp_fx_11
	b _xmsp_fx_exit
_xmsp_fx_00:
	bbs5 (row_flags), _xmsp_fx_10
	mov a, #0
	movz (xms_channels+cso_fx)+x, a
	movz (xms_channels+cso_fxparam)+x, a
	b _xmsp_fx_exit
_xmsp_fx_10:
	mov a, [xms_pattern_read]+y
	
	inc y
	movz (xms_channels+cso_fx)+x, a
	mov a, [xms_pattern_read]+y
	inc y
	movz (xms_channels+cso_fxparam)+x, a
	b _xmsp_fx_exit
_xmsp_fx_11:
	mov a, [xms_pattern_read]+y
	inc y
	movz (xms_channels+cso_fxparam)+x, a
_xmsp_fx_exit:
	
	; repeat stuff...?

	bbc6 (row_flags), _xmsp_norepeat
	; a = repeat byte, store somewhere
	push x
	mov a, x
	xcn a
	mov x, a

	mov a, [xms_pattern_read]+y
	inc y
	
	movz (xms_pattern_repeat)+x, a
	pop x
_xmsp_norepeat:
	
_xmsp_patt_repeated:

	movz (xms_pattern_plus), y				; 4		preserve read offset	4 cycles
	
	bbc0 (row_flags), _xmsp_nonotecheckthingy ; 5/7 GLISSANDO HACK			{ 27 cycles -- only happens with glissando
	movz a, (xms_channels+4)+x				 ; 4
	cmp a, #3								; 2
	beq _xmsp_omgdontsetnote				; 2/4
	cmp a, #5								; 2
	beq _xmsp_omgdontsetnote				; 2/4
	movz a, (xms_channels+3)+x				; 4
	and a, #$F0								; 2
	cmp a, #$F0								; 2
	bne _xmsp_nonotecheckthingy				; 2/4	negated branch
_xmsp_omgdontsetnote:						;
	and  (row_flags+1), #%0110				; 5 overwrites note flag		}
_xmsp_nonotecheckthingy:					;

	mov  a, x						; dont change instrument settings while a sample is playing
	xcn  a
	mov  y, a
	mov  a, (samp_playing)+y
	bne  _xmsp_no_inst				;---------------
	
	movz a, (xms_channels+cso_inst)+x		; 4								{ 16 cycles
	cmp a, #0								; 2
	bne _xmsp_valid_instrument				; 2/4
	mov a, #1								;   2
_xmsp_valid_instrument:						; 
	call XMS_GetInstOffset					; 8+49
											; 
	mov1 c, (row_flags+1+(1<<13))			; 4
	bcc _xmsp_no_inst						; 2/4							}
	
	; instrument value was set, get new sample
	
	mov y, #c_inst_nsamps						; 2 get n_samples			{ 30 cycles
	mov a, [cur_inst]+y							; 6
	cmp a, #1									; 2
	
	bne _xmsp_inst_samples_g1					; 2/4
	mov y, #c_inst_sampmap						; 2 monosampled instrument
	mov a, [cur_inst]+y							; 6
												;
	b _xmsp_inst_samples_e1						; skip
_xmsp_inst_samples_g1:
	movz a, (xms_channels+7)+x					; 4	multisampled instrument
	clrc										; 2
	adc a, #c_inst_sampmap						; 2
	mov y, a									; 2
	mov a, [cur_inst]+y							; 6							}
	
_xmsp_inst_samples_e1:
	
	; a = samp number
	
	
	cmpz a, (xms_channels+cso_sample)+x			; 4 check for sample change	{ 159 cycles
	beq _xmsp_inst_nochange						; 2/4
	movz (xms_channels+cso_sample)+x, a			;	5 store
	movz (xms_cc_source), a						;	4 change source
	or   (xms_cc_flags), #ccflag_source			;	5 set flag				
_xmsp_inst_nochange:							;
	
	call XMS_GetSampleHeader					; 8+102			read sample header
	movz a, (xms_samp_head+xsh_vol)				; 3 get volume
	movz (xms_channels+6)+x, a					; 5
	movz (xms_cc_volume), a						; 4 store
	
	movz a, (xms_samp_head+xsh_pan)				; 3 get panning
	bpl _xmsp_setpan_off
	movz (xms_channels+9)+x, a					; 5 store
_xmsp_setpan_off:
	
	or (xms_cc_flags), #( ccflag_volume | ccflag_panning )			; 5		}
	
	mov a, #$FF									; 2 Reset Fadeout			{ 81 cycles
	mov (xms_channels_other+0)+x, a				; 6
	mov (xms_channels_other+1)+x, a				; 6
	call XMS_CHANNEL_ENV_RESET					; 8+48
	
	mov a, #0									; 2 reset auto-vibrato sweep
	movz (xms_channels+cso_avsweep)+x, a		; 5
	
	b _xmsp_is_inst								; 4 skip					}
_xmsp_no_inst:
	
	call XMS_GetSampleHeader					; 8+102 read sample header	{ x cycles } (skipped)
_xmsp_is_inst:
	
	bbc0 (row_flags+1), _xmsp_no_note_entry		; 5/7 Check for note		{ 85 cycles
	movz a, (xms_channels+cso_note)+x
	call XMS_GetPeriod							; 8+57 get period
	movz (xms_channels+0)+x, a					; 5 store
	movz (xms_channels+1)+x, y					; 5
	or (xms_cc_flags), #(ccflag_pitch | ccflag_keyon)		; 5 set flag	}
	
_xmsp_no_note_entry:
	
	; get channel information
	movz a, (xms_channels+0)+x			; 4 period							{ 28 cycles
	movz y, (xms_channels+1)+x			; 4
	movw (xms_cc_period), ya			; 4
	movz a, (xms_channels+cso_vol)+x	; 4 volume
	movz (xms_cc_volume), a				; 4
	movz a, (xms_channels+9)+x			; 4 panning
	movz (xms_cc_panning), a			; 4

	or (xms_cc_flags), #ccflag_pitch
										;									}
	mov a, (xms_channels_other+coso_ndelay)+x
	bne _xmsp_nd1
	call XMS_AV_UPDATE					; 8+213								{ 1848 cycles
										; 
	call XMS_ProcessVolumeEffect		; 8+112
										;
_xmsp_nd1:
	call XMS_ProcessMainEffect			; 8+217
										;
	mov a, (xms_channels_other+coso_ndelay)+x
	bne _xmsp_nd2
	call XMS_CHANNEL_ENV_ROUTINE		; 8+628
										;
	call XMS_CHANNEL_UPDATEDSP			; 8+638								}

	
_xmsp_nd2:
	
	aslz (channel_bit)					; 4 shift channel bit				{ 8 cycles
	
	movz y, (xms_pattern_plus)			; 4 restore pattern reader			}
_xmsp_channel_next:						;
	clrc								; 2 increment channel offset		{ 21 cycles
	mov a, x							; 2
	adc a, #16							; 2
	mov x, a							; 2
	pop a								; 4 restore loop counter
	dec a								; 4
	
	beq _xmsp_channel_exit				; 2/4 long cbranch
	jmp _xmsp_channel_loop				;   3								}
_xmsp_channel_exit:						;------------------------------------------------------------
	
	mov a, y							; 2 add y to pattern read offset	{ 18 cycles
	adcz a, (xms_pattern_read)			; 3
	adc (xms_pattern_read+1), #0		; 5
	movz (xms_pattern_read), a			; 4
	
	b _xmsp_tick_notbetween				; 4 skip							}
	
_xmsp_tick_between:						; ON OTHER TICKS...
	
	mov x, #0							; 2 reset channel offset		{ 11 cycles
	mov a, (c_xmsoffset+xmso_chn)		; 4
	mov (channel_bit), #1				; 5 reset channel bit			}
_xmsptb_loop:							;																	[2347 cycles other :O]
	push a								; 4 preserve loop counter		{ 171 cycles
	
	movz a, (xms_channels+cso_inst)+x	; 4								
	call XMS_GetInstOffset				; 8+49 Get Inst Offset
	call XMS_GetSampleHeader			; 8+98 Get Samp Header			}
	
	; get channel information
	movz a, (xms_channels+0)+x			; 4 .....						{ 28 cycles
	movz y, (xms_channels+1)+x			; 4
	movw (xms_cc_period), ya			; 4
	movz a, (xms_channels+cso_vol)+x	; 4
	movz (xms_cc_volume), a				; 4
	movz a, (xms_channels+9)+x			; 4
	movz (xms_cc_panning), a			; 4								}
	
	mov a, (xms_channels_other+coso_ndelay)+x
	bne _xmsp_ndo1
	call XMS_AV_UPDATE					; 8+213							{ 2130 cycles
										; 
	call XMS_ProcessVolumeEffect		; 8+237
_xmsp_ndo1:
	mov a, #0							; 2
	call XMS_ProcessMainEffect			; 8+256
										; 
	mov a, (xms_channels_other+coso_ndelay)+x
	bne _xmsp_ndo2
	call XMS_CHANNEL_ENV_ROUTINE		; 8+628
										; 
	call XMS_CHANNEL_UPDATEDSP			; 8+752
										; 
_xmsp_ndo2:
	aslz (channel_bit)					; 4 shift channel bit			}
	
	clrc								; 2 Update counters				{ 18 cycles
	mov a, x							; 2
	adc a, #16							; 2
	mov x, a							; 2
	pop a								; 4
	dec a								; 2
	bne _xmsptb_loop					; 2/4 loop						}
	
_xmsp_tick_notbetween:					;------------------------------------------------------
	
	incz (xms_tick)						; 4 increment tick				{ 28 (if pattern delay)
	cmp (xms_tick), (xms_speed)			; 6
	beq _xmsp_tick_norestartj1			; 2/4		; REVERSE
	jmp _xmsp_tick_norestart
_xmsp_tick_norestartj1

	mov (xms_tick), #0					; 5
	cmp (xms_patt_delay), #0			; 5 check pattern delay
	beq _xmsp_no_delay2					; 2/4 pattern delay long branch	}
	
	b _xmsp_tick_norestart				; 4								{ 4 cycles }
_xmsp_no_delay2:
	mov a, (xms_patt_loop)				; 3 get pattern loop var		{ 7/9 cycles
	and a, #$F0							; 2 check pattern loop
	beq _xmsp_pattern_loop_no			; 2/4							}
	
	mov (xms_row), (xms_patt_loop_row)				; 5	rewind row		{ 24 cycles
	mov (xms_pattern_read), (xms_patt_loop_pos)		; 5
	mov (xms_pattern_read+1), (xms_patt_loop_pos+1)	; 5
	mov a, #0
	mov y, #0
	movw (xms_pattern_repeat+0), ya
	movw (xms_pattern_repeat+2), ya
	movw (xms_pattern_repeat+4), ya
	movw (xms_pattern_repeat+6), ya
	and (xms_patt_loop), #$0F						; 5
	b _xmsp_tick_norestart							; 4					}
	
_xmsp_pattern_loop_no:
	cmp (xms_patt_jump_enable), #1					; 5 check patt jump		{ 22 cycles
	bne _xmsp_noPjump								; 2/4
	mov (xms_patt_jump_enable), #0					; 5 disable
	mov a, (xms_patt_jump)							; 3 get jump index
	cmp a, (c_xmsoffset+xmso_len)					; 3
	bcc _xmsp_pattjump_valid						; 2/4
	mov a, #0										; 2						}
_xmsp_pattjump_valid:								;
	movz (xms_position), a							; 4 set position		{ 134 cycles
	mov y, a										; 2
	mov a, (c_xmsoffset + xmso_orders)+y					; 5
	call XMS_SetPattern								; 8+90 set pattern
	clrc											; 2
	movz a, (xms_patt_jump+1)						; 3 add offset...
	movz (xms_row), a								 ; 4
	adc (xms_pattern_read), (xms_patt_jump_offset)	  ; 6
	adc (xms_pattern_read+1), (xms_patt_jump_offset+1) ; 6
	mov a, #0
	movz (xms_patt_jump_offset), a
	movz (xms_patt_jump_offset+1), a
	b _xmsp_tick_restarted							  ; 4					}
_xmsp_noPjump:
	incz (xms_row)							; 4 increment row				{ 16 cycles
	beq _xmsp_tick_restart					; 2/4 branch if zero
	cmp (xms_row), (xms_pattern_length)		; 6 compare w/ patt length
	bcc _xmsp_tick_restarted				; 2/4 branch if <=
	beq _xmsp_tick_restarted				; 2/4							}
_xmsp_tick_restart:
	
	incz (xms_position)						; 4 increment position index	{ 119 cycles
	movz a, (xms_position)					; 3
	cmp a, (c_xmsoffset+xmso_len)			; 4 compare with length of song
	bcc _xmsp_notendofsong					; 2/4
	mov a, (c_xmsoffset+xmso_restart)
	mov (xms_position), a
_xmsp_notendofsong:							;
	mov y, a								; 2
	mov a, (c_xmsoffset + xmso_orders)+y			; 4 read order table
	call XMS_SetPattern						; 8+90 set pattern				}
_xmsp_tick_restarted:

XMS_NotePeek:								;[15 cycles] (inline)
	mov  y, #0								; 2
	mov  a, [xms_pattern_read]+y			; 6
	orz a, (xms_peek)						; 3
	movz (xms_peek), a						; 4

_xmsp_tick_norestart:
	
	ret										; 5 return!!					{ 5 cycles }

;----------------------------------------------------------------------------------------------------
XMS_GetInstOffset:						;[49 cycles]
	; a = inst							; 
	dec a								; 2 int3 = inst_offset in table....		{ 20 cycles
	asl a								; 2
	mov y, c_xmsoffset+xmso_insto
	movz (int3), y
	mov y, c_xmsoffset+xmso_insto+1
	movz (int3+1), y								; EDITED AREA

	adcz a, (int3)
	
;	adcz a, (inst_offsets)				; 3 
	movz (int3), a						; 4
	mov a, #0							; 2

	adcz a, (int3+1)
;	adcz a, (inst_offsets+1)			; 3
	movz (int3+1), a					; 4										}
	
	mov y, #1							; 2 cur_inst = inst_offset...			{ 29 cycles	
	mov a, [int3]+y						; 6
	movz (cur_inst+1), a				; 4
	dec y								; 2
	mov a, [int3]+y						; 6
	movz (cur_inst), a					; 4
	ret									; 5										}

XMS_GetSampleHeader:					;[102 cycles]
	; x = channel buffer offset			; 
	movz a, (xms_channels+10)+x			; 4 get offset				{ 47 cycles
										;
	asl a								; 2

	mov y, c_xmsoffset+xmso_sampo+1
	mov (int1+1), y									; EDITED AREA 

	adc a, c_xmsoffset+xmso_sampo
	
;	adcz a, (samp_offsets)				; 3
	movz (int1), a						; 4
;	mov (int1+1), (samp_offsets+1)		; 5
	adc (int1+1), #0					; 5
	mov y, #1							; 2
	mov a, [int1]+y						; 6
	movz (int4+1), a					; 4
	dec y								; 2
	mov a, [int1]+y						; 6
	movz (int4), a						; 4							}
										;
	mov a, [int4]+y						; 6							{ 55 cycles
	movz (xms_samp_head+xsh_ft), a
	
	mov y, #1							; 2
	mov a, [int4]+y						; 6
	movz (xms_samp_head+xsh_vol), a		; 4
	inc y								; 2
	mov a, [int4]+y						; 6
	movz (xms_samp_head+xsh_pan), a		; 4
	inc y								; 2
	mov a, [int4]+y						; 6
	movz (xms_samp_head+xsh_note), a	; 4
	ret									; 5							}
	
XMS_GetPeriod:
	nop				; <-- MODIFICATION SPACE (BRANCH)
	nop				;

XMS_GetLinearPeriod:							; [57 cycles]  /
	; period = 7680 - realnote*64 - finetune/2	; ____________/
	; x = channel buffer offset					;/
	; return:								  ;
	; ya = period							;
	; x is preserved						;
	clrc									; 2						{ 23 cycles
	movz a, (xms_channels+7)+x				; 4 get note
	adcz a, (xms_samp_head+xsh_note)			; 3 add rel note
											;
	bcc _xmsgp_nc							; 2/4 clamp to range
	cmp a, #0								; 2
	bpl _xmsgp_nc							; 2/4
	mov a, #0								; 2
_xmsgp_nc:									;
	cmp a, #96								; 2
	bcc _xmsgp_a_96							; 2/4
	mov a, #95								; 2						}
_xmsgp_a_96:								;
											;
	mov y, #64								; 2 get note*64			{ 24 cycles
	mul ya									; 9
	movw (int1), ya							; 4
	mov a, #$00								; 2 get 7680-note*64
	mov y, #$1E								; 2
	subw ya, (int1)							; 5						}

	subw ya, (xms_samp_head+xsh_ft)			; 5 subtract finetune   { 10 cycles
	ret										; 5						}

XMS_GetAmigaPeriod:
	; period = AFT[ (note % 12)*32 + finetune ] >> note \ 12
	
	clrc													; 2		{ 23 cycles   get note
	adcz a, (xms_samp_head+xsh_note)						; 3
	
	bcc _xmsgap_nc											; 2/4
	cmp a, #0												; 2
	bpl _xmsgap_nc											; 2/4
	mov a, #0												; 2
_xmsgap_nc:													; 
	cmp a, #96												; 2
	bcc _xmsgap_a_96										; 2/4
	mov a, #95												; 2
_xmsgap_a_96:												;		}
	
	mov y, #0												; 2		{ 28 cycles   divide
	push x													; 4
	mov x, #12												; 2
	div ya, x												; 12
	pop x													; 4
	; a = note\12											; 
	; y = note%12 (0-11)									; 
															; 
	push a													; 4		}
	
	mov a, #64 ; "ya = y << 6"								; 2		{ 48 cycles   fetch value
	mul ya     ;											; 9
	
	
	addw ya, (xms_samp_head+xsh_ft)	; ADD $0300 TO FINETUNE!; 5
	

	movw (int1), ya											; 4
	mov y, #0												; 2
	mov a, [int1]+y											; 6
	movz (int3), a											; 4
	inc y													; 2
	mov a, [int1]+y											; 6
	pop y													; 4		}
	
_xmsgap_shift:
	lsr a													; 2		{ 12 cycles   shift		{ VARIABLE LOOP, 84 cycles max }
	rorz (int3)												; 4
	dec y													; 2
	bne _xmsgap_shift										; 2/4	}
	mov y, a												; 2		{ 10 cycles   return
	movz a, (int3)											; 3

	ret														; 5		}

;--------------------------------------------------------------------------------------
; Volume Effect Table
table_veffect_first:
.word	XMS_VFX_UNUSED			; 0 Do Nothing
.word	XMS_VFX_VOLUMEf			; 1 Set Volume 0-15
.word	XMS_VFX_VOLUMEf			; 2 Set Volume 16-31
.word	XMS_VFX_VOLUMEf			; 3 Set Volume 32-47
.word	XMS_VFX_VOLUMEf			; 4 Set Volume 48-63
.word	XMS_VFX_VOLUMEf			; 5 Set Volume 64
.word	XMS_VFX_UNUSED			; 6 Volume Slide Down
.word	XMS_VFX_UNUSED			; 7 Volume Slide Up
.word	XMS_VFX_FINEVOLDf		; 8 Fine Volume Slide Down
.word	XMS_VFX_FINEVOLUf		; 9 Fine Volume Slide Up
.word	XMS_VFX_VIBSPEEDf		; A Vibrato (set speed)
.word	XMS_VFX_VIBDEPTHf		; B Vibrato (set depth)
.word	XMS_VFX_PANNINGf		; C Set Panning
.word	XMS_VFX_UNUSED			; D Panning Slide Left
.word	XMS_VFX_UNUSED			; E Panning Slide Right
.word	XMS_VFX_GLISSANDOf		; F Glissando

table_veffect_other:
.word	XMS_VFX_UNUSED
.word	XMS_VFX_VOLUMEo
.word	XMS_VFX_VOLUMEo
.word	XMS_VFX_VOLUMEo
.word	XMS_VFX_VOLUMEo
.word	XMS_VFX_VOLUMEo
.word	XMS_VFX_VOLSLIDEDo
.word	XMS_VFX_VOLSLIDEUo
.word	XMS_VFX_UNUSED
.word	XMS_VFX_UNUSED
.word	XMS_VFX_VIBSPEEDo
.word	XMS_VFX_VIBSPEEDo
.word	XMS_VFX_PANNINGo
.word	XMS_VFX_PANSLIDELo
.word	XMS_VFX_PANSLIDERo
.word	XMS_VFX_GLISSANDOo

XMS_ProcessVolumeEffect:					;[112/240 first/other]
	; x = channel * 16						;
	; return:								;
	; x = preserved							;
	push x									; 4 preserve x						{ 11/13 cycles
	cmp (xms_tick), #0						; 5 check tick
	bne _xmspve_other_ticks					; 2/4								}
_xmspve_first_tick:							;
	movz a, (xms_channels+3)+x				; 4 jump to effect routine (first)			{ 92 cycles
	xcn a									; 5
	and a, #15								; 2
	asl a									; 2
	mov x, a								; 2
	jmp [table_veffect_first+x]				; 6+71 (get maximum cycle effect)			}
_xmspve_other_ticks:						;
	movz a, (xms_channels+3)+x				; 4 jump to effect routine (other)			{ 220 cycles
	xcn a									; 5
	and a, #15								; 2
	asl a									; 2
	mov x, a								; 2
	jmp [table_veffect_other+x]				; 6+199 (maximum cycle effect [vibrato])	}
_xmspve_exit:								;
	pop x									; 4									{ 9 cycles
	ret										; 5									}

XMS_VFX_UNUSED					=_xmspve_exit

; VOLUME EFFECT 1-5 : SET VOLUME --------------------------------------------------------------------------------------
XMS_VFX_VOLUMEf:							; [34 cycles]
	pop x									; 4
	push x									; 4
	movz a, (xms_channels+3)+x				; 4 read value
	setc									; 2
	sbc a, #$10								; 2 get $10-value
	movz (xms_channels+6)+x, a				; 5 store
	movz (xms_cc_volume), a					; 4 store
	or   (xms_cc_flags), #ccflag_volume		; 5 set flag
	
XMS_VFX_VOLUMEo:							; 
	b _xmspve_exit							; 4

#ifdef VFX_INCLUDE_EXTENDED

; VOLUME EFFECT 6 : VOLUME SLIDE DOWN -----------------------------------------------------------------------------------
;XMS_VFX_VOLSLIDEDf:
;	b _xmspve_exit
XMS_VFX_VOLSLIDEDo:							;[59 cycles]
	pop x									; 4 get chan
	push x									; 4
	movz a, (xms_channels+3)+x				; 4 read value
	and a, #15								; 2 and
	call XMS_CHANNEL_VOLSLIDEd				; 8+33 slide
	b _xmspve_exit							; 4

; VOLUME EFFECT 7 : VOLUME SLIDE UP -----------------------------------------------------------------------------------
;XMS_VFX_VOLSLIDEUf:
;	b _xmspve_exit
XMS_VFX_VOLSLIDEUo:							;[57 cycles]
	pop x									; 4 get chan
	push x									; 4
	movz a, (xms_channels+3)+x				; 4 read value
	and a, #15								; 2 and
	call XMS_CHANNEL_VOLSLIDEu				; 8+31 slide
	b _xmspve_exit							; 4

; VOLUME EFFECT 8 : FINE VOLUME SLIDE DOWN -----------------------------------------------------------------------------------
XMS_VFX_FINEVOLDf:							;[59 cycles]
	pop x									; 4 get chan
	push x									; 4
	movz a, (xms_channels+3)+x				; 4 read value
	and a, #15								; 2 and
	call XMS_CHANNEL_VOLSLIDEd				; 8+33 slide
XMS_VFX_FINEVOLDo:							;
	b _xmspve_exit							; 4

; VOLUME EFFECT 9 : FINE VOLUME SLIDE UP -----------------------------------------------------------------------------------
XMS_VFX_FINEVOLUf:							;[53 cycles]
	pop x									; 4 get chan
	push x									; 4
	movz a, (xms_channels+3)+x				; 4 read value
	and a, #15								; 2 and
	call XMS_CHANNEL_VOLSLIDEu				; 8+31 slide
XMS_VFX_FINEVOLUo:							;
	b _xmspve_exit							; 4

; VOLUME EFFECT A : VIBRATO SPEED -----------------------------------------------------------------------------------
XMS_VFX_VIBSPEEDf:							;[71 cycles]
	pop x									; 4 get chan
	push x									; 4
	movz a, (xms_channels+3)+x				; 4 read value
	and a, #15								; 2
	xcn a									; 5
	movz (int1), a							; 4
	mov a, (XMS_FXP_PREVIOUS+3)+x			; 5 set prev value
	and a, #15								; 2
	orz a, (int1)							; 3
	mov (XMS_FXP_PREVIOUS+3)+x, a			; 6
	
_xmsvfx_vs_entry2:									;[32 cycles]
	movz a, (xms_fxfirst)							; 3
	and a, #1										; 2
	bne _xmsvfx_vs_exit								; 2/4
	movz a, (xms_channels+4)+x						; 4
	cmp a, #4										; 2
	beq _xmsvfx_vs_exit								; 2/4 skip if main effect is vibrato also

	mov a, (xms_channels_other+coso_pa+1)+x		; 5			COPIED FROM VIBRATOf ROUTINE
	or a, #1									; 2
	mov (xms_channels_other+coso_pa+1)+x, a		; 6
	jmp _xmspve_exit							; 4
XMS_VFX_VIBSPEEDo:									;[195 cycles]
	pop x											; 4
	push x											; 4
	movz a, (xms_channels+4)+x						; 4
	cmp a, #4										; 2
	beq _xmsvfx_vs_exit								; 2/4 skip if main effect is vibrato also
	mov (fx_return), #(_xmspve_exit & 255)			; 5
	mov (fx_return+1), #((_xmspve_exit >> 8) & 255)	; 5
	jmp XMS_FX_VIBRATOo								; 3+166
_xmsvfx_vs_exit:
	jmp _xmspve_exit
	
; VOLUME EFFECT B : VIBRATO DEPTH -----------------------------------------------------------------------------------
XMS_VFX_VIBDEPTHf:									;[70 cycles]
	pop x											; 4
	push x											; 4
	movz a, (xms_channels+3)+x						; 4
	and a, #15										; 2
	movz (int1), a									; 4
	mov a, (XMS_FXP_PREVIOUS+3)+x					; 5
	and a, #$F0										; 2
	orz a, (int1)									; 3
	mov (XMS_FXP_PREVIOUS+3)+x, a					; 6
	b _xmsvfx_vs_entry2								; 4+32
;XMS_VFX_VIBDEPTHo:									;[199 cycles]
;	b XMS_VFX_VIBSPEEDo								; 4+195

; VOLUME EFFECT C : SET PANNING -----------------------------------------------------------------------------------
XMS_VFX_PANNINGf:									;[39 cycles]
	pop x											; 4 get channel
	push x											; 4
	movz a, (xms_channels+3)+x						; 4 read value
	and a, #15										; 2 and..
	; a = pan (0-15)								;
	; expand to 0-60								;
	asl a											; 2
	asl a											; 2
	cmp a, #33										; 2
	bcc _xms_vfxp_nofix								; 2/4
	adc a, #3										; 2
_xms_vfxp_nofix:									;
	movz (xms_channels+9)+x, a						; 5 store..
	movz (xms_cc_panning), a						; 4
	or (xms_cc_flags), #ccflag_panning				; 5
XMS_VFX_PANNINGo:									;
	jmp _xmspve_exit								; 3

; VOLUME EFFECT D : PANNING SLIDE LEFT -----------------------------------------------------------------------------------
;XMS_VFX_PANSLIDELf:								;
;	jmp _xmspve_exit								;
XMS_VFX_PANSLIDELo:									;[48 cycles]
	pop x											; 4 get channel
	push x											; 4
	movz a, (xms_channels+3)+x						; 4 read value
	and a, #15										; 2 and..
	movz (int1), a									; 4 save..
	movz a, (xms_channels+9)+x						; 4 get panning
	setc											; 2
	sbcz a, (int1)									; 3 subtract
	bpl _xmsvfxpsl_noclamp							; 2/4
	mov a, #0										; 2 clamp
_xmsvfxpsl_noclamp:									; 
	movz (xms_channels+9)+x, a						; 5 store
	movz (xms_cc_panning), a						; 4
	or (xms_cc_flags), #ccflag_panning				; 5
	jmp _xmspve_exit								; 3

; VOLUME EFFECT E : PANNING SLIDE RIGHT -----------------------------------------------------------------------------------
;XMS_VFX_PANSLIDERf:
;	jmp _xmspve_exit
XMS_VFX_PANSLIDERo:									;[50 cycles]
	pop x											; 4 almost the same operation as slide left
	push x											; 4
	movz a, (xms_channels+3)+x						; 4
	and a, #15										; 2
	movz (int1), a									; 4
	movz a, (xms_channels+9)+x						; 4
	clrc											; 2
	adcz a, (int1)									; 3
	cmp a, #65										; 2
	bcc _xmsvfxpsr_noclamp							; 2/4
	mov a, #64										; 2
_xmsvfxpsr_noclamp:									;
	movz (xms_channels+9)+x, a						; 5
	movz (xms_cc_panning), a						; 4
	or (xms_cc_flags), #ccflag_panning				; 5
	jmp _xmspve_exit								; 3

; VOLUME EFFECT F : GLISSANDO ---------------------------------------------------------------------------------------------
XMS_VFX_GLISSANDOf:									;[27 cycles]
	pop x											; 4 get chan
	push x											; 4
	movz a, (xms_channels+3)+x						; 4 get value
	and a, #15										; 2
	xcn a											; 5
	movz (xms_channels+12)+x, a						; 5 store for later
	jmp _xmspve_exit								; 3
XMS_VFX_GLISSANDOo:									;[?? cycles]
	pop x											; 4 get chan
	push x											; 4
	movz a, (xms_channels+4)+x						; 4 check main effect
	cmp a, #3										; 2
	beq _xmsvfx_g_exit								; 2/4 skip if main effect is glissando also
	mov (fx_return), #(_xmspve_exit & 255)			; 5
	mov (fx_return+1), #((_xmspve_exit >> 8) & 255)	; 5
	jmp (XMS_FX_PORTA2NOTEo)						; 3+?? (add 2 to skip the push/pop x)
_xmsvfx_g_exit:										;
	jmp _xmspve_exit								; 3

#else

XMS_VFX_VOLSLIDEDo:
XMS_VFX_VOLSLIDEUo:
XMS_VFX_FINEVOLDf:
XMS_VFX_FINEVOLDo:
XMS_VFX_FINEVOLUf:
XMS_VFX_FINEVOLUo:
XMS_VFX_VIBSPEEDf:
XMS_VFX_VIBSPEEDo:
XMS_VFX_VIBDEPTHf:
XMS_VFX_PANNINGf:
XMS_VFX_PANNINGo:
XMS_VFX_PANSLIDELo:
XMS_VFX_PANSLIDERo:
XMS_VFX_GLISSANDOf:
XMS_VFX_GLISSANDOo:
	jmp _xmspve_exit

#endif
	
;XMS_VFX_UNUSED:
;	jmp _xmspve_exit
;---------------------------------------------------------------------------------------------------------------------------------------
; OMG XM MAIN EFFECTS

;-----------------------------------------------------------------------------------------------------------
; OTHER CHANNEL BUFFER, 256 bytes
;-----------------------------------------------------------------------------------------------------------
xms_channels_other:
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

;------------------------------------------------------------------------------------------------------------
;; PREVIOUS EFFECT PARAMETERS
;--------------------------------
XMS_FXP_PREVIOUS:
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	.byte 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

;---------------------------------

table_effect_first:
.word	XMS_FX_ARPEGGIOf		; 0xy Arpeggio
.word	XMS_FX_PORTAUPf			; 1xy Portamento Up
.word	XMS_FX_PORTADOWNf		; 2xy Portamento Down
.word	XMS_FX_PORTA2NOTEf		; 3xy Portamento to Note
.word	XMS_FX_VIBRATOf			; 4xy Vibrato
.word	XMS_FX_PORTASLIDEf		; 5xy Portamento + Volume Slide
.word	XMS_FX_VIBSLIDEf		; 6xy Vibrato + Volume Slide
.word	XMS_FX_TREMOLOf			; 7xy Tremolo
.word	XMS_FX_PANf				; 8xy Set Panning
.word	XMS_FX_SAMPOFFf			; 9xx Sample Offset
.word	XMS_FX_VOLUMESLIDEf		; Axy Volume Slide
.word	XMS_FX_PATTERNJUMPf		; Bxy Pattern Jump
.word	XMS_FX_SETVOLUMEf		; Cxy Set Volume
.word	XMS_FX_PATTERNBREAKf	; Dxy Pattern Break
.word	XMS_FX_EXTENDEDf		; Exy Extended Effects
.word	XMS_FX_SETSPEEDf		; Fxy Set Speed
.word	XMS_FX_SETGLOBALVOLf	; Gxy Set Global Volume
.word	XMS_FX_GLOBALVOLSLIDEf	; Hxy Global Volume Slide
.word	XMS_FX_UNUSED			; Ixy Unused
.word	XMS_FX_UNUSED			; Jxy Unused
.word	XMS_FX_KEYOFFf			; Kxy Keyoff
.word	XMS_FX_ENVELOPEPOSf		; Lxy Envelope Position
.word	XMS_FX_UNUSED			; Mxy Unused
.word	XMS_FX_UNUSED			; Nxy Unused
.word	XMS_FX_UNUSED			; Oxy Unused
.word	XMS_FX_PANSLIDEf		; Pxy Panning Slide
.word	XMS_FX_UNUSED			; Qxy Unused
.word	XMS_FX_RETRIGGERf		; Rxy Retrigger Note
.word	XMS_FX_UNUSED			; Sxy Unused
.word	XMS_FX_TREMORf			; Txy Tremor
.word	XMS_FX_UNUSED			; Uxy Unused
.word	XMS_FX_UNUSED			; Vxy Unused
.word	XMS_FX_UNUSED			; Wxy Unused
.word	XMS_FX_XFINEPORTAf		; Xxy Extra Fine Portamento
.word	XMS_FX_UNUSED			; Yxy Unused
.word	XMS_FX_UNUSED			; Zxy Unused

table_effect_other:
.word	XMS_FX_ARPEGGIOo
.word	XMS_FX_PORTAUPo
.word	XMS_FX_PORTADOWNo
.word	XMS_FX_PORTA2NOTEo
.word	XMS_FX_VIBRATOo
.word	XMS_FX_PORTASLIDEo
.word	XMS_FX_VIBSLIDEo
.word	XMS_FX_TREMOLOo
.word	XMS_FX_UNUSED
.word	XMS_FX_UNUSED
.word	XMS_FX_VOLUMESLIDEo
.word	XMS_FX_UNUSED
.word	XMS_FX_UNUSED
.word	XMS_FX_UNUSED
.word	XMS_FX_EXTENDEDo
.word	XMS_FX_UNUSED
.word	XMS_FX_UNUSED
.word	XMS_FX_GLOBALVOLSLIDEo
.word	XMS_FX_UNUSED
.word	XMS_FX_UNUSED
.word	XMS_FX_KEYOFFo
.word	XMS_FX_UNUSED
.word	XMS_FX_UNUSED
.word	XMS_FX_UNUSED
.word	XMS_FX_UNUSED
.word	XMS_FX_PANSLIDEo
.word	XMS_FX_UNUSED
.word	XMS_FX_RETRIGGERo
.word	XMS_FX_UNUSED
.word	XMS_FX_TREMORo
.word	XMS_FX_UNUSED
.word	XMS_FX_UNUSED
.word	XMS_FX_UNUSED
.word	XMS_FX_UNUSED
.word	XMS_FX_UNUSED
.word	XMS_FX_UNUSED

XMS_ProcessMainEffect:								;[217/256 first/other]
	; x = channel * 16								;
	; a = beginning of effect (first effect number or new note)
	; return
	; x = preserved
	; a = note enable, 0 if cancelled (by noteporta, fineporta, etc..)
	; ints0-4 clobbered
	push a											; 4			{ 21/23 cycles
	push x											; 4
	mov (fx_return), #(_xmspme_exit & 255)			 ; 5
	mov (fx_return+1), #((_xmspme_exit >> 8) & 255)	 ; 5
	cmp (xms_tick), #0								 ; 5
	bne _xmspme_other_ticks							; 2/4		}
_xmspme_first_tick:									; 
	movz a, (xms_channels+4)+x						; 4			{ 14+? cycles
	asl a											; 2
	mov x, a										; 2
	jmp [table_effect_first+x]						; 6+169		} <-- fineporta up
_xmspme_other_ticks:								; 
	movz a, (xms_channels+4)+x						; 4			{ 14+? cycles
	asl a											; 2
	mov x, a										; 2
	jmp [table_effect_other+x]						; 6+206		}
_xmspme_exit:										; 
	pop x											; 4			{ 13 cycles
	pop a											; 4
													; 
	ret												; 5			}
	
; must restore X in effect functions.

; --------------------------------------------------------------------------------------------------------------------
; MOD EFFECTS
; --------------------------------------------------------------------------------------------------------------------
	
; EFFECT 0xy ARPEGGIO ------------------------------------------------------------------------------------------------
XMS_FX_ARPEGGIOf:									;[24 cycles]
	pop x											; 4 get chan
	push x											; 4
	mov a, #1										; 2 reset counter
	movz (xms_channels+11)+x, a						; 5
	or (xms_cc_flags), #ccflag_pitch				; 5 set pitch flag
	b _xmspme_exit									; 4
XMS_FX_ARPEGGIOo:									;[103 cycles]
	pop x											; 4	get chan		{ 8 cycles
	push x											; 4					}
													;
	movz a, (xms_channels+11)+x						; 4	check counter	} 30 cycles
	cmp a, #0										; 2	do actions..	}
	beq _xmsfxa_0									; 2/4				}
	cmp a, #1										; 2					}
	beq _xmsfxa_1									; 2/4				} jump
	mov a, #0										; 2					x	|
	movz (xms_channels+11)+x, a						; 5					x	|
	movz a, (xms_channels+5)+x						; 4					x	|
	and a, #15										; 2					x	|
	b _xmsfxa_2										; 4					x	|
_xmsfxa_0:											;					x	|
	inc (xms_channels+11), x						; 5					x	|
	mov a, #0										; 2					x	|
	b _xmsfxa_2										; 4					x  /
_xmsfxa_1:											;					}_/
	inc (xms_channels+11), x						; 5					}
	movz a, (xms_channels+5)+x						; 4					}
	xcn a											; 5					}
	and a, #15										; 2					}
_xmsfxa_2:											;					}
													;

_xmsfxa_modspace:
	nop						; <--- modspace
	nop						;
	xcn a											; 5	multiply, 64	{ 26 cycles
	mov (int1+1), #0								; 5
	asl a											; 2
	rolz (int1+1)									; 4
	asl a											; 2
	rolz (int1+1)									; 4
	movz (int1), a									; 4					}
	
	setc											; 2	subtract from period	{ 39 cycles
	movz a, (xms_cc_period)							; 4
	sbcz a, (int1)									; 3
	push a											; 4
	movz a, (xms_cc_period+1)						; 4
	sbcz a, (int1+1)								; 3
	mov y, a										; 2
	pop a											; 4
													;
	movw (xms_cc_period), ya						; 4 set pitch
	or (xms_cc_flags), #ccflag_pitch				; 5 set flag
	b _xmspme_exit									; 4	exit					}

_xmsfxa_amiga:
	clrc
	adc a, (xms_channels+cso_note)+x
	call XMS_GetPeriod
	movw (xms_cc_period), ya
	or (xms_cc_flags), #ccflag_pitch
	b _xmspme_exit

; EFFECT 1xy PORTAMENTO UP ------------------------------------------------------------------------------------------------
XMS_FX_PORTAUPf:									;[34 cycles]
	pop x											; 4 get chan
	push x											; 4
	movz a, (xms_channels+5)+x						; 4 check parameter
	bne _xmsfxpu_isparam							; 2/4
	mov a, (XMS_FXP_PREVIOUS+0)+x					; 5
	movz (xms_channels+5)+x, a						; 5
_xmsfxpu_isparam:									; 
	mov (XMS_FXP_PREVIOUS+0)+x, a					; 6
	b _xmspme_exit									; 4
XMS_FX_PORTAUPo:									;[159 cycles]
	pop x											; 4 get chan
	push x											; 4
_xmsfxpu_entry2:									;
	movz a, (xms_channels+5)+x						; 4 get slide value
	mov (int2), #0									; 5
	asl a											; 2
	rolz (int2)										; 4
	asl a											; 2
	rolz (int2)										; 4
													;
	call XMS_CHANNEL_PITCH_SLIDEUP					; 8+118
													;
	jmp _xmspme_exit								; 4

; EFFECT 2xy PORTAMENTO DOWN ------------------------------------------------------------------------------------------------
XMS_FX_PORTADOWNf:									;[33 cycles]
	pop x											; 4 get chan
	push x											; 4
	movz a, (xms_channels+5)+x						; 4 check parameter
	bne _xmsfxpd_isparam							; 2/4
	mov a, (XMS_FXP_PREVIOUS+1)+x					; 5
	movz (xms_channels+5)+x, a						; 5
_xmsfxpd_isparam:									;
	mov (XMS_FXP_PREVIOUS+1)+x, a					; 6
	jmp _xmspme_exit								; 3
XMS_FX_PORTADOWNo:									;[120 cycles]
	pop x											; 4 get chan
	push x											; 4
_xmsfxpd_entry2:									;
	movz a, (xms_channels+5)+x						; 4 get slide value
	mov (int2), #0									; 5
	asl a											; 2
	rolz (int2)										; 4
	asl a											; 2
	rolz (int2)										; 4
													;
	call XMS_CHANNEL_PITCH_SLIDEDOWN				; 8+80
													;
	jmp _xmspme_exit								; 3

; EFFECT 3xy PORTAMENTO TO NOTE ------------------------------------------------------------------------------------------------
XMS_FX_PORTA2NOTEf:									;[43 cycles]
	pop x											; 4 get chan
	push x											; 4
	movz a, (xms_channels+5)+x						; 4 check parameter
	bne _xmsfxpn_isparam							; 2/4
	mov a, (XMS_FXP_PREVIOUS+2)+x					; 5
	movz (xms_channels+5)+x, a						; 5
_xmsfxpn_isparam:									;
	mov (XMS_FXP_PREVIOUS+2)+x, a					; 6
	movz (xms_channels+12)+x, a						; 5
	mov x, #0										; 2
	jmp [fx_return+x]								; 6
XMS_FX_PORTA2NOTEo:									;[]
	pop x											; 4 get chan
	push x											; 4
_xmsfxpn_entry2:									;
													;
	mov (int2), (xms_cc_period)						; 5 copy xms_cc_period
	mov (int2+1), (xms_cc_period+1)					; 5
													;
	movz a, (xms_channels+cso_note)+x				; 4
													;
	call XMS_GetPeriod								; 8+57
	; ya = wanted period							;
	; int2 = current period							;
													;
	push a											; 4 preserve period
	push y											; 4
	cmpw ya, (int2)									; 4 compare with new period
	bne _xmsfxpn_notequal							; 2/4
	pop y											;   4 old period=new period
	pop a											;   4 
	mov x, #0										;   2 exit
	jmp [fx_return+x]								;   6
_xmsfxpn_notequal:									;
	bcs _xmsfxpn_less								; 2/4 jump to more/less routine
	b _xmsfxpn_more									; 4

_xmsfxpn_less:										;
	; subtract from pitch (add to period)			;
	movz a, (xms_channels+12)+x						; 4 get porta speed			{ 90 cycles
	mov (int2), #0									; 5
	asl a											; 2 shift
	rolz (int2)										; 4
	asl a											; 2
	rolz (int2)										; 4
													;
	clrc											; 2 add to period
	movz (int1), a									; 4
	movz a, (xms_cc_period)							; 3
	adcz a, (int1)									; 3
	push a											; 4
	movz a, (xms_cc_period+1)						; 3
	adcz a, (int2)									; 3
	mov y, a										; 2
	pop a											; 4
													;
	movw (int2), ya									; 4 store
													;
	pop y											; 4 restore old period
	pop a											; 4
													;
	; check if it slid past wanted note				;
	cmpw ya, (int2)  ; wanted - current				; 4 compare
	bcs _xmsfxpn_less_continue						; 2/4
	movw (int2), ya									; 4
_xmsfxpn_less_continue:								;
													;
	movw ya, (int2)									; 5 save
	movz (xms_channels+0)+x, a						; 5
	movz (xms_channels+1)+x, y						; 5
													; 
	b _xmsfxpn_edited								; 4 skip					}
	
_xmsfxpn_more:
	
	; add to pitch (subtract from period)			;
	movz a, (xms_channels+12)+x						; 4	get porta speed			{ 78 cycles
	mov (int2), #0									; 5
	asl a											; 2
	rolz (int2)										; 4
	asl a											; 2
	rolz (int2)										; 4
													;
	setc											; 2 subtract from period
	movz (int1), a									; 4
	movz a, (xms_cc_period)							; 3
	sbcz a, (int1)									; 3
	push a											; 4
	movz a, (xms_cc_period+1)						; 3
	sbcz a, (int2)									; 3
	mov y, a										; 2
	pop a											; 4
													;
	movw (int2), ya									; 4 store
													;
	pop y											; 4 restore old period
	pop a											; 4
													; 
	; check if it slid past wanted note				; 
	cmp int2+1, #0
	bmi _xmsfxpn_more_stop
	cmpw ya, (int2) ; wanted - current				; 4 compare
	bcc _xmsfxpn_more_continue						; 2/4
_xmsfxpn_more_stop:
	movw (int2), ya									; 4
_xmsfxpn_more_continue:								;
	movw ya, (int2)									; 5 save
	movz (xms_channels+0)+x, a						; 5
	movz (xms_channels+1)+x, y						; 5							}

_xmsfxpn_edited:									;
	; set frequency									;
													;
	call XMS_CHANNEL_SETPITCH						; 8+59
	
_xmsfxpn_equal:										;
													;
	mov x, #0										; 2							{ 8 cycles
	jmp [fx_return+x]								; 6							}

; EFFECT 4xy VIBRATO ------------------------------------------------------------------------------------------------
XMS_FX_VIBRATOf:								;[89 cycles]
	pop x										; 4
	push x										; 4 get channel
	mov a, (XMS_FXP_PREVIOUS+3)+x				; 5 check/get previous parameter
	movz (int1), a								; 4 
	movz y, (xms_channels+5)+x					; 4
	mov a, y									; 2
	and a, #$0F									; 2
	beq _xmsfxv_noparam2						; 2/4
	and (int1), #$F0							;   5
_xmsfxv_noparam2:								; 
	mov a, y									; 2
	and a, #$F0									; 2
	beq _xmsfxv_noparam1						; 2/4
	and (int1), #$0F							;   5
_xmsfxv_noparam1:								; 
	mov a, y									; 2
	orz a, (int1)								; 3
	mov (XMS_FXP_PREVIOUS+3)+x, a				; 6
	movz (xms_channels+5)+x, a					; 5
												; 
	movz a, (xms_fxfirst)						; 3
	and a, #1									; 2
	beq _xmsfxv_entry2							; 2/4
	mov a, #0									; 2
	movz (xms_channels+14)+x, a					; 5
												; 
	b _xmsfxv_firstrun							; 4
_xmsfxv_entry2:									;[21 cycles]
	mov a, (xms_channels_other+coso_pa+1)+x		; 5
	or a, #1									; 2
	mov (xms_channels_other+coso_pa+1)+x, a		; 6
_xmsfxv_firstrun:								; 
	mov x, #0									; 2
	jmp [fx_return+x]							; 6
XMS_FX_VIBRATOo:								;[147 cycles]
	pop x										; 4 get chan		{ 32 cycles
	push x										; 4
	mov a, (xms_channels_other+6)+x				; 5 get param
	lsr a										; 2
	and a, #3									; 2
	push a										; 4
												;
	mov a, (XMS_FXP_PREVIOUS+3)+x				; 5 get prev data
	and a, #%11110000
	lsr a										; 2
	lsr a										; 2					}
												;----------------------------------------
;	clrc										; 2					{ 84 cycles
	adcz a, (xms_channels+14)+x					; 4 add sine pos
	movz (xms_channels+14)+x, a					; 5
_xmsfxv_getvalue:								;[115 cycles]
	; a = 0 to 255								;
	pop y										; 4 get table entry
	call XMS_GETVIBTABLE						; 46
	mov y, a									; 2
												;
	mov a, (XMS_FXP_PREVIOUS+3)+x				; 4
	and a, #15									; 2
												;
	asl a										; 2 
	asl a										; 2
	asl a										; 2
												;
	mul ya										; 9 mul by depth
	; ya = vibrato value						;					}
												;----------------------------------------
	mov a, y									; 2 store ya >> 8	{ 33 cycles
	mov (xms_channels_other+coso_pa)+x, a		; 6
	movz a, (xms_cc_flags)						; 3
	and a, #%10000000							; 2
	or a, #1									; 2
	mov (xms_channels_other+coso_pa+1)+x, a		; 6
												; 
	set1 (xms_cc_flags)							; 4 set flag
	mov x, #0									; 2
	jmp [fx_return+x]							; 6					}

; * change to bit operation

; EFFECT 5xy NOTEPORTA+VOLSLIDE ------------------------------------------------------------------------------------------------
XMS_FX_PORTASLIDEf:								;[30 cycles]
	pop x										; 4 get chan
	push x										; 4
	movz a, (xms_channels+5)+x					; 4 check&get prev param
	bne _xmsfxps_isparam						; 2/4
	mov a, (XMS_FXP_PREVIOUS+4)+x				; 5
	movz (xms_channels+5)+x, a					; 5
_xmsfxps_isparam:								;
	mov (XMS_FXP_PREVIOUS+4)+x, a				; 6
	jmp _xmspme_exit							 ; 3 ---------------
XMS_FX_PORTASLIDEo:								  ;[26+? cycles]
	mov fx_return, #(_xmsfxps_ret1 & 255)		   ; 5 set return
	mov fx_return+1, #((_xmsfxps_ret1 >> 8) & 255) ; 5
	jmp XMS_FX_PORTA2NOTEo						   ; 3+? call
_xmsfxps_ret1:									  ;
	mov fx_return, #(_xmspme_exit & 255)		  ; 5 set return
	mov fx_return+1, #((_xmspme_exit >> 8) & 255) ; 5
	jmp XMS_FX_VOLUMESLIDEo						  ; 3+? call
												 ;
; EFFECT 6xy VIBRATO+VOLSLIDE ------------------------------------------------------------------------------------------------
XMS_FX_VIBSLIDEf:								;[30 cycles]
	pop x										; 4 get chan
	push x										; 4
	movz a, (xms_channels+5)+x					; 4 check&get prev param
	bne _xmsfxvs_isparam						; 2/4
	mov a, (XMS_FXP_PREVIOUS+5)+x				; 5
	movz (xms_channels+5)+x, a					; 5
_xmsfxvs_isparam:								;
	mov (XMS_FXP_PREVIOUS+5)+x, a				; 6
	jmp _xmspme_exit							 ; 3 --------------
XMS_FX_VIBSLIDEo:								  ;[26+? cycles]
	mov fx_return, #(_xmsfxvs_ret1 & 255)		   ; 5 set return
	mov fx_return+1, #((_xmsfxvs_ret1 >> 8) & 255) ; 5
	jmp XMS_FX_VIBRATOo							   ; 3+? call
_xmsfxvs_ret1:									  ;
	mov fx_return, #(_xmspme_exit & 255)		  ; 5 set return
	mov fx_return+1, #((_xmspme_exit >> 8) & 255) ; 5
	jmp XMS_FX_VOLUMESLIDEo						  ; 3+? call
												 ;
; EFFECT 7xy TREMOLO ------------------------------------------------------------------------------------------------

#ifdef FX_INCLUDE_TREMOLO
XMS_FX_TREMOLOf:								;[118 cycles]
	pop x										; 4 get chan				{ 28 cycles
	push x										; 4
	movz a, (xms_fxfirst)						; 3 check..
	and a, #1									; 2
	beq _xmsfxt_exit							; 2/4
	mov a, #0									; 2 reset counter on first time
	movz (xms_channels+11)+x, a					; 5
	mov  (xms_channels_other+coso_vadd)+x, a	; 6							}
_xmsfxt_exit:									; 
	mov a, (XMS_FXP_PREVIOUS+6)+x				; 5 check prev data			{ 51 cycles
	movz (int1), a								; 4
	movz y, (xms_channels+5)+x					; 4
	mov a, y									; 2
	and a, #$0F									; 2
	beq _xmsfxt_noparam2						; 2/4
	and (int1), #$F0							; 5
_xmsfxt_noparam2:								; 
	mov a, y									; 2
	and a, #$F0									; 2
	beq _xmsfxt_noparam1						; 2/4
	and (int1), #$0F							; 5
_xmsfxt_noparam1:								;
	mov a, y									; 2
	orz a, (int1)								; 3
	mov (XMS_FXP_PREVIOUS+6)+x, a				; 6
	movz (xms_channels+5)+x, a					; 5							}
												;
	
	mov a, (xms_channels_other+coso_vadd)+x		; 5							{ 39 cycles
	adcz a, (xms_channels+6)+x					; 4
	b _xmsfxt_setvalue							; 4+26						}
XMS_FX_TREMOLOo:								;[180 cycles]
	pop x										; 4				{ 44 cycles
	push x										; 4
												; 
	mov a, (xms_channels_other+6)+x				; 5
	xcn a										; 5
	and a, #3									; 2
	push a										; 4
												; 
	movz a, (xms_channels+5)+x					; 4
	lsr a										; 2
	lsr a										; 2
	and a, #%00111100							; 2
												; 
	clrc										; 2
	adcz a, (xms_channels+11)+x					; 4
	movz (xms_channels+11)+x, a					; 4				}
_xmsfxt_getvalue:								;[136 cycles]
	
	pop y										; 4 get table entry		{ 110 cycles
	call XMS_GETVIBTABLE						; 46
	
	mov y, a									; 2
	movz a, (xms_channels+5)+x					; 4
	and a, #15									; 2
	mul ya										; 9
	; ya = tremolo value						; 
	push x										; 4
	mov x, #64									; 2
	div ya, x									; 12
	pop x										; 4						
												; 
	mov1 c, (xms_cc_flags+(7<<13))				; 4
	bcs _xmsfxt_add_val							; 2/4
	eor a, #255									; 2
	inc a										; 2
_xmsfxt_add_val:								;
	clrc										; 2
	mov (xms_channels_other+coso_vadd)+x, a		; 6
	adcz a, (xms_channels+6)+x					; 3						}
_xmsfxt_setvalue:								;[26 cycles]
	
	cmp a, #65									; 2 clamp to 0-64		{ 26 cycles
	bcc _xmsfxt_clip_good						; 2/4
	cmp a, #128									; 2
	bcs _xmsfxt_gt								; 2/4
	mov a, #64									; 2
	b _xmsfxt_clip_good							; 4
_xmsfxt_gt:										; 
	mov a, #0									; 2
_xmsfxt_clip_good:								; 
	; a = channel volume						; 
	movz (xms_cc_volume), a						; 4 set volume
	or (xms_cc_flags), #ccflag_volume			; 5 set flag
												; 
	jmp _xmspme_exit							; 3						}
#else

XMS_FX_TREMOLOf:
XMS_FX_TREMOLOo:
	jmp _xmspme_exit

#endif

; EFFECT 8xy SET PANNING ------------------------------------------------------------------------------------------------
XMS_FX_PANf:									;[33 cycles]
	; set pan!									; 
	pop x										; 4 get chan
	push x										; 4
	movz a, (xms_channels+5)+x					; 4 get param
	lsr a										; 2 shift
	lsr a										; 2
	movz (xms_channels+9)+x, a					; 5 set
	movz (xms_cc_panning), a					; 4
	or (xms_cc_flags), #ccflag_panning			; 5
	jmp _xmspme_exit							; 3 ret
;XMS_FX_PANo:									; 
;	; do nothing								; 
;	jmp _xmspme_exit							; 

; EFFECT 9xx SAMPLE OFFSET ------------------------------------------------------------------------------------------------
XMS_FX_SAMPOFFf:								;[24 cycles]
	pop x										; 4 get chan
	push x										; 4
	movz a, (xms_channels+5)+x					; 4 get param
	movz (xms_cc_sampoff), a					; 4 store
	or (xms_cc_flags), #ccflag_offset			; 5 set flag
XMS_FX_SAMPOFFo:								; 
	jmp _xmspme_exit							; 3
												;
; EFFECT Axy VOLUME SLIDE ------------------------------------------------------------------------------------------------
XMS_FX_VOLUMESLIDEf:							;[38 cycles]
	pop x										; 4 get chan
	push x										; 4
	movz a, (xms_channels+5)+x					; 4 check prev data
	bne _xmsfxvols_isparam						; 2/4
	mov a, (XMS_FXP_PREVIOUS+7)+x				; 5
	movz (xms_channels+5)+x, a					; 5
_xmsfxvols_isparam:								; 
	mov (XMS_FXP_PREVIOUS+7)+x, a				; 6
	mov x, #0									; 2
	jmp [fx_return+x]							; 6 -------------
XMS_FX_VOLUMESLIDEo:							;[61 cycles]
	pop x										; 4 get chan				{ 14 cycles
	push x										; 4
	movz a, (xms_channels+5)+x					; 4 get param
	cmp a, #16									; 2 check slide direction	}
	bcs _xmsfxvs_hiparam						; 2/4
	; loparam									; 
	and a, #$0F									; 2			{ 28 cycles  + nojump(2)
	; subtract from volume						; 
	setc										; 2
	movz (int1), a								; 4
	movz a, (xms_channels+6)+x					; 4 get volume
	sbcz a, (int1)								; 3 slide
	bcs _xmsfxvs_lo_over						; 2/4
	mov a, #0									; 2 clip
_xmsfxvs_lo_over:								; 
	movz (xms_channels+6)+x, a					; 5 store
	b _xmsfxvs_finish							; 4			}
_xmsfxvs_hiparam:								; 
	and a, #$F0									; 2			{ 24 cycles  + jump(4)
	xcn a										; 5
	; add to volume								; 
	clrc										; 2
	adcz a, (xms_channels+6)+x					; 4 slide volume
	cmp a, #64									; 2 clip
	bcc _xmsfxvs_hi_over						; 2/4
	mov a, #64									; 2
_xmsfxvs_hi_over:								;
	movz (xms_channels+6)+x, a					; 5	store	}
_xmsfxvs_finish:								;
												;
	movz (xms_cc_volume), a						; 4			{ 17 cycles
	or (xms_cc_flags), #ccflag_volume			; 5
												;
	mov x, #0									; 2
	jmp [fx_return+x]							; 6			}

; EFFECT Bxy PATTERN JUMP ------------------------------------------------------------------------------------------------
XMS_FX_PATTERNJUMPf:							;[24 cycles]
	pop x										; 4 get chan
	push x										; 4
	movz a, (xms_channels+5)+x					; 4 get param
	movz (xms_patt_jump), a						; 4 set patt jump params
	mov (xms_patt_jump_enable), #1				; 5
XMS_FX_PATTERNJUMPo:							;
	jmp _xmspme_exit							; 3

; EFFECT Cxy SET VOLUME ---------------------------------------------------------------------------------------------------
XMS_FX_SETVOLUMEf:								;[40 cycles]
	pop x										; 4 get chan
	push x										; 4
	movz a, (xms_channels+5)+x					; 4 get param
	cmp a, #64									; 2 check
	bcc _xmsfxsv_lt_64							; 2/4
	mov a, #64									; 2 set to 64 if greater than
_xmsfxsv_lt_64:
	movz (xms_channels+6)+x, a					; 5 set volume
	movz (xms_cc_volume), a						; 4
	or (xms_cc_flags), #%ccflag_volume			; 5 &flag
XMS_FX_SETVOLUMEo:
	mov x, #0									; 2 return
	jmp [fx_return+x]							; 6

; EFFECT Dxy PATTERN BREAK ------------------------------------------------------------------------------------------------
XMS_FX_PATTERNBREAKf:							;[193 cycles]	(5 loops)
	movz a, (xms_position)						; 4				{ 33 cycles
	inc a										; 2
	; this function WILL crash if the xms does not have a proper table
	
	movz (xms_patt_jump), a						; 4 omg, will break on pattern 255
	mov (xms_patt_jump_enable), #1				; 5 enable jump
	
	pop x										; 4 get chan
	push x										; 4
	movz a, (xms_channels+cso_fxparam)+x		; 4
	
	movz (xms_patt_jump+1), a					; 4
	
;	cmp a, #0									; 2				}
	bne _xmsfx_pb_notzero						; 2/4			{ 14 cycles
;	mov a, #0									; 2 break to zero is much faster
	mov y, a									; 2
	movw (xms_patt_jump_offset), ya				; 4
	jmp _xmspme_exit							; 3
_xmsfx_pb_notzero:								;
	
	movz y, (xms_pattern_plus)
	mov a, [xms_pattern_read]+y

	bne _xmsfx_pb_startsearch
	b   _xmsfx_pb_instant
_xmsfx_pb_startsearch:
												;
	movz y, (xms_patt_jump)						; 3
	mov a, (c_xmsoffset+xmso_orders)+y					; 5								JUMP WILL NOT BE 3, FIX PLEASE
	movz y, (xms_pattern_plus)					; ?				}
	inc y
	
_xmsfx_pb_search:
	cmpi a, [xms_pattern_read]+y				; 6	compare index				{ 5 misses, 146 cycles
	bne _xmsfx_pb_next							; 2/4 branch to next if not match
_xmsfx_pb_instant:
	inc y											; 2			{ 46 cycles
	mov a, [xms_pattern_read]+y						; 6
	
	movz (xms_patt_jump_offset), a					; 4
	inc y											; 2
	mov a, [xms_pattern_read]+y						; 6
	movz (xms_patt_jump_offset+1), a				; 4
	
	movz y, (xms_pattern_plus)						; 2
	mov a, [xms_pattern_read]+y						; 6
	beq _xmsfx_pb_fexit
	clrc											; 2
	adcz a, (xms_pattern_read)						; 3
	movz (xms_pattern_read), a						; 4
	adc (xms_pattern_read+1), #0					; 5			}
	jmp _xmspme_exit
_xmsfx_pb_fexit:
	clrc
	mov a, #3
	adcz a, (xms_pattern_read)
	movz (xms_pattern_read), a
	adc (xms_pattern_read+1), #0
	jmp _xmspme_exit
	
_xmsfx_pb_next:
	inc y										; 2		{ 9 cycles
	inc y										; 2
	inc y										; 2
	jmp _xmsfx_pb_search						; 3		}						}
_xmsfx_pb_match:
;XMS_FX_PATTERNBREAKo:
;	jmp _xmspme_exit

; EFFECT Fxy SET SPEED ----------------------------------------------------------------------------------------------------
XMS_FX_SETSPEEDf:								;[150 cycles]
	pop x										; 4 get chan
	push x										; 4
	movz a, (xms_channels+5)+x					; 4 read param
	cmp a, #32									; 2 check speed/bpm
	bcs _xmsfxss_bpm							; 2/4
	; set speed									; 
	movz (xms_speed), a							; 4 set speed
	jmp _xmspme_exit							; 3
_xmsfxss_bpm:									; or..
	; set bpm									; 
	call XMS_SetBPM								; 8+124 set bpm
												; 
	; uses jump from other ticks!				; 
XMS_FX_SETSPEEDo:								; 
	jmp _xmspme_exit							; 3
; -------------------------------------------------------------------------------------------------------------------------
; EFFECT Exy EXTENDED EFFECTS ---------------------------------------------------------------------------------------------
; -------------------------------------------------------------------------------------------------------------------------

; Extended Effect Table
table_effect_ex_first:
.word	XMS_FX_EX_SETFILTERf		; E0y Set Filter		( XMSNES EXCLUSIVE )
.word	XMS_FX_EX_FINEPORTAUPf		; E1y Fine Porta Up
.word	XMS_FX_EX_FINEPORTADOWNf	; E2y Fine Porta Down
.word	XMS_FX_EX_GLISSANDOCTRLf	; E3y Glissando Control	( NOT IMPLEMENTED )
.word	XMS_FX_EX_VIBRATOFORMf		; E4y Vibrato Waveform
.word	XMS_FX_EX_SETFINETUNEf		; E5y Set Finetune		( NOT IMPLEMENTED )
.word	XMS_FX_EX_PATTERNLOOPf		; E6y Pattern Loop
.word	XMS_FX_EX_TREMOLOFORMf		; E7y Tremolo Waveform
.word	XMS_FX_EX_SETPANNINGf		; E8y Set Panning
.word	XMS_FX_EX_RETRIGGERf		; E9y Retrigger Note
.word	XMS_FX_EX_FINEVOLUPf		; EAy Fine Volume Slide Up
.word	XMS_FX_EX_FINEVOLDOWNf		; EBy Fine Volume Slide Down 
.word	XMS_FX_EX_NOTECUTf			; ECy Note Cut
.word	XMS_FX_EX_NOTEDELAYf		; EDy Note Delay
.word	XMS_FX_EX_PATTERNDELAYf		; EEy Pattern Delay
.word	XMS_FX_EX_SENDMESSAGEf		; EFy Send Message		( XMSNES EXCLUSIVE )

table_effect_ex_other:
.word	XMS_FX_EX_SETFILTERo
.word	XMS_FX_EX_FINEPORTAUPo
.word	XMS_FX_EX_FINEPORTADOWNo
.word	XMS_FX_EX_GLISSANDOCTRLo
.word	XMS_FX_EX_VIBRATOFORMo
.word	XMS_FX_EX_SETFINETUNEo
.word	XMS_FX_EX_PATTERNLOOPo
.word	XMS_FX_EX_TREMOLOFORMo
.word	XMS_FX_EX_SETPANNINGo
.word	XMS_FX_EX_RETRIGGERo
.word	XMS_FX_EX_FINEVOLUPo
.word	XMS_FX_EX_FINEVOLDOWNo
.word	XMS_FX_EX_NOTECUTo
.word	XMS_FX_EX_NOTEDELAYo
.word	XMS_FX_EX_PATTERNDELAYo
.word	XMS_FX_EX_SENDMESSAGEo

XMS_FX_EXTENDEDf:					;[29+? cycles]
	pop x							; 4 get chan
	push x							; 4
	movz a, (xms_channels+5)+x		; 4 get param
	xcn a							; 5 swap
	and a, #15						; 2 a = (param & hi) >> 4
	asl a							; 2 *2
	mov x, a						; 2 move
	jmp [table_effect_ex_first+x]	; 6+? jump to routine
XMS_FX_EXTENDEDo:					;[29+? cycles]
	pop x							; 4
	push x							; 4
	movz a, (xms_channels+5)+x		; 4
	xcn a							; 5
	and a, #15						; 2
	asl a							; 2
	mov x, a						; 2
	jmp [table_effect_ex_other+x]	; 6+?

; EFFECT E0y SET FILTER ------------------------------------------------------------------------
XMS_FX_EX_SETFILTERf:					;[349 cycles]
	pop x								; 4	get chan		{ 24 cycles
	push x								; 4
	movz a, (xms_channels+5)+x			; 4
	cmp a, #1							; 2
	beq _xmsfxxsf_on					; 2/4
	cmp a, #2							; 2
	beq _xmsfxxsf_off					; 2/4
	bcs _xmsfxxsf_set					; 2/4				} jump..
	jmp _xmspme_exit					; 3
_xmsfxxsf_on:							;
	mov ($F2), #DSP_EON					; 5 enable echo
	or  ($F3), (channel_bit)			; 6
	b XMS_FX_EX_SETFILTERo				; 4+3
_xmsfxxsf_off:							; 
	mov ($F2), #DSP_EON					; 5 disable echo
	movz a, (channel_bit)				; 3
	eor a, #255							; 2
	andz a, ($F3)						; 3
	movz ($F3), a						; 4
	b XMS_FX_EX_SETFILTERo				; 4+3
_xmsfxxsf_set:
	; a = filter + 3					;

	mov ($F2), #DSP_EDL					; RESET EDL
	mov ($F3), #0						;
	mov $F2, #DSP_FLG					; DISABLE ECHO WRITE
	or  $F3, #%00100000					;

	sbc a, #3							; 2 carry was set with cmp earlier		{ 47 cycles
	mov y, #12							; 2
	mul ya								; 9 mul (filter*12)
	movz (int2), a						; 4
	mov a, (c_xmsoffset + xmso_foff+1)	; 4 add filter table offset
	clrc								; 2 ...
	adc a, #c_xmsoffseth				; 2 ...
	movz (int1+1), a					; 4 ...
	mov a, (c_xmsoffset + xmso_foff)	; 4 ...
	adcz a, (int2)						; 3 ...
	adc (int1+1), #0					; 5 ...
	movz (int1), a						; 4 ...
	mov y, #0							; 2	...									}
	
	mov  a, [int1]+y					; 6	volume L							{ 17 cycles
	inc  y								; 2
	mov  ($F2), #DSP_EVOLL				; 5
	movz ($F3), a						; 4										}
	
	mov  a, [int1]+y					; 6 volume R							{ 17 cycles
	inc  y								; 2
	mov  ($F2), #DSP_EVOLR				; 5
	movz ($F3), a						; 4										}
										; 
	mov  ($F2), #DSP_C0					; 5 set filter coefficients...			{ 187 cycles
_xmsfxxsf_set_loop:						; 
	mov  a, [int1]+y					; 6
	inc  y								; 2
	movz ($F3), a						; 4
	adc	 ($F2), #$10					; 5 carry was cleared above
	cmp  y, #$A							; 2
	bne _xmsfxxsf_set_loop				; 2/4									}
										; 
	mov  a, [int1]+y					; 6	fetch EDL							{ 17 cycles
	inc  y								; 2
	push a
	
	asl a								; 2
	asl a								; 2
	asl a								; 2
	mov  ($F2), #DSP_ESA				; 5
	eor a, #255							; 2
	
	movz  ($F3), a						; 4
	mov a, (c_xmsoffset+xmso_freqmode)
	bne _xmsfxxsf_notamiga
	; make space for amiga table
	setc
	sbc ($F3), #16
_xmsfxxsf_notamiga:
	mov  ($F2), #DSP_EFB				; 5
	mov  a, [int1]+y
	movz ($F3), a						; 4
	
	pop a
	
	mov $F2, #DSP_EDL
	mov ($F3), a
	
	mov ($F2), #DSP_FLG					; enable echo
	mov ($F3), #%00000000				;								----------------------------------------------------- WILL MESS UP NOISE...
										; 
XMS_FX_EX_SETFILTERo:					; 
	jmp _xmspme_exit					; 3										}
	
; EFFECT E1y FINE PORTA UP -----------------------------------------------------------------------
XMS_FX_EX_FINEPORTAUPf:					;[140 cycles]
	pop x								; 4 get chan
	push x								; 4
	movz a, (xms_channels+5)+x			; 4 get param
	and a, #15							; 2
	bne _xmsfxxfpu_isparam				; 2/4 if zero, get previous
	mov a, (XMS_FXP_PREVIOUS+8)+x		; 5
	movz (xms_channels+5)+x, a			; 5
_xmsfxxfpu_isparam:						; 
	mov (XMS_FXP_PREVIOUS+8)+x, a		; 6
	and a, #15							; 2
	asl a								; 2
	asl a								; 2
	mov (int2), #0						; 5
	call XMS_CHANNEL_PITCH_SLIDEUP		; 8+86 slide..
										; 
	; uses jump from other ticks!		; 
XMS_FX_EX_FINEPORTAUPo:					; 
	jmp _xmspme_exit					; 3

; EFFECT E2y FINE PORTA DOWN -----------------------------------------------------------------------------------------------
XMS_FX_EX_FINEPORTADOWNf:				;[134 cycles]
	pop x								; 4 get chan
	push x								; 4
	movz a, (xms_channels+5)+x			; 4 get param
	and a, #15							; 2
	bne _xmsfxxfpd_isparam				; 2/4
	mov a, (XMS_FXP_PREVIOUS+9)+x		; 5
	movz (xms_channels+5)+x, a			; 5
_xmsfxxfpd_isparam:						; 
	mov (XMS_FXP_PREVIOUS+9)+x, a		; 6
	and a, #15							; 2
	asl a								; 2
	asl a								; 2
	mov (int2), #0						; 5
	call XMS_CHANNEL_PITCH_SLIDEDOWN	; 8+80 slide
										; 
	; uses jump from other ticks!		; 
XMS_FX_EX_FINEPORTADOWNo:				; 
	jmp _xmspme_exit					; 3

; EFFECT E3y GLISSANDO CONTROL -----------------------------------------------------------------------------------------------
XMS_FX_EX_GLISSANDOCTRLf:				;[3 cycles]
	; uses jump from other ticks!		;
XMS_FX_EX_GLISSANDOCTRLo:				;
	jmp _xmspme_exit					; 3

table_vib_conv:
.byte	0, 3, 1, 0

; EFFECT E4y VIBRATO WAVEFORM -----------------------------------------------------------------------------------------------
XMS_FX_EX_VIBRATOFORMf:					;[46 cycles]
	pop x								; 4 get chan
	push x								; 4
	movz a, (xms_channels+5)+x			; 4 get param
	and a, #3							; 2
	mov x, a							; 2
	mov a, (table_vib_conv)+x			; 5 convert param
										; 
	asl a								; 2
	movz (int1), a						; 4
										; 
	mov a, (xms_channels_other+6)+x		; 5 store
	and a, #%11111001					; 2 edit flags
	orz a, (int1)						; 3
	mov (xms_channels_other+6)+x, a		; 6
										; 
XMS_FX_EX_VIBRATOFORMo:					; 
	jmp _xmspme_exit					; 3

; EFFECT E5y SET FINETUNE -----------------------------------------------------------------------------------------------
XMS_FX_EX_SETFINETUNEf:					;[3 cycles]
	; uses jump from other ticks!		;
XMS_FX_EX_SETFINETUNEo:					;
	jmp _xmspme_exit					;

; EFFECT E6y PATTERN LOOP -----------------------------------------------------------------------------------------------
XMS_FX_EX_PATTERNLOOPf:					;[35 cycles]
	pop x								; 4 get chan
	push x								; 4
	movz a, (xms_channels+5)+x			; 4 get param
	and a, #$0F							; 2
	beq _xmsfxxpl_set					; 2/4 set jump pos if zero
	cmp (xms_patt_loop), #0				; 5 check loop count
	bne _xmsfxxpl_hasloop				; 2/4
	movz (xms_patt_loop), a				; 4
	b _xmsfxxpl_gotloop					; 4
_xmsfxxpl_hasloop:						; 
	decz (xms_patt_loop)				; 4
	beq XMS_FX_EX_PATTERNLOOPo			; 2/4
_xmsfxxpl_gotloop:						; 
	or (xms_patt_loop), #$10			; 5 stuff..
	jmp _xmspme_exit					; 3
_xmsfxxpl_set:							; 
	movw ya, (xms_pattern_read)			; 5
	movw (xms_patt_loop_pos), ya		; 4
	mov (xms_patt_loop_row), (xms_row)	; 5
XMS_FX_EX_PATTERNLOOPo:					; 
	jmp _xmspme_exit					; 3

; EFFECT E7y TREMOLO WAVEFORM -----------------------------------------------------------------------------------------------
XMS_FX_EX_TREMOLOFORMf:					;[46 cycles]
	pop a								; 4 get chan
	push a								; 4
	movz a, (xms_channels+5)+x			; 4 get param
	and a, #3							; 2
	mov x, a							; 2
	mov a, (table_vib_conv)+x			; 5 convert param
										; 
	xcn a								; 5
	movz (int1), a						; 4
										; 
	mov a, (xms_channels_other+6)+x		; 5 store
	and a, #%11001111					; 2 edit flags
	orz a, (int1)						; 3
	mov (xms_channels_other+6)+x, a		; 6
										; 
XMS_FX_EX_TREMOLOFORMo:					; 
	jmp _xmspme_exit					; 3

; EFFECT E8y Set Panning (old) -----------------------------------------------------------------------------------------------
XMS_FX_EX_SETPANNINGf:					;[35 cycles]
	pop x								; 4 get chan
	push x								; 4
	movz a, (xms_channels+5)+x			; 4 get param
	and a, #15							; 2 parse
	asl a								; 2
	asl a								; 2
	movz (xms_channels+9)+x, a			; 5 store
	movz (xms_cc_panning), a			; 4 store
	or (xms_cc_flags), #ccflag_panning	; 5 set flag
	jmp _xmspme_exit					; 3 exit
XMS_FX_EX_SETPANNINGo:					; 
	jmp _xmspme_exit					; 3

; EFFECT E9y Retrigger Note (old) -----------------------------------------------------------------------------------------------
XMS_FX_EX_RETRIGGERf:					;[22 cycles]
	pop x								; 4 get chan
	push x								; 4
	movz a, (xms_channels+5)+x			; 4 get param
	and a, #15							; 2 parse
	movz (xms_channels+11)+x, a			; 5 set counter
	jmp _xmspme_exit					; 3
XMS_FX_EX_RETRIGGERo:					;[45 cycles]
	pop x								; 4 get chan
	push x								; 4
	movz a, (xms_channels+5)+x			; 4 get param
	bne _xmsfxxr_skip					; 2/4 exit if zero (??? this is an extended effect)
	jmp _xmspme_exit					; 3
_xmsfxxr_skip:							; 
	movz a, (xms_channels+11)+x			; 4 get counter
	dec a								; 2 decrement
	cmp a, #0							; 2
	bne _xmsfxxr_wait					; 2/4 check if zero
										; 
	or (xms_cc_flags), #ccflag_keyon	; 5 retrigger note
										; 
	movz a, (xms_channels+5)+x			; 4 get param
	and a, #15							; 2 parse
										; 
_xmsfxxr_wait:							; 
	movz (xms_channels+11)+x, a			; 5 save
	cmp a, #1
	bne _xmsfxxr_exit
	or (xms_peek), (channel_bit)
_xmsfxxr_exit:
	jmp _xmspme_exit					; 3 exit

; EFFECT EAy FINE VOLUME SLIDE UP -----------------------------------------------------------------------------------------------
XMS_FX_EX_FINEVOLUPf:					;[76 cycles]
	pop x								; 4 get chan
	push x								; 4
	movz a, (xms_channels+5)+x			; 4 get param
	and a, #15							; 2 parse
	bne _xmsfxxfvu_isparam				; 2/4 check zero
	mov a, (XMS_FXP_PREVIOUS+10)+x		; 5 get prev param
	movz (xms_channels+5)+x, a			; 5
_xmsfxxfvu_isparam:						; 
	mov (XMS_FXP_PREVIOUS+10)+x, a		; 6 save
	and a, #15							; 2
										; 
	call XMS_CHANNEL_VOLSLIDEu			; 8+31 slide volume
	; uses other jump					; 
XMS_FX_EX_FINEVOLUPo:					; 
	jmp _xmspme_exit					; 3

; EFFECT EBy FINE VOLUME SLIDE DOWN -----------------------------------------------------------------------------------------------
XMS_FX_EX_FINEVOLDOWNf:					;[75 cycles]
	pop x								; 4 get chan
	push x								; 4
	movz a, (xms_channels+5)+x			; 4 get param
	and a, #15							; 2 parse
	bne _xmsfxxfvd_isparam				; 2/4
	mov a, (XMS_FXP_PREVIOUS+11)+x		; 5
	movz (xms_channels+5)+x, a			; 5
_xmsfxxfvd_isparam:						; 
	mov (XMS_FXP_PREVIOUS+11)+x, a		; 6
	and a, #15							; 2
										; 
	call XMS_CHANNEL_VOLSLIDEd			; 8+33 slide volume
	; uses other jump					; 
XMS_FX_EX_FINEVOLDOWNo:					; 
	jmp _xmspme_exit					; 3

; EFFECT ECy NOTE CUT -----------------------------------------------------------------------------------------------
XMS_FX_EX_NOTECUTf:						;[39 cycles]
	pop x								; 4 get chan
	push x								; 4
	movz a, (xms_channels+5)+x			; 4 get param
	and a, #15							; 2 parse
	cmp a, #0							; 2 cut immediately if zero...
	beq _xmsfxxnc_cut_imm				; 2/4+19
	jmp _xmspme_exit					; 3
XMS_FX_EX_NOTECUTo:						;[40 cycles]
	pop x								; 4 get chan
	push x								; 4
	movz a, (xms_channels+5)+x			; 4 get param
	and a, #15							; 2 parse
	cmpz a, (xms_tick)					; 3 compare with tick count
	beq _xmsfxxnc_cut_imm				; 2/4
	jmp _xmspme_exit					; 3
_xmsfxxnc_cut_imm:						; 
	mov a, #0							; 2 cut volume		{ 19 cycles
	movz (xms_channels+6)+x, a			; 5
	movz (xms_cc_volume), a				; 4
	or  (xms_cc_volume), #ccflag_volume	; 5
	jmp _xmspme_exit					; 3					}

; EFFECT EDy NOTE DELAY -----------------------------------------------------------------------------------------------
XMS_FX_EX_NOTEDELAYf:							;[8 cycles]
	pop x
	push x
	and (xms_cc_flags), #( $FF - (ccflag_pitch | ccflag_volume | ccflag_keyon | ccflag_panning | ccflag_source) )	; 5 prevent keyon
	mov a, #1
	mov (xms_channels_other+coso_ndelay)+x, a
	jmp _xmspme_exit							; 3
XMS_FX_EX_NOTEDELAYo:							;[27 cycles]
	pop x										; 4 get chan
	push x										; 4
	movz a, (xms_channels+5)+x					; 4 get param
	and a, #15									; 2
	setc
	sbcz a, (xms_tick)							; 3 compare with tick count
	bne _xmsfxxnd_wait							; 2/4
																			; 
	or (xms_cc_flags), #( ccflag_pitch | ccflag_volume | ccflag_keyon | ccflag_panning | ccflag_source )		; 5 keyon
	mov a, #0
	mov (xms_channels_other+coso_ndelay)+x, a
	movz (xms_channels+cso_fxparam)+x, a
	movz a, (xms_channels+cso_sample)+x
	movz (xms_cc_source), a
	
	jmp _xmspme_exit							; 3
_xmsfxxnd_wait:									; 
	cmp a, #1
	bne _xmsfxxnd_exit
	or (xms_peek), (channel_bit)
_xmsfxxnd_exit:									; 
	jmp _xmspme_exit							; 3

; EFFECT EEy PATTERN DELAY -----------------------------------------------------------------------------------------------
XMS_FX_EX_PATTERNDELAYf:						;[27 cycles]
	pop x										; 4 get chan
	push x										; 4
	movz a, (xms_fxfirst)						; 3 skip if first trigger...
	and a, #1									; 2
	beq XMS_FX_EX_PATTERNDELAYo					; 2/4
												; 
	movz a, (xms_channels+5)+x					; 3
	and a, #15									; 2
	movz (xms_patt_delay), a					; 4
XMS_FX_EX_PATTERNDELAYo:						; 
	jmp _xmspme_exit							; 3

; EFFECT EFy SEND MESSAGE -----------------------------------------------------------------------------------------------
XMS_FX_EX_SENDMESSAGEf:
	pop x
	push x
	movz a, (xms_channels+cso_fxparam)+x
	and  a, #15
	movz (int1), a
	movz a, (xms_last_message)
	and  a, #128
	orz  a, (int1)
	eor  a, #128
	movz (xms_last_message), a
	movz ($F4), a
XMS_FX_EX_SENDMESSAGEo:
	jmp _xmspme_exit

; ----------------------------------------------------------------------------------------------------
; XM EFFECTS
; ----------------------------------------------------------------------------------------------------
; EFFECT Gxy SET GLOBAL VOLUME -----------------------------------------------------------------------------------------------
XMS_FX_SETGLOBALVOLf:							;[25 cycles]
	pop x										; 4 blah blah
	push x										; 4
	movz a, (xms_channels+5)+x					; 4 blah
	cmp a, #65									; 2 clip
	bcc _xmsfxsgv_vol_good						; 2/4
	mov a, #64									; 2
_xmsfxsgv_vol_good:								; 
	movz (xms_global_vol), a					; 4 set global vol
XMS_FX_SETGLOBALVOLo:							; 
	jmp _xmspme_exit							; 3 exit

; EFFECT Hxy GLOBAL VOLUME SLIDE -----------------------------------------------------------------------------------------------
XMS_FX_GLOBALVOLSLIDEf:							;[33 cycles]
	pop x										; 4 get chan
	push x										; 4
	movz a, (xms_channels+5)+x					; 4 check/get prev data
	bne _xmsfxgvs_isparam						; 2/4
	mov a, (XMS_FXP_PREVIOUS+12)+x				; 5
	movz (xms_channels+5)+x, a					; 5
_xmsfxgvs_isparam:								; 
	mov (XMS_FXP_PREVIOUS+12)+x, a				; 6
xmsfxgvs_isparam:								; 
												; 
	jmp _xmspme_exit							; 3
XMS_FX_GLOBALVOLSLIDEo:							;[46 cycles]
	pop x										; 4 get chan
	push x										; 4
	movz a, (xms_channels+5)+x					; 5 get param
	cmp a, #16									; 2
	bcc _xmsfxgvs_down							; 2/4
	xcn a										; 5 slide volume up:
	and a, #15									; 2
	clrc										; 2
	adcz a, (xms_global_vol)					; 3
	cmp a, #64									; 2
	bcc _xmsfxgvs_finish						; 2/4
	mov a, #64									; 2 clip
	b _xmsfxgvs_finish							; 4
_xmsfxgvs_down:									; slide volume down:
	setc										; 2
	movz a, (xms_global_vol)					; 4
	sbcz a, (xms_channels+5)+x					; 4
	bpl _xmsfxgvs_finish						; 2/4
	mov a, #0									; 2 clip
_xmsfxgvs_finish:								; 
	movz (xms_global_vol), a					; 4 store
	jmp _xmspme_exit							; 3

; EFFECT Kxy KEY OFF -----------------------------------------------------------------------------------------------
XMS_FX_KEYOFFf:									;[42 cycles]
	pop x										; 4 get chan
	push x										; 4
	movz a, (xms_channels+5)+x					; 4 get param
	and a, #15									; 2 parse
	cmp a, #0									; 2 check zero, immediate cut
	beq _xmsfxko_imm							; 2/4
	jmp _xmspme_exit							; 3
XMS_FX_KEYOFFo:									;[37 cycles]
	pop x										; 4 get chan
	push x										; 4
	movz a, (xms_channels+5)+x					; 4 get param
	and a, #15									; 2 parse
	cmpz a, (xms_tick)							; 3
	beq _xmsfxko_imm							; 2/4
	jmp _xmspme_exit							; 3
_xmsfxko_imm:									; 
												; 
	mov a, (xms_channels_other+6)+x				; 5 read
	and a, #%01111111							; 2 keyoff
	mov (xms_channels_other+6)+x, a				; 6 store
	jmp _xmspme_exit							; 3 exit

; EFFECT Lxy ENVELOPE POSITION -----------------------------------------------------------------------------------------------
#ifdef FX_INCLUDE_ENVPOS
XMS_FX_ENVELOPEPOSf:							;[23+? cycles]
	pop x										; 4 get chan
	push x										; 4
	movz a, (xms_channels+cso_fxparam)+x		; 4 get param
	movz (int1), a
	call XMS_CHANNEL_ENV_SETPOSITION			; 8+39
	; uses other jump							; 
XMS_FX_ENVELOPEPOSo:							; 
	jmp _xmspme_exit							; 3

#else
XMS_FX_ENVELOPEPOSf:
XMS_FX_ENVELOPEPOSo:
	jmp _xmspme_exit
#endif

; EFFECT Pxy PANNING SLIDE -----------------------------------------------------------------------------------------------
XMS_FX_PANSLIDEf:								;[33 cycles]
	pop x										; 4 get chan
	push x										; 4
	movz a, (xms_channels+5)+x					; 4 check/get prev data
	bne _xmsfxpans_isparam						; 2/4
	mov a, (XMS_FXP_PREVIOUS+13)+x				; 5
	movz (xms_channels+5)+x, a					; 5
_xmsfxpans_isparam:								; 
	mov (XMS_FXP_PREVIOUS+13)+x, a				; 6
	jmp _xmspme_exit							; 3
XMS_FX_PANSLIDEo:								;[56 cycles]
	pop x										; 4 get chan
	push x										; 4
	movz a, (xms_channels+5)+x					; 4 get param
	cmp a, #16									; 2 check direction
	bcs _xmsfxps_hiparam						; 2/4
	; loparam									; 
	and a, #$0F									; 2
	setc										; 2 subtract from volume...
	movz (int1), a								; 4
	movz a, (xms_channels+9)+x					; 4
	sbcz a, (int1)								; 3
	bcs _xmsfxps_lo_over						; 2/4
	mov a, #0									; 2 clip
_xmsfxps_lo_over:								; 
	movz (xms_channels+9)+x, a					; 5 store
	b _xmsfxps_finish							; 4
_xmsfxps_hiparam:								; 
	and a, #$F0									; 2
	xcn a										; 5
	clrc										; 2 add to volume...
	adcz a, (xms_channels+9)+x					; 3
	cmp a, #64									; 2
	bcc _xmsfxps_hi_over						; 2/4
	mov a, #64									; 2 clip
_xmsfxps_hi_over:								; 
	movz (xms_channels+9)+x, a					; 5 store
_xmsfxps_finish:								; 
												; 
	movz (xms_cc_panning), a					; 4 set panning
	or (xms_cc_flags), #ccflag_panning			; 5 set flag
												; 
	jmp _xmspme_exit							; 3 exit
	
; EFFECT Rxy RETRIGGER NOTE (extended) -----------------------------------------------------------------------------------------------

table_xms_retrig_jumps:
.word	_xmsfxr_0
.word	_xmsfxr_1
.word	_xmsfxr_1
.word	_xmsfxr_1
.word	_xmsfxr_1
.word	_xmsfxr_1
.word	_xmsfxr_6
.word	_xmsfxr_7
.word	_xmsfxr_8
.word	_xmsfxr_9
.word	_xmsfxr_9
.word	_xmsfxr_9
.word	_xmsfxr_9
.word	_xmsfxr_9
.word	_xmsfxr_E
.word	_xmsfxr_F

XMS_FX_RETRIGGERf:							;[68+? cycles]
	pop x									; 4 get chan
	push x									; 4
	mov a, (XMS_FXP_PREVIOUS+14)+x			; 5 get prev data
	movz (int1), a							; 4 
	movz y, (xms_channels+5)+x				; 4 get param
	mov a, y								; 2 save
	and a, #$0F								; 2 mask
	beq _xmsfxr_noparam2					; 2/4 lo param != 0?
	and (int1), #$F0						; 5 mask out lo param
_xmsfxr_noparam2:							; 
	mov a, y								; 2 load
	and a, #$F0								; 2 mask
	beq _xmsfxr_noparam1					; 2/4 hi param != 0?
	and (int1), #$0F						; 5 mask out hi param
_xmsfxr_noparam1:							; 
	mov a, y								; 2 load
	orz a, (int1)							; 3 or masked prev data
	mov (XMS_FXP_PREVIOUS+14)+x, a			; 6 save
	movz (xms_channels+5)+x, a				; 5 save (?)
											; 
	movz a, (xms_fxfirst)					; 3 check first time
	and a, #%10								; 2
	beq XMS_FX_RETRIGGERo					; 2/4 branch if not
	and (xms_fxfirst), #%11111101			; 5 reset counters on first time
	movz a, (xms_channels+5)+x				; 4
	and a, #15								; 2
	movz (int1), a							; 4
	movz a, (xms_channels+15)+x				; 4
	and a, #$F0								; 2
	orz a, (int1)							; 3
	movz (xms_channels+15)+x, a				; 4
											; 
	jmp _xmspme_exit						; 3
XMS_FX_RETRIGGERo:							;[174 cycles]
	pop x									; 4 get chan						{ 20 cycles
	push x									; 4
											; 
	movz a, (xms_channels+15)+x				; 4 get counter
	and a, #15								; 2
	dec a									; 2
	beq _xmsfxr_no_retrig_NOT				; 2/4
	jmp _xmsfxr_no_retrig					; 3									}
_xmsfxr_no_retrig_NOT:						; 
											; 
	movz a, (xms_channels+5)+x				; 4 read param						{ 58 cycles
	and a, #15								; 2 mask
	movz (int1), a							; 4 store
	movz a, (xms_channels+15)+x				; 4 read counter var
	and a, #$F0								; 2 mask
	orz a, (int1)							; 3 mix
	movz (xms_channels+15)+x, a				; 4 store
											; 
	; edit volume							; 
	movz a, (xms_channels+5)+x				; 4 get param
	xcn a									; 5 parse
	and a, #15								; 2
	asl a									; 2
	push a									; 4 preserve jump offset
	movz a, (xms_channels+6)+x				; 4 get volume
	movz (int1), a							; 4 store
	pop x									; 4 retrieve
											; 
	jmp [table_xms_retrig_jumps+x]			; 6 jump							}
_xmsfxr_0:									; -----------------------------------------------
											;   shouldn't equal zero USES LAST VALUE
	jmp _xmsfxr_8							; 3 jump to 8 if user didn't specify any previous
_xmsfxr_1:									; -----------------------------------------------
	mov a, x								; 2 x == mode*2			{ 70 cycles
	mov (int1+1), #1						; 5 default: 1
	lsr a									; 2
	dec a									; 2
	beq _xmsfxr_1_exit						; 2/4
_xmsfxr_1_loop:								;			{ 38 cycles (4x)
	aslz (int1+1)							; 4 shift value
	dec a									; 2 dec&loop
	bne _xmsfxr_1_loop						; 2/4		}
_xmsfxr_1_exit:								; 
	setc									; 2
	sbc (int1), (int1+1)					; 6 subtract value
	bpl _xmsfxr_1_pl						; 2/4
	mov a, #0								; 2 clip
	b _xmsfxr_setvol						; 4 break
_xmsfxr_1_pl:								; 
	movz a, (int1)							; 3 fetch
	b _xmsfxr_setvol						; 4 break				}
_xmsfxr_6:									; -----------------------------------------------
	movz a, (int1)							; 3						{ 20 cycles
	mov y, #171								; 2
	mul ya									; 9 compute (volume*171)>>8
	mov a, y								; 2 ...
	b _xmsfxr_setvol						; 4 break				}
_xmsfxr_7:									; -----------------------------------------------
	movz a, (int1)							; 3 get volume			{ 9 cycles
	lsr a									; 2 /2
	b _xmsfxr_setvol						; 4 break				}
_xmsfxr_8:									; -----------------------------------------------
	movz a, (int1)							; 3 no change			{ 7 cycles
	b _xmsfxr_setvol						; 4 break				}
_xmsfxr_9:									; -----------------------------------------------
	mov a, x								; 2 a = mode*2			{ 71 cycles
	lsr a									; 2 a = mode
	setc									; 2 
	mov (int1+1), #1						; 5 default=1
	sbc a, #9								; 2
	beq _xmsfxr_9_exit						; 2/4
_xmsfxr_9_loop:								;			{ 38 cycles
	aslz (int1+1)							; 4 shift value
	dec a									; 2 dec&loop
	bne _xmsfxr_9_loop						; 2/4		}
_xmsfxr_9_exit:								; 
	clrc									; 2
	movz a, (int1)							; 3 add value
	adcz a, (int1+1)						; 3
	cmp a, #64								; 2
;	bcc _xmsfxr_9_cc						; 2/4
;	mov a, #64								; 2 clip
;	b _xmsfxr_setvol						; 4
;_xmsfxr_9_cc:								; 
;	movz a, (int1)							; X BUG???
	b _xmsfxr_setvol						; 4 return				}
_xmsfxr_E:									; -----------------------------------------------
	movz a, (int1)							; 3 read				{ 24 cycles
	mov y, #192								; 2
	mul ya									; 9 compute (vol*192)>>7 (?) :)
	rol a									; 2
	mov a, y								; 2
	rol a									; 2
	b _xmsfxr_setvol						; 4						}
_xmsfxr_F:									; -----------------------------------------------
	movz a, (int1)							; 3	vol*2				{ 5 cycles
	asl a									; 2						}
_xmsfxr_setvol:								; 
	pop x									; 4	get chan			{ 25 cycles
	push x									; 4
	cmp a, #64
	bcc _xmsfxr_volclip
	mov a, #64
_xmsfxr_volclip:
	movz (xms_channels+6)+x, a				; 5 save volume
	movz (xms_cc_volume), a					; 4 save volume
	or (xms_cc_flags), #(ccflag_volume| ccflag_keyon)	; 5 set flag
											; 
	jmp _xmsfxr_exit						; 3	return				}
_xmsfxr_no_retrig:							; 
	cmp a, #1
	bne _xmsfxr_no_ramp
	or (xms_peek), (channel_bit)
_xmsfxr_no_ramp:
	movz (int1), a							; 4	do some things		{ 21 cycles
	movz a, (xms_channels+15)+x				; 4
	and a, #$F0								; 2
	orz a, (int1)							; 3
	movz (xms_channels+15)+x, a				; 5
_xmsfxr_exit:								;
	jmp _xmspme_exit						; 3						}

; EFFECT Txy TREMOR -----------------------------------------------------------------------------------------------
#ifdef FX_INCLUDE_TREMOR

XMS_FX_TREMORf:								;[44 cycles]
	pop x									; 4 get chan
	push x									; 4
	movz a, (xms_fxfirst)					; 3 check fx_first
	and a, #%100							; 2
	bne _xmsfxtr_first						; 2/4
	b _xmsfxtr_exit							; 4
_xmsfxtr_first:								; 
	movz a, (channel_bit)					; 3 reset flags
	eor a, #255								; 2
	andz a, (xms_tremor_flags)				; 3
	movz a, (xms_channels+15)+x				; 4
	and a, #15								; 2
	movz (xms_channels+15)+x, a				; 5
	and (xms_fxfirst), #%11111011			; 5
_xmsfxtr_exit:								; 
	jmp _xmspme_exit						; 3
XMS_FX_TREMORo:								;[73 cycles]
	pop x									; 4 get chan			{ 24 cycles
	push x									; 4
	movz a, (xms_channels+15)+x				; 4
	and a, #$F0								; 2
	bne _xmsfxtr_next						; 2/4
	; DO SOME TREMOR						; 
	movz a, (channel_bit)					; 3
	andz a, (xms_tremor_flags)				; 3
	
	beq _xmsfxtr_on							; 2/4					}
_xmsfxtr_off:
	; get OFF ticks
	eor (xms_tremor_flags), (channel_bit)	; 5 set off flag		{ 49 cycles
	movz a, (xms_channels+5)+x				; 4 set counter..
	and a, #15								; 2
	xcn a									; 5
	movz (int1), a							; 4
	movz a, (xms_channels+15)+x				; 4
	orz a, (int1)							; 3
	movz (xms_channels+15)+x, a				; 5
											; 
	mov (xms_cc_volume), #0					; 5
	or  (xms_cc_flags),  #ccflag_volume		; 5
	b _xmsfxtr_exit							; 4+3					}
_xmsfxtr_on:								; 
	; get ON ticks							; 
	eor (xms_tremor_flags), (channel_bit)	; 5 set on flag			{ 47 cycles
	movz a, (xms_channels+5)+x				; 4 set counter...
	and a, #$F0								; 2
	movz (int1), a							; 4
	movz a, (xms_channels+15)+x				; 4
	orz a, (int1)							; 3
	movz (xms_channels+15)+x, a				; 5
	movz a, (xms_channels+6)+x				; 4
	movz (xms_cc_volume), a					; 4
	or   (xms_cc_flags), #ccflag_volume		; 5
	b _xmsfxtr_exit							; 4+3					}
											; 
_xmsfxtr_next:								; 
	setc									; 2 decrement counter	{ 12 cycles
	sbc a, #$10								; 2
	movz (xms_channels+15)+x, a				; 5
											; 
	jmp _xmspme_exit						; 3 return				}

#else
XMS_FX_TREMORf:	
XMS_FX_TREMORo:
	jmp _xmspme_exit
	
#endif

; EFFECT Xxy EXTRA FINE PORTA ------------------------------------------------------------------------------------------
XMS_FX_XFINEPORTAf:							;[148 cycles]
	pop x									; 4 get chan
	push x									; 4
	mov (int2), #0							; 5
	movz a, (xms_channels+5)+x				; 4 get param
	and a, #15								; 2 check zero
	bne _xmsfxxfp_isparam					; 2/4
	mov a, (XMS_FXP_PREVIOUS+15)+x			; 5 get last
	movz (xms_channels+5)+x, a				; 5
_xmsfxxfp_isparam:							; 
	mov (XMS_FXP_PREVIOUS+15)+x, a			; 6 save
	and a, #$F0								; 2 mask
	cmp a, #$10								; 2
	beq _xmsfxefp_up						; 2/4 check direction
	; PORTA DOWN							; 
	movz a, (xms_channels+5)+x				; 4 slide down
	and a, #$0F								; 2
	call XMS_CHANNEL_PITCH_SLIDEDOWN		; 8+80
	b XMS_FX_XFINEPORTAo					; 4
_xmsfxefp_up:								; 
	; PORTA UP								; slide up..
	mov a, (xms_channels+5)+x				; 4
	and a, #$0F								; 2
	call XMS_CHANNEL_PITCH_SLIDEUP			; 8+86
XMS_FX_XFINEPORTAo:							; 
	jmp _xmspme_exit						; 3 return

; UNUSED EFFECTS -----------------------------------------------------------------------------------------------
XMS_FX_UNUSED				=_xmspme_exit

; -----------------------------------------------------------------------------------------------------
; XMS CHANNEL ROUTINES
; -----------------------------------------------------------------------------------------------------

XMS_CHANNEL_INTERRUPT:
	; cut channel
	push a
	mov  a, #0
	mov  a, (xms_channels_other+6)+x
	and  a, #$Fe
	mov  (xms_channels_other+6)+x, a
	mov  a, #0
	mov  (xms_channels_other+0)+x, a
	mov  (xms_channels_other+1)+x, a
	mov  a, #255
	movZ (xms_channels+cso_sample)+x, a
	pop  a
	ret

XMS_CHANNEL_RESTORE:
	

XMS_CHANNEL_UPDATEDSP:							;[638 cycles]								| BIG ROUTINE
	; x = channel * 16																		| OPTIMIZATION: MEDIUM
	; updates channel with xms_cc parameters												|
	; and it goes kinda like this...
	; 0. RESET GAIN if <setsource> or <startnote>
	; 1. set source			; small
	; 2. set pitch			; 357 cycles
	; 3. set volume			; 82 cycles
	; 4. set offset			; 120 cycles
	; 5. start note			; small
	; 6. reset offset		; 32
	; 6. set panning		; 114 cycles

	; dont update over sound effects
	mov a, x
	xcn a
	mov y, a
	mov a, (samp_playing)+y
	beq _xcu_ok
	mov (xms_cc_flags), #0						; 5 reset flags
	ret
_xcu_ok:
	
	mov  a, (XMS_TABLE_BITS)+y
	andz a, (samp_dirty)
	beq _xcu_ok2
	; fix channel
	
	mov  a, (XMS_TABLE_BITS)+y
	eorz a, (samp_dirty)
	movz (samp_dirty), a
	
	; reset adsr
	mov  a, x
	or   a, #DSP_ADSR
	mov  y, #0
	movw ($F2), ya
	
	; reset source
;	movz a, (xms_channels+cso_sample)+x
;	movz (xms_cc_source), a
;	or   (xms_cc_flags), #ccflag_source
	
_xcu_ok2:
	
	
	
	movz a, (xms_cc_flags)						; 3 get flags			{ 5 cycles
	and a, #%10001								; 2 check for start		}
	
	
	
	beq _xmscu_nostarting						; 2/4					{ 12 cycles
	mov a, x									; 2 reset gain
	or a, #DSP_GAIN								; 2
	mov y, #0									; 2		<------------------------------------
	movw ($F2), ya								; 4
_xmscu_nostarting:								;						}
	
	bbc4 (xms_cc_flags), _xmscu_nosource		; 5/7 check source		{ 16 cycles
	mov a, x									; 2 set source number
	or a, #4									; 2
	movz y, (xms_cc_source)						; 3
;	movz y, (xms_channels+cso_sample)+x
	movw ($F2), ya								; 4
_xmscu_nosource:								;						}
	
	bbs1 (xms_cc_flags), _xmscu_nopitchN		; 5/7 check pitch		{ ???
	jmp _xmscu_nopitch
_xmscu_nopitchN:
	mov y, #c_inst_noise
	mov a, [cur_inst]+y
	bne _xmscu_noisepitch
	mov a, x									; 2 set dsp access
	or a, #2									; 2
	movz ($F2), a								; 4
	mov a, (xms_channels_other+coso_pa+1)+x		; 5 ya = period + vibrato
	
	lsr a										; 2
	bcc _xmscu_novib							; 2/4
	asl a										; 2
	mov (xms_channels_other+coso_pa+1)+x, a		; 6
	
	bmi _xmscu_vsub								; 2/4
_xmscu_vadd:
	mov a, (xms_channels_other+coso_pa)+x		; 5
	mov y, #0									; 2
	b _xmscu_isvib								; 4
_xmscu_vsub:
	
	mov a, (xms_channels_other+coso_pa)+x		; 5
	beq _xmscu_novib
	eor a, #255									; 2
	inc a										; 2
	beq _xmscu_novib							; 2/4
	mov y, #255									; 2
	b _xmscu_isvib								; 4
	
_xmscu_novib:
	mov a, #0									; 2
	mov y, #0									; 2
_xmscu_isvib:
	addw ya, (xms_cc_period)					; 5
	addw ya, (xms_cc_perioda)					; 5
	
	;call period_clip							; 8+25 clip period
	call XMS_PeriodClip
	movw (int1), ya								; 4
												; 
	push x										; 4
	call XMS_PERIOD2FREQ
	pop x										; 4
	
	incz ($F2)
	mov ($F3), (int3+1)
	decz ($F2)
	mov ($F3), (int3)
	
	mov  ($F2), #DSP_NON
	movz a, (channel_bit)
	eor  a, #255
	and  a, ($F3)
	movz ($F3), a
	b _xmscu_nopitch
_xmscu_noisepitch:
	bbc0 (xms_cc_flags), _xmscu_nopitch
	movz a, (xms_channels+cso_note)+x
	setc
	sbc  a, #48									; sub middle C
	and  a, #31   ; mask for stupid people
	movz (int1), a
	mov  ($F2), #DSP_FLG
	movz a, ($F3)
	and  a, #%11100000
	orz  a, (int1)

	mov  ($F3), a
	
	mov ($F2), #DSP_NON
	or  ($F3), (channel_bit)

	mov a, x
	or  a, #2
	mov y, #$00
	movw ($F2), ya
	inc a
	mov y, #$10
	movw ($F2), ya
_xmscu_nopitch:									;						}
	
	mov1 c, (xms_cc_flags+(5<<13))				; 4 check sample offset	{ 98 cycles/8 cycles : ONLY GETS APPLIED WITH EFFECT 9xx! (on first tick)
	bcc _xmscu_nooffset							; 2/4
	; so = xms_cc_sampoff						; 
	; sampoff = source_offset + so*144			; 
												; 
	movz a, (xms_channels+10)+x					; 4 get sample
	asl a										; 2 *4
	asl a										; 2
	movz (int2), a								; 4 save
	mov (int2+1), #$02							; 5
												; 
	movz a, (xms_cc_sampoff)					; 3 get sample offset
	mov y, #144									; 2 multiply
	mul ya										; 9
	movw (int1), ya								; 4
												; 
	mov y, #0									; 2 read source offset
	mov a, [int2]+y								; 6
	push a										; 4
												; 
	adcz a, (int1)								; 3 add sample offset....
	movz (int1), a								; 4
	inc y										; 2
	mov a, [int2]+y								; 6
	push a										; 4
	adcz a, (int1+1)							; 3
	movz (int1+1), a							; 4
												; 
	; int1 = new offset							; 
	mov [int2]+y, a								; 7 overwrite
	movz a, (int1)								; 3
	dec y										; 2
	mov [int2]+y, a								; 7
_xmscu_nooffset:								;						}

	movz a, (xms_cc_panning)					; 3 get panning			{ 109 cycles
	asl a										; 2 shift
	asl a										; 2
	notc										; 2 trick
	sbc a, #0									; 2
	
XMS_CHANNEL_SETPAN:						;[98 cycles]							| INLINE COMMON ROUTINE
	; a=pan								;										| OPTIMIZATION: MEDIUM HIGH
	movz (int1), a						; 4 save pan			{ 23 cycles
	
	sbc a, #128							; 2 compute 128-abs(128-pan)
	bpl _xmscev_plus1					; 2
	eor a, #255							; 2
	inc a								; 2
	setc								; 2
_xmscev_plus1:
	sbc a, #128							; 2
	eor a, #255							; 2
	inc a								; 2
	push a								; 4	save result			}
	mov a, (xms_channels_other+5)+x		; 5 read pan envelope	{ 15/17 cycles negative/positive
	pop y								; 4 restore value
	setc								; 2
	sbc a, #32							; 2 epan-32
	bpl _xmscev_eplus					; 2/4					}
_xmscev_eminus:
	eor a, #255							; 2	compute pan+((epan-32)*8*128-abs(pan-128))\256	{ 35 cycles
	inc a								; 2 some values were calculated already
	asl a								; 2
	asl a								; 2
	asl a								; 2
	notc								; 2
	sbc a, #0							; 2
	mul ya								; 9
	mov a, y							; 2
	sbcz a, (int1)						; 3
	eor a, #255							; 2
	inc a								; 2
	jmp _xmscev_setvol					; 3													}
_xmscev_eplus:
	asl a								; 2	compute pan+((epan-32)*8*128-abs(pan-128))\256	{ 26 cycles
	asl a								; 2 some values were calculated already
	asl a								; 2
	notc								; 2
	sbc a, #0							; 2
	mul ya								; 9
	mov a, y							; 2
	clrc								; 2
	adcz a, (int1)						; 3													}
_xmscev_setvol:
	
	lsr a								; 2	set dsp volumes		{ 25 cycles
	
	mov y, a							; 2
	inc x								; 2
	movz ($F2), x						; 4
	dec x								; 2
	eor a, #127							; 2
	movz ($F3), y						; 4
	mov y, a							; 2
	mov a, x							; 2
	movw ($F2), ya						; 4						}
	;-------------------------------------------------------------------------------------------
_xmscu_nopan:									;						}


	bbc0 (xms_cc_flags), _xmscu_nostart


#ifdef xms_ssfix
	mov a, x			; make sure GAIN is in direct mode
	or  a, #DSP_GAIN	; and zero volume
	mov y, #0			; 
	movw ($F2), ya
#endif

	mov ($F2), #DSP_KON							; 5 set dsp access
	movz a, (channel_bit)						; 3 load channel_bit
	movz ($F3), a								; 4 store, keyon		}

_xmscu_nostart:

;-------------------------------------------------------------------------------------------------------------------------
	movz a, (xms_cc_volume)						; 4	set volume...		{ 72 cycles
	asl a
	mov y, a
	; y=0-127
XMS_CHANNEL_SETVOLUME:									;[68 cycles]									; INLINE COMMON ROUTINE
	; a = volume										;												; OPTIMIZATION: MEDIUM/HIGH
	; x = channel * 16									; 
	; formula = ((((((fadeout*envelope)/65536)*global)/64)*channel)/64)
	movz a, (xms_global_vol)							; 3 global*volume			{ 20 cycles
	asl a												; 2
	asl a												; 2
	notc												; 2
	sbc a, #0											; 2
	mul ya												; 9							}
	mov a, (xms_channels_other+3)+x						; 5 *envelope				{ 22 cycles
	asl a												; 2
	asl a												; 2
	notc												; 2
	sbc a, #0											; 2
	mul ya												; 9							}
	mov a, (xms_channels_other+1)+x						; 5 *fadeout				{ 14 cycles
	mul ya												; 9							}
#ifndef xmspc_mode
	movz a, (xms_volume)								; skip if its for an SPC image
	mul ya												;
#endif
	beq _xmscu_zvolume									; <---------------------------------------------- PROBLEM...
_xmscu_zvol_set:
	mov a, x
	xcn a
	mov x, a
	movz (xms_final_volume)+x, y
	lsr  (xms_final_volume), x
	xcn a
	mov x, a
;	bbc4 (xms_cc_flags), _xmscu_rvolume
	bbc0 (xms_cc_flags), _xmscu_rvolume
_xmscu_isvolume:
;	mov a, x											; 2							{ 8 cycles
	or a, #DSP_GAIN											; 2 set dsp volume
	movw ($F2), ya										; 4							}
	jmp _xmscu_novolume
_xmscu_zvolume:
;	bbs0 (xms_cc_flags), _xmscu_zvol_set				; ??? IS THIS NECESSARY ANYMORE?
	mov y, #(%10000000 | $1E) ; 4ms
	mov a, x
	or a, #DSP_GAIN
	movw ($F2), ya
	jmp _xmscu_novolume
_xmscu_rvolume:
	or a, #8
	movz ($F2), a
	movz y, ($F3)
	dec a
	movw ($F2), ya
_xmscu_novolume:								;						}
;---------------------------------------------------------------------------------------------------------------------------
	
	mov1 c, (xms_cc_flags+(5<<13))				; 4 check sample offset (restore)	{ 32 cycles/8 cycles
	bcc _xmscu_nooffset2						; 2/4
	; so = xms_cc_sampoff						;
	; sampoff = source_offset + so*144			;
	pop a										; 4 pop hi
	mov y, #1									; 2 
	mov [int2]+y, a								; 7 restore
	pop a										; 4 pop lo
	dec y										; 2
	mov [int2]+y, a								; 7 restore
_xmscu_nooffset2:								;									}
	
	mov (xms_cc_flags), #0						; 5 reset flags						{ 10 cycles
	ret											; 5 return							}
	
XMS_CHANNEL_PITCH_SLIDEUP:								;[86 cycles]
	; a = amount - lo byte								;
	; (int2) = hi byte									;
	; x = channel * 16									;
	movz (int1+1), a									; 4 save lo
	movw ya, (xms_cc_period)							; 5 read period
	subw ya, (int1+1)									; 5 subtract
	call XMS_CHANNEL_SETPITCH							; 8+59 set pitch
	ret													; 5

XMS_CHANNEL_PITCH_SLIDEDOWN:							;[80 cycles]
	; a = amount - lo byte								;
	; (int2) = hi byte									;
	; x = channel * 16									;
	movz y, (int2)										; 3
	addw ya, (xms_cc_period)							; 5
	call XMS_CHANNEL_SETPITCH							; 8+59 set pitch
	ret													; 5

XMS_CHANNEL_SETPITCH:									;[? cycles]
	; ya = pitch										;
	call XMS_PeriodClip
	movw (xms_cc_period), ya							; 4 save period
	or (xms_cc_flags), #ccflag_pitch					; 5 set flag
														;
	movz (xms_channels+0)+x, a							; 5 store
	movz (xms_channels+1)+x, y							; 5
														;
	ret													; 5

XMS_CHANNEL_VOLSLIDEu:									;[31 cycles]
	; a = amount										;
	; x = channel * 16									;
	clrc												; 2
	adcz a, (xms_channels+6)+x							; 4 add amount
	cmp a, #64											; 2 clip
	bcc _xmscvsu_good									; 2/4
	mov a, #64											; 2
_xmscvsu_good:											; 
	movz (xms_channels+6)+x, a							; 5 save
	movz (xms_cc_volume), a								; 4 save
	or   (xms_cc_flags), #ccflag_volume					; 5 set flag
	ret													; 5

XMS_CHANNEL_VOLSLIDEd:									;[33 cycles]
	; a = amount										; 
	; x = channel * 16									; 
	setc												; 2
	sbcz a, (xms_channels+6)+x							; 4 reverse subtract
	eor a, #255											; 2
	inc a												; 2
	bpl _xmscvsd_good									; 2/4
	mov a, #0											; 2 clip
_xmscvsd_good:											; 
	movz (xms_channels+6)+x, a							; 5 save
	movz (xms_cc_volume), a								; 4 save
	or   (xms_cc_flags), #ccflag_volume					; 5 set flag
	ret													; 5

XMS_PeriodClip:
	nop		; <------- MODIFICATION SPACE (branch)
	nop		;

XMS_PeriodClipL:				;[25 cycles]
	; ya = period				;
	; returns clamped period	;
	
	cmp y, #((plimitl_lo>>8)+1)	; 2
	bcs _pc_pge_1600			; 2/4
	cmp a, #((plimitl_lo&255)	; 2
	bcc _pc_plt_1600			; 2/4
_pc_pge_1600:
	cmp y, #(plimitl_hi>>8)		; 2
	bcc _pc_p_good				; 2/4
	cmp a, #((plimitl_hi&255)+1); 2
	bcc _pc_p_good				; 2/4
	mov a, #(plimitl_hi&255)	;   2
	mov y, #(plimitl_hi>>8)		;   2
_pc_p_good:						;
	ret							; 5
	
_pc_plt_1600:					;
	mov a, #(plimitl_lo&255)	; 2
	mov y, #(plimitl_lo>>8)		; 2
	ret							; 5
	

XMS_PeriodClipA:
	; ya = period				;
	; returns clamped period	;
	cmp y, #0
	bmi _pca_plt_1600
	
	mov (int1),   #(plimita_lo&255)
	mov (int1+1), #(plimita_lo>>8)
	cmpw ya, (int1)
	bcs _pca_pge_1600
_pca_pge_1600:
	mov (int1),   #(plimita_hi&255)
	mov (int1+1), #(plimita_hi>>8)
	cmpw ya, (int1)
	bcc _pca_p_good
	mov a, #(plimita_hi&255)	;   2
	mov y, #(plimita_hi>>8)		;   2
_pca_p_good:						;
	ret							; 5
	
_pca_plt_1600:					;
	mov a, #(plimita_lo&255)	; 2
	mov y, #(plimita_lo>>8)		; 2
	ret							; 5
	
;============================================================================================
; ENVELOPE CONTROLLERS!
; ------------------------------------------------------------------
#ifdef FX_INCLUDE_ENVPOS
XMS_CHANNEL_ENV_SETPOSITION:				;[119 cycles]
	; int1 = position						;
	; x = channel * 16						;
	; return								;
	; x = preserved							;
	; position = 0-324						;
	; routine:								;
	;   set position						;
	;	DO NOT READ!						;
	
	; scan envelopes & save
	mov a, #c_inst_volenv					; 2			{ 114 cycles
	mov y, #0								; 2
	addw ya, (cur_inst)						; 5
	movw (int4), ya							; 4
	movz a, (int1)							; 3 save positions
	mov (xms_channels_other+2)+x, a			; 6
	call XMS_CHANNEL_ENVSCAN				; 8+52
	; y = point
	
	mov a, y								; 2
	clrc
	adc a, #c_inst_volenv
	mov (xms_channels_other+coso_epv)+x, a	; 6
	inc y									; 2
	mov a, [int4]+y							; 6
	dec y									; 2
	setc									; 2
	sbci a, [int4]+y						; 6
	mov (xms_channels_other+coso_eyv)+x, a	; 6			}
	
	ret										; 5			+5 return
#endif

;------------------------------------------------------------------
XMS_CHANNEL_ENV_RESET:						;[48 cycles]
	; no parameters
	;--------------
	; x is preserved
	mov a, #%10000000						; 2 set keyon
	or  a, (xms_channels_other+6)+x			; 5
	mov (xms_channels_other+6)+x, a			; 6
	mov a, #0								; 2 a = 0
	mov (xms_channels_other+2)+x, a			; 6 reset envelope X (volume)
	mov (xms_channels_other+4)+x, a			; 6 reset envelope X (pan)
	mov a, #c_inst_volenv					; 2 set envelope variables
	mov (xms_channels_other+coso_epv)+x, a	; 6
	mov a, #c_inst_panenv					; 2
	mov (xms_channels_other+coso_epp)+x, a	; 6
	ret										; 5 return

;==================================================================\
XMS_CHANNEL_ENV_KEYOFF:				;[18 cycles]				  \
	; x = channel * 16				; 							   \
	; x is preserved				; 							  \
	mov a, (xms_channels_other+6)+x	; 5	read flags				   \
	and a, #%01111111				; 2	reset keyon				  \
	mov (xms_channels_other+6)+x, a	; 6	save					   \
	ret								; 5							  \
;==================================================================\

;--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
; MAIN ENVELOPE ROUTINE
;--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

_xms_channel_env_dis:
	mov a, #0								; 2 set null values
	mov (xms_channels_other+2)+x, a			; 6
	mov (xms_channels_other+3)+x, a			; 6
	mov (xms_channels_other+4)+x, a			; 6
	mov a, #32								; 2
	mov (xms_channels_other+5)+x, a			; 6
	or (xms_cc_flags), #ccflag_volume		; 5
	ret										; 5

_xmscev_envelope_dis:
	mov a, #0								; 2 store default volume values (if disabled)
	mov (xms_channels_other+2)+x, a			; 6
	mov a, #64								; 2
	mov (xms_channels_other+3)+x, a			; 6
	jmp _xmscev_skipvol						; 3 skip

XMS_CHANNEL_ENV_ROUTINE:					;[628 cycles] :OO										<------------ ENTRY POINT				| HEAVY CPU ROUTINE
	; x = channel * 16						;																								| OPTIMIZATION: HIGH
	; x is preserved						;																								|
	; this routine reads values in envelope, and then increases the counters
											;
	movz a, (xms_channels+cso_inst)+x		; 4 check for valid instrument			{ 8 cycles
	cmp a, #0								; 2
											;
	beq _xms_channel_env_dis				; 2/4
											;										}
	; volume envelope & fadeout				;-----------------------------------------------------------------
	
	mov y, #c_inst_envflags					; 2 read envelope flags							{ 23 cycles
	mov a, [cur_inst]+y						; 6
	movz (int2), a							; 4 save for later
	
	and a, #1								; 2 check envelope_enable
	beq _xmscev_envelope_dis				; 2/4
	
	mov a, (xms_channels_other+2)+x			; 5 get volume x
	movz (int1), a							; 4 int1 = volume x								}
	
	mov y, #c_inst_nvpoints					; 2 compute y = env_offset+npoints-1			{ 12 cycles
	mov a, [cur_inst]+y						; 6
	
	push a									; 4												}
	
	mov (int7), #0							; 5												{ 194 cycles
	mov a, (xms_channels_other+coso_eyv)+x	; 5
	mov (int7+1), a							; 4
	mov a, (xms_channels_other+coso_epv)+x	; 5
	inc a									; 2
	mov y, a								; 2
	movz a, (int1)							; 3
	cmpi a, [cur_inst]+y					; 6
	bne _xmscev_readvol						; 2/4
	inc y									; 2
_xmscev_readvol:							;
	dec y									; 2
	mov a, y								; 2
	mov (xms_channels_other+coso_epv)+x, a	; 6
	
	pop y									; 4
	movz (byte1), a							; 4
	cmpz y, (byte1)							; 3
	
	beq _xmscev_quickvol					; 2/4
	call XMS_CHANNEL_ENV_READ				; 8+108
	
	mov (xms_channels_other+2)+x, a			; 6
	mov a, y								; 2
	mov (xms_channels_other+3)+x, a			; 6
	mov a, (xms_channels_other+6)+x			; 5												}
	
	bpl _xmscev_vol_nosustain				; 2/4 CHECK IF KEY IS ON (keyon bit is msb)		{ 34 cycles
	
	bbc1 (int2), _xmscev_vol_nosustain		; 5/7 check & branch if bit2 cleared
	
	mov a, (xms_channels_other+2)+x			; 5 compare x with sustain point
	mov y, #c_inst_volsus					; 2
	cmpi a, [cur_inst]+y					; 6
	beq _xmscev_skipvol						; 2/4
	b _xmscev_vol_inc						; 4
_xmscev_vol_nosustain:						; 
	mov a, (xms_channels_other+2)+x			; 5 load old value
_xmscev_vol_inc:							;
	inc a									; 2 increment position
	mov (xms_channels_other+2)+x, a			; 6
	b _xmscev_skipvol
_xmscev_quickvol:
	call XMS_CHANNEL_ENV_READ
	
	mov (xms_channels_other+2)+x, a			; save values
	mov a, y								;
	mov (xms_channels_other+3)+x, a			;
_xmscev_skipvol:							;												}

	mov a, (xms_channels_other+6)+x			; 5												{ 62 cycles
	
	bmi _xmscev_nofadeout					; 2/4 CHECK IF KEY IS OFF
	bbc0 (int2), _xmscev_fadeout_isover		; 5/7 check & branch if bit1 cleared
	
	;-----------------						; do some fadeout here
	setc									; 2 subtract fadeout value
	mov y, #c_inst_fadeout					; 2
	mov a, (xms_channels_other+0)+x			; 5
	sbci a, [cur_inst]+y					; 6
	push a									; 4
	inc y									; 2
	mov a, (xms_channels_other+1)+x			; 5
	sbci a, [cur_inst]+y					; 6
	bcs _xmscev_fadeout_noOver				; 2/4
	pop y									; 4	
_xmscev_fadeout_isover:
	
	mov a, #0								; 2 clip
	mov (xms_channels_other+0)+x, a			; 6
	mov (xms_channels_other+1)+x, a			; 6
	jmp _xmscev_nofadeout					; 3
_xmscev_fadeout_noOver:
	mov (xms_channels_other+1)+x, a			; 6 store
	pop a									; 4
	mov (xms_channels_other+0)+x, a			; 6
_xmscev_nofadeout:							;												}
	
	; Panning envelope ---------------------------------------------------------------------
	
	bbs3 (int2), _xmscev_pan_enabled		; 5/7 check & branch if bit4 set			{ 33 cycles
	mov a, #0								; 2 default values
	mov (xms_channels_other+4)+x, a			; 6
	mov a, #32								; 2
	mov (xms_channels_other+5)+x, a			; 6
	b _xmscev_skippan						; 4 skip
_xmscev_pan_enabled:
	
	or  (xms_cc_flags), #ccflag_panning		; 5 panning changes, set flag
	
	mov y, #c_inst_nppoints					; 2 compute y = env_offset+nppoints-1 
	mov a, [cur_inst]+y						; 6 
	push a									; 4
	
	mov a, (xms_channels_other+4)+x			; 5
	movz (int1), a							; 4											}
	
	mov (int7), #1							; 5 panning mode							{ 190 cycles
	mov a, (xms_channels_other+coso_eyp)+x	; 5
	mov (int7+1), a							; 4
	mov a, (xms_channels_other+coso_epp)+x	; 5
	inc a									; 2
	mov y, a								; 2
	movz a, (int1)							; 4
	cmpi a, [cur_inst]+y					; 6
	bne _xmscer_readpan						; 2/4
	inc y									; 2
_xmscer_readpan:							;
	dec y									; 2
	mov a, y								; 2
	mov (xms_channels_other+coso_epp)+x, a	; 6
	pop y									; 4
	movz (byte1), a							; 4
	cmpz y, (byte1)							; 3
	beq _xmscev_quickpan					; 2/4
	call XMS_CHANNEL_ENV_READ				; 8+108
	
	
	mov (xms_channels_other+4)+x, a			; 6 save values
	mov a, y								; 2
	mov (xms_channels_other+5)+x, a			; 6
											; ------------------------------------		}
	mov a, (xms_channels_other+6)+x			; 5											{ 39 cycles
	bpl _xmscev_pan_nosustain				; 2/4 CHECK IF KEY IS ON
											;
	bbc4 (int2), _xmscev_pan_nosustain		; 5/7 check & branch if bit5 set
	
	mov a, (xms_channels_other+4)+x			; 5 read panning envelope position
	mov y, #c_inst_pansus					; 2
	cmpi a, [cur_inst]+y					; 6 compare with sustain
	
	beq _xmscev_skippan						; 2/4 skip
	b _xmscev_pan_inc						; 4
_xmscev_pan_nosustain:						;
	
	mov a, (xms_channels_other+4)+x			; 5 read panning envelope position
_xmscev_pan_inc:							;
	
	inc a									; 2 increment
	mov (xms_channels_other+4)+x, a			; 6 save
	b _xmscev_skippan
_xmscev_quickpan:
	call XMS_CHANNEL_ENV_READ
	mov (xms_channels_other+4)+x, a			; save values
	mov a, y								;
	mov (xms_channels_other+5)+x, a			;
_xmscev_skippan:							;											}
	
	ret										; 5											} 5 cycles

;=======================================================================================,
XMS_CHANNEL_ENV_READ:						;[108 cycles]
	; x = channel * 8
	; int1 = x pos
	; int7+1 = special optimization
	; cur_inst = inst_offset
	; int7 = PANNING mode
	; a = env_offset + point
	;----------------------------
	; ya = envelope value
	; this function updates values in channel other info
	
	clrc									; 2									{ 25 cycles
	mov y, a								; 2
	adc a, #12								; 2
	movz (byte1), a							; 4
	movz a, (int1)							; 3
	setc									; 2
	sbci a, [cur_inst]+y					; 6
	
	; a = cx = x-point.x					;
	bne _xmscer_cxnot0						; 2/4								} branches
_xmscer_zero:
	; cx is zero, no interpolation needed
	bbs0 (int7), _xmscer_panningl			; 5/7																{ 82 cycles
_xmscer_volumel:
	movz a, (int1)							; 3									{ 70 cycles
	mov y, #c_inst_volloope					; 2
	cmpi a, [cur_inst]+y					; 6
	bne _xmscer_noloop						; 2/4
	
	mov y, #c_inst_volloopsp				; 2
	mov a, [cur_inst]+y						; 6
	mov (xms_channels_other+coso_epv)+x, a	; 6
	mov y, #c_inst_volloopsy				; 2
	mov a, [cur_inst]+y						; 6
	mov (xms_channels_other+coso_eyv)+x, a	; 6
	mov y, #c_inst_volloopsv				; 2
	mov a, [cur_inst]+y						; 6
	push a									; 4
	mov y, #c_inst_volloopsx				; 2
	mov a, [cur_inst]+y						; 6
	
	pop y									; 4
	ret										; 5									}
	;---------------------------------------------------------------------------------------------------------------------------------------
_xmscer_panningl:
	movz a, (int1)							; 3									{ 70 cycles
	mov y, #c_inst_panloope					; 2
	cmpi a, [cur_inst]+y					; 6
	bne _xmscer_noloop						; 2/4
	
	mov y, #c_inst_panloopsp				; 2
	mov a, [cur_inst]+y						; 6
	mov (xms_channels_other+coso_epp)+x, a	; 6
	mov y, #c_inst_volloopsy				; 2
	mov a, [cur_inst]+y						; 6
	mov (xms_channels_other+coso_eyp)+x, a	; 6
	mov y, #c_inst_panloopsv				; 2
	mov a, [cur_inst]+y						; 6
	push a									; 4
	mov y, #c_inst_panloopsx				; 2
	mov a, [cur_inst]+y						; 6
	
	pop y									; 4
	ret										; 5									}								}
_xmscer_noloop:
	setc									; 2									{ 58 cycles
	movz a, (byte1)							; 3
	sbc a, #11								; 2
	mov y, a								; 2
	mov a, [cur_inst]+y						; 6
	dec y									; 2
	sbci a, [cur_inst]+y					; 6
	bbs0 (int7), _xmscer_panning3			; 5/7
	mov (xms_channels_other+coso_eyv)+x, a	; 6
	b _xmscer_volume3						; 4
_xmscer_panning3:							;
	mov (xms_channels_other+coso_eyp)+x, a	;   6
_xmscer_volume3:							;----
	
	movz y, (byte1)							; 2
	mov a, [cur_inst]+y						; 6
	mov y, a								; 4 restore
	movz a, (int1)							; 3 load
	ret										; 5 exit							}
_xmscer_cxnot0:
	; cx isn't zero					; newsflash: current point is NOT LAST
	; interpolation is needed       ; current position is NOT zero difference from point
	; int3 = point1 x
	
	movz (int1+1), a			; 4							{ 31 cycles		{ 83 cycles
	push x						; 4
	
	movz y, (byte1)				; 3 p1.y - p2.y
	mov a, [cur_inst]+y			; 6
	
	movz (byte1), a				; 4
	inc y						; 2
	sbci a, [cur_inst]+y		; 6
	
	bpl _xmscer_iplus			; 2/4						} minus
_xmscer_ineg:
	eor a, #255					; 2							{ 38 cycles
	inc a						; 2
	movz y, (int1+1)			; 3
	mul ya						; 9
	movz x, (int7+1)			; 3
	div ya, x					; 12
	adcz a, (byte1)				; 3
	jmp _xmscer_exit			; 4							}
_xmscer_iplus:
	movz y, (int1+1)			; 3						{ 34 cycles
	mul ya						; 9
	movz x, (int7+1)			; 3
	div ya, x					; 12
	sbcz a, (byte1)				; 3
	eor a, #255					; 2
	inc a						; 2						}
_xmscer_exit:

	pop x						; 4							{ 14 cycles
	mov y, a					; 2
	movz a, (int1)				; 3

	ret							; 5							}				}

#ifdef FX_INCLUDE_ENVPOS
;===================================================================================================,
XMS_CHANNEL_ENVSCAN:								; [52 cycles]									;
	; a      = x position							;												;//
	; (int4) = env offset							;												;//
	; clobbering:									;												;//
	; returns:										;												;//
	; y = result (point)							;												;//
													;												'===========================,
_xmscev_start:										;																			;
	mov y, #6										; 2		check >= 6															;//
	cmpi a, [int4]+y								; 6																			;//
	bcs _xmscev_g6									; 2/4																		;//
_xmscev_l6:											;		( < 6 )																;//
	mov y, #3										; 2																			;//
	cmpi a, [int4]+y								; 6		check >= 3															;//
	bcs _xmscev_g3									; 2/4																		;//
	dec y											; 2		( < 3 )																;//
	cmpi a, [int4]+y								; 6		check == 2															;//
	bcs _xmscev_ret									; 2/4																		;//
	dec y											; 2																			;//
	cmpi a, [int4]+y								; 6		check == 1															;//
	bcs _xmscev_ret									; 2/4																		;//
	dec y											; 2		answer = 0															;//
	ret												;																			;//
_xmscev_g3:											;																			;//
	mov y, #5										; 2		( >= 3 )															;//
	cmpi a, [int4]+y								; 6		check == 5															;//
	bcs _xmscev_ret									; 2/4																		;//
	dec y											; 2																			;//
	cmpi a, [int4]+y								; 6		check == 4															;//
	bcs _xmscev_ret									; 2/4																		;//
	dec y											; 2		answer = 3															;//
	ret												;																			;//
_xmscev_g6:											;																			;//
	mov y, #9										; 2		( >= 6 )															;//
	cmpi a, [int4]+y								; 6		check >= 9															;//
	bcs _xmscev_g9									; 2/4																		;//
	dec y											; 2		( < 9 )																;//
	cmpi a, [int4]+y								; 6		check == 8															;//
	bcs _xmscev_ret									; 2/4																		;//
	dec y											; 2																			;//
	cmpi a, [int4]+y								; 6		check == 7															;//
	bcs _xmscev_ret									; 2/4																		;//
	dec y											; 2		answer = 5															;//
	ret												;																			;//
_xmscev_g9:											;																			;//
	mov y, #11										; 2		( >= 9 )															;//
	cmpi a, [int4]+y								; 6		check == 11															;//
	bcs _xmscev_ret									; 2/4																		;//
	dec y											; 2																			;//
	cmpi a, [int4]+y								; 6		check == 10															;//
	bcs _xmscev_ret									; 2/4																		;//
	dec y											; 2		answer = 9															;//
_xmscev_ret:										;																			;//
	ret												; 5																			;//
																																;//
;==============================================================================================================================='//
  ;////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ;////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif

;--------------------------------------------------------------------------------------------------------
;  VIBRATO ROUTINES!
;--------------------------------------------------------------------------------------------------------
XMS_AV_UPDATE:												;[213 cycles]												|
	clrc													; 2											{ 43 cycles		| COMMON ROUTINE
	mov a, (xms_channels_other+7)+x							; 5															| OPTIMIZATION: MED/LOW
	mov y, #c_inst_vibsweep									; 2															|
	adci a, [cur_inst]+y									; 6
	mov (xms_channels_other+7)+x, a							; 6
	inc y													; 2
	mov a, [cur_inst]+y										; 6
	adcz a, (xms_channels+cso_avsweep)+x					; 5
	bcc _xmsavu_noover										; 2/4
_xmsavu_nosweep:											; 
	mov a, #255												; 2 clip
_xmsavu_noover:												; 
	movz (xms_channels+cso_avsweep)+x, a					; 5 save									}
	
_xmsavu_skip_sweep:
	
	mov y, #c_inst_vibtype									; 2 type...												{ 63 cycles 
	mov a, [cur_inst]+y										; 6
	push a													; 4 save
															;
	mov y, #c_inst_vibrate									; 2 rate...
	mov a, [cur_inst]+y										; 6
	movz (int1+1), a										; 4
															;
	dec y													; 2 depth -- dont change instrument structure between depth&rate
	mov a, [cur_inst]+y										; 6
	cmp a, #0												; 2		exit if depth=0
	beq _xmsav_exit											; 2/4
	mov y, a				; depth							; 2
															;
	movz a, (xms_channels+cso_avsweep)+x					; 4
	cmp a, #255												; 2
	beq _xmsfxv_fullav										; 2/4 ----------------------------------------
	mul ya													;		9
	movz (int1), y											;		4
															;
	b _xmsfxv_hasavd										;		4
_xmsfxv_fullav:												;----------------------------------------------
	movz (int1), y											;					4
_xmsfxv_hasavd:												;----------------------------------------------			}
	
	movz a, (int1+1)							; 3										{ 107 cycles
	
	clrc										; 2
	adcz a, (xms_channels+cso_avsine)+x			; 4 increment sine pos
	movz (xms_channels+cso_avsine)+x, a			; 5
	
	pop y										; 4
	call XMS_GETVIBTABLE						; 8+38 get table value
	
	mov y, a									; 2 multiply depth * "sine"
	movz a, (int1)								; 3
	mul ya										; 9
	
	mov1 c, (xms_cc_flags+(7<<13))				; 4 read sign			{ 20 cycles
	
	bcs _xmsav_set_neg							; 2/4
	mov a, y									; 2
_xmsav_0fix:
	mov y, #0									; 2
	movw (xms_cc_perioda), ya					; 4 set perioda....
	b _xmsav_set_pos							; 4
_xmsav_set_neg:
	
	mov a, y									; 2
	eor a, #255									; 2
	inc a										; 2
	beq _xmsav_0fix								; 2/4
	mov y, #255									; 2
	movw (xms_cc_perioda), ya					; 4
_xmsav_set_pos:									;						}
	
	set1 (xms_cc_flags)							; 4 set flag
	ret											; 5 exit								}
	
_xmsav_exit:
	mov (xms_cc_perioda), #0					; 5
	mov (xms_cc_perioda+1), #0					; 5
	pop a										; 4 restore
	ret											; 5 exit
	
table_mod_sine:
.byte	0, 24, 49, 74, 97,120,141,161,
.byte	180,197,212,224,235,244,250,253,
.byte	255,253,250,244,235,224,212,197,
.byte	180,161,141,120, 97, 74, 49, 24

;--------------------------------------------------------------------------------------------
XMS_GETVIBTABLE:						;								; 38 cycles maximum ;
	; a = position (0-255)				;								;--------------------
	; y = table (0-3)					;								; optimization: med/high
	; return:							;								;
	; a = value							;								;
	; sets xms_cc_flags +/- PA bit		;								;
	cmp y, #3							; 2			12 cycles			;
	beq _xmsgvt_T3						; 2/4							;
	cmp y, #2							; 2								;
	beq _xmsgvt_T2						; 2/4							;
	cmp y, #1							; 2								;
	beq _xmsgvt_T1						; 2/4							;
										;--------------------------------
_xmsgvt_T0:			; sine				;								;
	cmp a, #128							; 2         26 cycles			;
	mov1 (xms_cc_flags+(7<<13)), c		; 6								;
	lsr a								; 2								;
	lsr a								; 2								;
	and a, #31							; 2								;
	mov y, a							; 2								;
	mov a, (table_mod_sine)+y			; 5								;
										; 								;
	ret									; 5								;
_xmsgvt_T1:			; square			;--------------------------------
	cmp a, #128							; 2			15 cycles			;
	mov1 (xms_cc_flags+(7<<13)), c		; 6								;
	mov a, #255							; 2								;
	ret									; 5								;
_xmsgvt_T2:			; ramp down			;--------------------------------
	cmp a, #128							; 2			8 cycles	(28)	;
	mov1 (xms_cc_flags+(7<<13)), c		; 6								;
										;								;
_xmsgvt_rd_hack:						;								;
										;								;
	bcs _xmsgvt_rd_128					;---- 2/4	11 cycles			;
											; 							;
	asl a				; clears carry...	; 2							;
	eor a, #255								; 2							;
	ret										; 5							;
_xmsgvt_rd_128:								;---------					;
											;							;
	; ...carry is set						;							;
	sbc a, #128			; a = 0 to 127		; 2		9 cycles (+2 jump)	;
	asl a				; a = 0 to 254		; 2							;
	ret										; 5							;
											;							;
_xmsgvt_T3:			; ramp up				;----------------------------
	cmp a, #128								; 2		26 cycles			;
	notc									; 2							;
	mov1 (xms_cc_flags+(7<<13)), c			; 6							;
	notc									; 2							;
	jmp _xmsgvt_rd_hack						; 3+11						;
											;							;
;------------------------------------------------------------------------

; draft 2
XMS_VOLUMERAMP:						;[269 cycles]
	movz a, xms_playing
	bne _xms_vrok
	ret
_xms_vrok:
	mov x, #8						; 2		{ 7 cycles
	mov ($F2), #$77					; 5		}
_xms_vrloop:						;
	movz a, ($F3)					; 3		{ 15 cycles			{ 257 cycles
	bmi _xms_vrnone					; 2/4
	lsr a
	cmpz a, (xms_final_volume-1)+x	; 4
	beq _xms_vrnoneC				; 2/4
	bcs _xms_vrgreat				; 2/4	}
_xms_vrless:						;
	inc a							; 2		{ 17 cycles
	asl a
	movz ($F3), a					; 4
	sbc ($F2), #$0F					; 5 <-- carry was cleared
	dec x							; 2
	bne _xms_vrloop					; 2/4
	ret								;		}
_xms_vrgreat:						;
	dec a							; 2		{ 17 cycles
	asl a
	movz ($F3), a					; 4
	sbc ($F2), #$0F					; 5 <-- carry was cleared
	dec x							; 2
	bne _xms_vrloop					; 2/4	}					}
_xms_vrbreak:
	ret								; 5

_xms_vrnone:
	setc
_xms_vrnoneC:
	sbc ($F2), #$10
	dec x
	bne _xms_vrloop
	ret

XMS_CHECKSFX:
	mov x, #7						; setup loop
	mov ($F2), #$7C					; read from ENDX
_xmsfxc_loop:
	movz a, (samp_playing)+x		; load sfx value
	beq _xmsfxc_zero				; skip if already 0
	mov  a, (XMS_TABLE_BITS)+x		; translate to bit
	andz a, ($F3)					; mask with ENDX
	beq _xmsfxc_zero				; skip if zero
	mov a, #0						; clear sfx value
	mov (samp_playing)+x, a			; ...
_xmsfxc_zero:
	dec x							; loop
	bpl _xmsfxc_loop

	ret

XMS_TABLE_BITS:
.byte %1, %10, %100, %1000, %10000, %100000, %1000000, %10000000

.END

EOF
