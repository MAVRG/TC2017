!*****************************************************************************
!   SH7045マイコンボード　スタートアップルーチン
!                                          (C) 30Nov2003 T.KOSAKA, A.YAMASHITA
!  
!  このスタートアップで行なっていること
!  （１）スタックポインタの初期化
!  （２）マルチプレックスピンの初期化
!      （アドレスとポート共用のピンをアドレスとして使えるようにするなど）
!  （３）変数領域への初期値のコピー
!  （４）初期値を持たない変数領域のクリア（これはサービス）
!  （５）main()の呼び出しと直後に無限ループの設置
!  このスタートアップルーチンファイルでのサービス関数
!  （１）void setIntMask(int mask)　割り込みマスクの設定関数
!  （２）int getIntMask(void)　割り込みマスクの読み出し関数
!  （３）__main()　ダミー関数
!      これを定義しておくと予期しないライブラリ関数が付加されることがない
!*****************************************************************************
    .section    .text          ! スタートアップルーチンをROM領域に書き込み
    .global     _start         ! プログラム開始位置をエクスポート
    .extern     _dataRAM_begin
    .extern     _dataRAM_end
    .extern     _dataROM_begin
    .extern     _bss_begin
    .extern     _bss_end

_start:                                 ! プログラム開始位置
!-----------------------------------------------------------------------------
! スタックポインタ設定
!
! 汎用レジスタR0～R15のうちR15がハードウェアスタックポインタに指定されている 
! アドレスは 0xFFFFF000～0xFFFFFFFF の内臓RAMを使用するため、 
! STACK_ROOTは0x0を入れておく 
!-----------------------------------------------------------------------------
    MOV.L    STACK_ROOT,    r15
!-----------------------------------------------------------------------------
! 各ポート初期化
! 秋月製のマイコンキットでは標準で外部RAMが実装されている．mode2にセットする
!-----------------------------------------------------------------------------
    MOV.L       BCR1,     r1         ! BCR1   -> r1
    MOV.W       D_BCR1,   r0         ! D_BCR1 -> r0
    MOV.W       r0,       @r1
    MOV.L       BCR2,     r1         ! BCR2   -> r1
    MOV.W       D_BCR2,   r0         ! D_BCR2 -> r0
    MOV.W       r0,       @r1
    MOV.L       WCR1,     r1         ! WCR1   -> r1
    MOV.W       D_WCR1,   r0         ! D_WCR1 -> r0
    MOV.W       r0,       @r1
    MOV.L       PACRH,    r1         ! PACRH   -> r1
    MOV.W       D_PACRH,  r0         ! D_PACRH -> r0
    MOV.W       r0,       @r1
    MOV.L       PACRL1,   r1         ! PACRL1   -> r1
    MOV.W       D_PACRL1, r0         ! D_PACRL1 -> r0
    MOV.W       r0,       @r1
    MOV.L       PBCR1,    r1         ! PBCR1   -> r1
    MOV.W       D_PBCR1,  r0         ! D_PBCR1 -> r0
    MOV.W       r0,       @r1
    MOV.L       PBCR2,    r1         ! PBCR2   -> r1
    MOV.W       D_PBCR2,  r0         ! D_PBCR2 -> r0
    MOV.W       r0,       @r1
    MOV.L       PCCR,     r1         ! PCCR   -> r1
    MOV.W       D_PCCR,   r0         ! D_PCCR -> r0
    MOV.W       r0,       @r1
    MOV.L       PDCRH1,   r1         ! PDCRH1   -> r1
    MOV.W       D_PDCRH1, r0         ! D_PDCRH1 -> r0
    MOV.W       r0,       @r1
    MOV.L       PDCRH2,   r1         ! PDCRH2   -> r1
    MOV.W       D_PDCRH2, r0         ! D_PDCRH2 -> r0
    MOV.W       r0,       @r1
    MOV.L       PDCRL,    r1         ! PDCRL   -> r1
    MOV.W       D_PDCRL,  r0         ! D_PDCRL -> r0
    MOV.W       r0,       @r1
    NOP

!-----------------------------------------------------------------------------
! ROMに格納されているデータ(変数など)をRAM領域にコピーする
!   RAM_BGN  : データを格納するRAM領域の開始アドレス
!   RAM_END : データを格納するRAM領域の終了アドレス
!   ROM_BGN : 実際にデータが書き込まれているROM領域の開始アドレス
!-----------------------------------------------------------------------------
    MOV.L       RAM_BGN,  r0        ! RAM_BGN -> r0
    MOV.L       RAM_END,  r1        ! RAM_END -> r1
    MOV.L       ROM_BGN,  r2        ! ROM_BGN -> r2
    BRA         LOOP11
    NOP
LOOP1:
    MOV.B       @r2,    r3          ! ROM領域からデータを取得
    MOV.B       r3,     @r0         ! RAM領域へデータをコピー
    ADD         #1,     r0          ! RAM領域を指すアドレスをインクリメント
    ADD         #1,     r2          ! ROM領域を指すアドレスをインクリメント
LOOP11:
    CMP/eq      r0,     r1          ! RAM_BGN == RAM_END ならば T=1
    BF          LOOP1               ! T=1 ならば LOOP1へ
    NOP
END_LOOP1:

!-----------------------------------------------------------------------------!
!   初期化無しのグローバル変数を0で初期化
!   BSS_BGN  : 初期化無しのグローバル変数が格納されている開始アドレス
!   BSS_END : 初期化無しのグローバル変数が格納されている終了アドレス
!-----------------------------------------------------------------------------
    MOV.L       BSS_BGN,  r0        ! BSS_BGN -> r0
    MOV.L       BSS_END,  r1        ! BSS_END -> r1
    BRA         LOOP21
    NOP
LOOP2:
    MOV         #0,     r3          ! r3=0
    MOV.B       r3,     @r0         ! グローバル変数の領域に0を代入
    ADD         #1,     r0          ! 領域を指すアドレスをインクリメント
LOOP21:
    CMP/eq      r0,     r1          ! BSS_BGN == BSS_END ならば T=1
    BF          LOOP2               ! T=1 ならば LOOP2へ
    NOP
END_LOOP2:

!-----------------------------------------------------------------------------
! main関数呼び出し
!-----------------------------------------------------------------------------
START_MAIN:                         ! mainをCALL
    MOV.L       MAIN_ADRS, r0       ! main関数のアドレス -> R0
    JSR         @r0                 ! mainへのジャンプサブルーチン
    OR          r0,     r0          ! JSR命令は遅延分岐のため
                                    ! (遅延分岐とはJSR直後の命令を先に実行)

! 万が一mainが終了して戻ってきても無限ループにして停止させる 
FOREVER:
    BRA         FOREVER
    OR          r0,     r0

.align  4                               ! 4Byte = 32Bit固定
!----実行時に参照される初期値有りデータの領域（RAM領域）----
RAM_BGN:            .long   _dataRAM_begin    ! 開始アドレス 
RAM_END:            .long   _dataRAM_end      ! 終了アドレス 
!----リンク時に参照される初期値有りデータの初期値領域（ROM領域）----
ROM_BGN:            .long   _dataROM_begin    ! 開始アドレス 
!----実行時に参照される初期値なしデータの領域（RAM領域）----
BSS_BGN:            .long   _bss_begin        ! 開始アドレス 
BSS_END:            .long   _bss_end          ! 終了アドレス 

STACK_ROOT:         .long   0x0               !スタックポインタの初期値
MAIN_ADRS:          .long   _main             !main関数

    .align  4
BCR1:    .long 0xffff8620
BCR2:    .long 0xffff8622
WCR1:    .long 0xffff8624
PACRH:   .long 0xffff8388
PACRL1:  .long 0xffff838c
PBCR1:   .long 0xffff8398
PBCR2:   .long 0xffff839a
PCCR:    .long 0xffff839c
PDCRH1:  .long 0xffff83a8
PDCRH2:  .long 0xffff83aa
PDCRL:   .long 0xffff83ac

!----外付けカウンタICの設定(CS0)はここでする ----
D_BCR1:    .short 0x203f ! CS2/CS3 are 16bit-bus,CS0,CS1 is 32 bit
                         ! 0010 0000 0011 1111
D_BCR2:    .short 0x5610 ! CS1/CS2/CS3 wait 1 idle,CS0 wait 2 idle
                         ! 0101 0110 0001 0000
D_WCR1:    .short 0x0002 ! CS0 is 2 wait, CS1-3 are 0 wait
                         ! 0000 0000 0000 0010
D_PACRH:   .short 0x5020 ! WRHH WRHL DRAK0 are enable
                         ! 0101 0000 0010 0000
D_PACRL1:  .short 0x5550 ! CK -RD -WRH -WRL -CS[1..0] are enable
                         ! 0101 0101 0101 0000
D_PBCR1:   .short 0x000a ! A21/A20 are enable
D_PBCR2:   .short 0xa005 ! A19/A18/A17/16 are enable
D_PCCR:    .short 0xffff ! A15-0 are enable
D_PDCRH1:  .short 0x5555 ! D31-24 are enable
D_PDCRH2:  .short 0x5555 ! D23-16 are enable
D_PDCRL:   .short 0xffff ! D15-0 are enable

! void setIntMask(int mask)   r4:mask r2:work r1:srreg
    .align  4
    .global	_setIntMask
_setIntMask:
    stc      sr,r1          ! srreg = __sr__
    mov.l    MASKVALUER,r2
    and      r2,r1          ! srreg &= 0xffffff0f
    shll2    r4             ! mask <<= 2
    shll2    r4             ! mask <<= 2
    mov.l    MASKVALUE,r2
    and      r2,r4          ! mask &= 0x00f0
    or       r4,r1          ! srreg |= mask
    ldc      r1,sr 
    rts	
    nop

! この関数はIntMaskを返します。 
! int getIntMask(void)
    .align  4
    .global	_getIntMask
_getIntMask:
    stc      sr,r0
    mov.l    MASKVALUE,r2
    and      r2,r0
    shlr2    r0
    shlr2    r0
    rts	
    nop

    .align  4
MASKVALUE:    .long  0x000000f0
MASKVALUER:   .long  0xffffff0f

! 次の関数はリンカがデフォルト関数群をつけるのを防ぎます 
    .align  4
    .global ___main
___main:
    rts	
    nop



.end
	