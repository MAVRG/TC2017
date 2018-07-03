/*****************************************************************************
         リンカスクリプト　1Dec2003 T.kosaka A.Yamashita 

機械語のプログラムやデータの配置（どこのアドレスに配置するか）を決めます

キーワード
OUTPUT_ARCH マシンのアーキテクチャ
ENTRY       プログラムの中で実行する最初の命令の位置を設定
MEMORY      メモリブロックの位置とサイズを記述
SECTIONS    出力ファイルのメモリレイアウトを記述

セクションの名前
.vector  割り込みベクタテーブル
.text    プログラム機械語コード，定数（静的文字列を含む）
.data    外部変数，static変数で初期化（宣言時に値が代入されている）済の変数
.bss     外部変数，static変数で初期化なしの変数
例
int aaa=1234;                   aaaは.dataに確保される
int bbb;                        bbbは.bssに確保される
const int bbb1=5678;            bbb1は.textに確保される

main()
{
    int ccc;                    cccはスタックに動的に確保される
      :
}

int func(void)
{
    static int ddd=0;           dddは.dataに確保される
    static int eee;             eeeは.bssに確保される
    int fff;                    fffはスタックに動的に確保される
      :
    fff += 2345;                2345は.textに確保される
      :
}
******************************************************************************/
OUTPUT_ARCH(sh)
ENTRY("_start")
MEMORY
{
    /* 内臓ROM 0x00000000～0x0003ffff */
    /* 外部RAM 0x00400000～0x0041ffff */
    /* 内臓I/O 0xffff8000～0xffff87ff */
    /* 内臓RAM 0xfffff000～0xffffffff */

    vect(r)         : org = 0x00000000, len = 0x280
    dtc_vect(r)     : org = 0x00000400, len = 0x60
    rom(rx)         : org = 0x00000480, len = 256k-0x280
    ram2(rwx)       : org = 0x00400000, len = 1024k
    ram(rwx)        : org = 0xfffff000, len = 4k
}
SECTIONS
{
    .vector : {
        /* 0 */
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(0))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(0))
        /* 4 */
        LONG(DEFINED(_int_GeneralIllegalInstruction)?ABSOLUTE(_int_GeneralIllegalInstruction):ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(DEFINED(_int_SlotIllegalInstruction)?ABSOLUTE(_int_SlotIllegalInstruction):ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 8 */
        LONG(ABSOLUTE(_start))
        LONG(DEFINED(_int_CPUAddressError)?ABSOLUTE(_int_CPUAddressError):ABSOLUTE(_start))
        LONG(DEFINED(_int_DMAAddressError)?ABSOLUTE(_int_DMAAddressError):ABSOLUTE(_start))
        LONG(DEFINED(_int_NMI)?ABSOLUTE(_int_NMI):ABSOLUTE(_start))
        /* 12 */
        LONG(DEFINED(_int_Userbrake)?ABSOLUTE(_int_Userbrake):ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 16 */
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 20 */
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 24 */
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 28 */
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 32 */
        LONG(DEFINED(_int_trap0)?ABSOLUTE(_int_trap0):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap1)?ABSOLUTE(_int_trap1):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap2)?ABSOLUTE(_int_trap2):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap3)?ABSOLUTE(_int_trap3):ABSOLUTE(_start))
        /* 36 */
        LONG(DEFINED(_int_trap4)?ABSOLUTE(_int_trap4):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap5)?ABSOLUTE(_int_trap5):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap6)?ABSOLUTE(_int_trap6):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap7)?ABSOLUTE(_int_trap7):ABSOLUTE(_start))
        /* 40 */
        LONG(DEFINED(_int_trap8)?ABSOLUTE(_int_trap8):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap9)?ABSOLUTE(_int_trap9):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap10)?ABSOLUTE(_int_trap10):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap11)?ABSOLUTE(_int_trap11):ABSOLUTE(_start))
        /* 44 */
        LONG(DEFINED(_int_trap12)?ABSOLUTE(_int_trap12):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap13)?ABSOLUTE(_int_trap13):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap14)?ABSOLUTE(_int_trap14):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap15)?ABSOLUTE(_int_trap15):ABSOLUTE(_start))
        /* 48 */
        LONG(DEFINED(_int_trap16)?ABSOLUTE(_int_trap16):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap17)?ABSOLUTE(_int_trap17):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap18)?ABSOLUTE(_int_trap18):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap19)?ABSOLUTE(_int_trap19):ABSOLUTE(_start))
        /* 52 */
        LONG(DEFINED(_int_trap20)?ABSOLUTE(_int_trap20):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap21)?ABSOLUTE(_int_trap21):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap22)?ABSOLUTE(_int_trap22):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap23)?ABSOLUTE(_int_trap23):ABSOLUTE(_start))
        /* 56 */
        LONG(DEFINED(_int_trap24)?ABSOLUTE(_int_trap24):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap25)?ABSOLUTE(_int_trap25):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap26)?ABSOLUTE(_int_trap26):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap27)?ABSOLUTE(_int_trap27):ABSOLUTE(_start))
        /* 60 */
        LONG(DEFINED(_int_trap28)?ABSOLUTE(_int_trap28):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap29)?ABSOLUTE(_int_trap29):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap30)?ABSOLUTE(_int_trap30):ABSOLUTE(_start))
        LONG(DEFINED(_int_trap31)?ABSOLUTE(_int_trap31):ABSOLUTE(_start))
        /* 64 */
        LONG(DEFINED(_int_irq0)?ABSOLUTE(_int_irq0):ABSOLUTE(_start))
        LONG(DEFINED(_int_irq1)?ABSOLUTE(_int_irq1):ABSOLUTE(_start))
        LONG(DEFINED(_int_irq2)?ABSOLUTE(_int_irq2):ABSOLUTE(_start))
        LONG(DEFINED(_int_irq3)?ABSOLUTE(_int_irq3):ABSOLUTE(_start))
        /* 68 */
        LONG(DEFINED(_int_irq4)?ABSOLUTE(_int_irq4):ABSOLUTE(_start))
        LONG(DEFINED(_int_irq5)?ABSOLUTE(_int_irq5):ABSOLUTE(_start))
        LONG(DEFINED(_int_irq6)?ABSOLUTE(_int_irq6):ABSOLUTE(_start))
        LONG(DEFINED(_int_irq7)?ABSOLUTE(_int_irq7):ABSOLUTE(_start))
        /* 72 */
        LONG(DEFINED(_int_dei0)?ABSOLUTE(_int_dei0):ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 76 */
        LONG(DEFINED(_int_dei1)?ABSOLUTE(_int_dei1):ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 80 */
        LONG(DEFINED(_int_dei2)?ABSOLUTE(_int_dei2):ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 84 */
        LONG(DEFINED(_int_dei3)?ABSOLUTE(_int_dei3):ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 88 */
        LONG(DEFINED(_int_tgi0a)?ABSOLUTE(_int_tgi0a):ABSOLUTE(_start))
        LONG(DEFINED(_int_tgi0b)?ABSOLUTE(_int_tgi0b):ABSOLUTE(_start))
        LONG(DEFINED(_int_tgi0c)?ABSOLUTE(_int_tgi0c):ABSOLUTE(_start))
        LONG(DEFINED(_int_tgi0d)?ABSOLUTE(_int_tgi0d):ABSOLUTE(_start))
        /* 92 */
        LONG(DEFINED(_int_tci0v)?ABSOLUTE(_int_tci0v):ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 96 */
        LONG(DEFINED(_int_tgi1a)?ABSOLUTE(_int_tgi1a):ABSOLUTE(_start))
        LONG(DEFINED(_int_tgi1b)?ABSOLUTE(_int_tgi1b):ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 100 */
        LONG(DEFINED(_int_tci1v)?ABSOLUTE(_int_tci1v):ABSOLUTE(_start))
        LONG(DEFINED(_int_tci1u)?ABSOLUTE(_int_tci1u):ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 104 */
        LONG(DEFINED(_int_tgi2a)?ABSOLUTE(_int_tgi2a):ABSOLUTE(_start))
        LONG(DEFINED(_int_tgi2b)?ABSOLUTE(_int_tgi2b):ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 108 */
        LONG(DEFINED(_int_tci2v)?ABSOLUTE(_int_tci2v):ABSOLUTE(_start))
        LONG(DEFINED(_int_tci2u)?ABSOLUTE(_int_tci2u):ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 112 */
        LONG(DEFINED(_int_tgi3a)?ABSOLUTE(_int_tgi3a):ABSOLUTE(_start))
        LONG(DEFINED(_int_tgi3b)?ABSOLUTE(_int_tgi3b):ABSOLUTE(_start))
        LONG(DEFINED(_int_tgi3c)?ABSOLUTE(_int_tgi3c):ABSOLUTE(_start))
        LONG(DEFINED(_int_tgi3d)?ABSOLUTE(_int_tgi3d):ABSOLUTE(_start))
        /* 116 */
        LONG(DEFINED(_int_tci3v)?ABSOLUTE(_int_tci3v):ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 120 */
        LONG(DEFINED(_int_tgi4a)?ABSOLUTE(_int_tgi4a):ABSOLUTE(_start))
        LONG(DEFINED(_int_tgi4b)?ABSOLUTE(_int_tgi4b):ABSOLUTE(_start))
        LONG(DEFINED(_int_tgi4c)?ABSOLUTE(_int_tgi4c):ABSOLUTE(_start))
        LONG(DEFINED(_int_tgi4d)?ABSOLUTE(_int_tgi4d):ABSOLUTE(_start))
        /* 124 */
        LONG(DEFINED(_int_tci4v)?ABSOLUTE(_int_tci4v):ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 128 */
        LONG(DEFINED(_int_eri0)?ABSOLUTE(_int_eri0):ABSOLUTE(_start))
        LONG(DEFINED(_int_rxi0)?ABSOLUTE(_int_rxi0):ABSOLUTE(_start))
        LONG(DEFINED(_int_txi0)?ABSOLUTE(_int_txi0):ABSOLUTE(_start))
        LONG(DEFINED(_int_tei0)?ABSOLUTE(_int_tei0):ABSOLUTE(_start))
        /* 132 */
        LONG(DEFINED(_int_eri1)?ABSOLUTE(_int_eri1):ABSOLUTE(_start))
        LONG(DEFINED(_int_rxi1)?ABSOLUTE(_int_rxi1):ABSOLUTE(_start))
        LONG(DEFINED(_int_txi1)?ABSOLUTE(_int_txi1):ABSOLUTE(_start))
        LONG(DEFINED(_int_tei1)?ABSOLUTE(_int_tei1):ABSOLUTE(_start))
        /* 136 */
        LONG(DEFINED(_int_adi0)?ABSOLUTE(_int_adi0):ABSOLUTE(_start))
        LONG(DEFINED(_int_adi1)?ABSOLUTE(_int_adi1):ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 140 */
        LONG(DEFINED(_int_swdtce)?ABSOLUTE(_int_swdtce):ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 144 */
        LONG(DEFINED(_int_cmi0)?ABSOLUTE(_int_cmi0):ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 148 */
        LONG(DEFINED(_int_cmi1)?ABSOLUTE(_int_cmi1):ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 152 */
        LONG(DEFINED(_int_iti)?ABSOLUTE(_int_iti):ABSOLUTE(_start))
        LONG(DEFINED(_int_cmi)?ABSOLUTE(_int_cmi):ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        LONG(ABSOLUTE(_start))
        /* 156 */
        LONG(DEFINED(_int_oei)?ABSOLUTE(_int_oei):ABSOLUTE(_start))
        FILL(0xff)
    } > vect                         /*ここまでをvect領域に割り付ける*/
    .dtc : {
	/* 0x400 - 0x460*/
	. += 0x003c;
        SHORT(DEFINED(_DTC1)?ABSOLUTE((_DTC1)&0xffff):ABSOLUTE(_start))
    } > dtc_vect
    .text : {
        CREATE_OBJECT_SYMBOLS
        *(.text)                     /*すべてのソースファイルの.textセクションをここに*/
        _dataROM_begin = .;          /*変数_dataROM_beginにカレントのアドレスを代入*/
    }  > rom                         /*ここまでをrom領域に割り付ける*/
    .data : AT (_dataROM_begin) {    /*AT 初期値データを_dataROM_beginに書き込む*/
        _dataRAM_begin = .;          /*変数_dataRAM_beginにカレントのアドレスを代入*/
        *(.data)                     /*すべてのソースファイルの.dataセクションをここに*/
        _dataRAM_end = .;            /*変数_dataRAM_endにカレントのアドレスを代入*/
    } > ram2                         /*ここまでをram2領域に割り付ける*/
    .bss : {
        _bss_begin = .;              /*変数_bss_beginにカレントのアドレスを代入*/
        *(.bss)                      /*すべてのソースファイルの.bssセクションをここに*/
        _bss_end = .;                /*変数_bss_endにカレントのアドレスを代入*/
    }  >ram2                         /*ここまでをram2領域に割り付ける*/
}

/*******************************************************************************
補足説明

リンク作業により次の各変数（5個）の値が定まります。これらの変数はスタートアップル
ーチンで利用されます。
_dataROM_begin    .dataセクションのメモリイメージで初期値が並ぶ先頭アドレス
                  この領域がユーザプログラム実行時に参照されることはない
_dataRAM_begin    .dataセクションの先頭アドレス
_dataRAM_end      .dataセクションの末尾アドレス＋１
_bss_begin        .bssセクションの先頭アドレス
_bss_end          .bssセクションの末尾アドレス＋１

初期化されたグローバル変数（先の例ではint aaaa=1234;）がRAM上に配置されると，電
源が切れた状態では初期値1234を覚えておくことが出来ない。またROM上に配置されると，
ユーザプログラムで値を変更することが出来ない。
そこでこのような変数は，実行プログラムではRAM領域にその変数があるように機械語コ
ードを作っておき，初期値はROM領域に書きこむようになる。同じ変数のめもり割り当て
を二重化するために「AT」コマンドが使われる。
ROM領域に書き込まれた初期値をユーザプログラムが動き出す前にRAM領域にコピーするこ
とはスタートアップルーチンの仕事の１つです。
*******************************************************************************/