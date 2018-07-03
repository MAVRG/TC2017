/* --------------------------------------------------- */
/*   SH2-PC間のシリアル通信をDTCで行うためのプログラム     */
/*                   ISSのプログラムをベースに作成        */
/* --------------------------------------------------- */
#ifndef __SERIAL__
#define __SERIAL__

//defines
#define DTC_RECEIVE_BUFFER_SIZE 255
#define SER_STX 0x09 //start
#define SER_ETX 0x0a //end


// DTCのメモリ上のアドレス指定
struct dtc1 {                             /* struct DTC1  */
    union {                               /*    DTMR      */
	unsigned short WORD;              /*  Word Access */
	struct {                          /*  Bit  Access */
	    unsigned char SM1  :1;        /*     SM1      */
	    unsigned char SM0  :1;        /*     SM0      */
	    unsigned char DM1  :1;        /*     DM1      */
	    unsigned char DM0  :1;        /*     DM0      */
	    unsigned char MD1  :1;        /*     MD1      */
	    unsigned char MD0  :1;        /*     MD0      */
	    unsigned char SZ1  :1;        /*     SZ1      */
	    unsigned char SZ0  :1;        /*     SZ0      */
	    unsigned char DTS  :1;        /*     DTS      */
	    unsigned char CHNE :1;        /*     CHNE     */
	    unsigned char DISEL:1;        /*     DISEL    */
	    unsigned char NMIM :1;        /*     NMIM     */
	    unsigned char      :4;        /*              */
	}      BIT;                       /*              */
    }           DTMR;                     /*              */
    union {                               /*    DTCRA     */
        unsigned short WORD;              /*  Word Access */
        struct {                          /*  Byte Access */
            unsigned char H;              /*    DTCRAH    */
            unsigned char L;              /*    DTCRAL    */
        }     BYTE;                       /*              */
    }          DTCRA;                     /*              */
    unsigned int    DTIAR;                /*    DTIAR     */
    unsigned int    DTSAR;                /*    DTSAR     */
    unsigned int    DTDAR;                /*    DTDAR     */
}DTC1;


/*初期化*/
void initSCI1(void);

/*送信用関数*/
int encode(unsigned char *src,int len,unsigned char *dst,int buf_max);
void odometry_send(short cnt1, short cnt2, short pwm1,short pwm2 );

/*受信用関数*/
void initDTC(void);// DTCの初期化
void startDTC(void);// 受信スタート
void stopDTC(void);// 受信ストップ
int DTC_receive(void);/*DTCからの読み込み、デコード*/

#endif
/*************************************************************/

