/*
  山彦用共通時間ライブラリ用ヘッダ
  
  特に特別なことをするわけではなく、時間に共通性を持たせるため


  YTimeは現状ではdouble型でやってるけど、本来はlong型でやる方が最小分解能の関係上適している。
  でもなんかめんどいのでとりあえずdouble型。
  03/1/22 やっぱしlongにする。
  03/1/22 ・・・やっぱdouble。
*/

#ifndef __YMBC_TIME__
#define __YMBC_TIME__

typedef double YTime;
/*
typedef long YTime_int;

typedef struct{
  YTime time;//実時間単位
  YTime_int time_int;
  double offset; //内部時間から調整した量
}YTime_information;
*/



#endif
