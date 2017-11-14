/*
 * Version:  2017.07.31
 * Authors:  Members of the Team NAIST-Panasonic at the Amazon Robotics Challenge 2017:
 *           Gustavo A. Garcia R. <garcia-g at is.naist.jp> (Captain), 
 *           Lotfi El Hafi, Felix von Drigalski, Wataru Yamazaki, Viktor Hoerig, Arnaud Delmotte, 
 *           Akishige Yuguchi, Marcus Gall, Chika Shiogama, Kenta Toyoshima, Pedro Uriguen, 
 *           Rodrigo Elizalde, Masaki Yamamoto, Yasunao Okazaki, Kazuo Inoue, Katsuhiko Asai, 
 *           Ryutaro Futakuchi, Seigo Okada, Yusuke Kato, and Pin-Chu Yang
 *********************
 * Copyright 2017 Team NAIST-Panasonic 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at 
 *     http://www.apache.org/licenses/LICENSE-2.0 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************
 */

//
// Dynamixel controller by serial communication
//

/* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
/* Dynamixel ID defines */
#define ID_NUM_1 1
#define ID_NUM_2 2
/* Control table defines */
#define GOAL_POSITION 30
#define GOAL_SPEED 32

unsigned long cur_time, prv_time;

Dynamixel Dxl(DXL_BUS_SERIAL3);

bool WasteGate = 0;

void setup() {
  
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  Dxl.writeWord(ID_NUM_1, GOAL_SPEED, 1024);  //Dynamixel ID 1 Speed Control (Address_data : 0~1024)
  Dxl.jointMode(ID_NUM_1); //jointMode() is to use position mode
  Dxl.writeWord(ID_NUM_2, GOAL_SPEED, 256);  //Dynamixel ID 1 Speed Control (Address_data : 0~1024)
  Dxl.jointMode(ID_NUM_2); //jointMode() is to use position mode
    
  Serial2.begin(57600);

  SerialUSB.begin();
}

void loop() {
  
  char  c[12];
  int   cmd1_len, cmd2_len;
  int   dxl_angle;
  static int i=0, receiveCmd = 0;

       Serial2.print( 1 );  // 受信可、送信要求
       while (Serial2.available()) {
        c[i] = Serial2.read();  //gets one byte from serial buffer
        delay(4);  // 読み込み待ち
        if ( receiveCmd == 0 ) {//コマンド待ち状態なら
          if ( c[i] == '#' ) {//先頭文字を検出したら
            Serial2.print( 0 );  // 受信不可、送信停止要求(受信バッファーが溢れないよう停止要求）
            receiveCmd = 1;  //コマンド受付けフラグを立て
            i++;              //次の文字の読み込みへ
            continue;
          }
          else {
            while(Serial2.available() ) {//先頭文字を検出できなかったらバッファーをクリアし次のコマンド待ち（通信が来るまで待つ）
              Serial2.read();
            }
            SerialUSB.print("error1");
            break;
          }
        }
        else if ( receiveCmd == 1 ) { // 前半コマンド受付状態なら
          if ( c[i]== '$' ){//前半終端文字なら1番目のDyanamixelの角度を計算
           receiveCmd = 2;  // 後半コマンドの読み込みへ
           cmd1_len = i - 1;
           i++;
           continue;
           }
           else if ( i==5 ) {//前半終端文字を受け取れず文字超過の場合、バッファークリアして次の通信待ちへ
             while(Serial2.available() ) {
               Serial2.read();
             }
             receiveCmd = 0;
             i = 0;
             SerialUSB.print("error2");
             break;
           }
          else {//前半終端文字でなければ次の文字（次の桁）の読み込みへ （通信正常）
            i++;
            continue;
          }
        }
        else { // recieveCmd == 2 後半コマンド受付状態なら
           if ( c[i]== '/' ){//終端文字ならバッファーをクリアーしDyanamixelの角度を計算
             while(Serial2.available() ) {//バッファーのクリアー
               Serial2.read();
             }
            break;
          }
          else if ( i - cmd1_len ==6 ) {//終端文字を受け取れず文字超過の場合、バッファークリアして次の通信待ちへ
            while(Serial2.available() ) {
              Serial2.read();
            }
            receiveCmd = 0;
            i = 0;
            SerialUSB.print("error3");
            break;
          }
          else {//終端文字でなければ次の文字（次の桁）の読み込みへ （通信正常）
            i++;
            continue;
          }
        }
      }
    if ( receiveCmd == 2 ) {//通信がうまく行ったとき(#-$-/の順で読み込めたとき）DXLへ指令
      if ( cmd1_len == 4 ) {
        dxl_angle = (c[1]-'0')*1000+(c[2]-'0')*100+(c[3]-'0')*10+c[4]-'0';
      }
      else if ( cmd1_len == 3 ) {
        dxl_angle = (c[1]-'0')*100+(c[2]-'0')*10+c[3]-'0';
      }
      else if ( cmd1_len == 2 ) {
        dxl_angle = (c[1]-'0')*10+c[2]-'0';
      }
      else {
        dxl_angle = c[1]-'0';
      }
      SerialUSB.println( dxl_angle );
      Dxl.writeWord(ID_NUM_1, GOAL_POSITION, dxl_angle); //Dynamixel への動作指令
 
      cmd2_len = i - cmd1_len - 2;
      if ( cmd2_len == 4 ) {
        dxl_angle = (c[cmd1_len+2]-'0')*1000+(c[cmd1_len+3]-'0')*100+(c[cmd1_len+4]-'0')*10+c[cmd1_len+5]-'0';
      }
      else if ( cmd2_len == 3 ) {
        dxl_angle = (c[cmd1_len+2]-'0')*100+(c[cmd1_len+3]-'0')*10+c[cmd1_len+4]-'0';
      }
      else if ( cmd2_len == 2 ) {
        dxl_angle = (c[cmd1_len+2]-'0')*10+c[cmd1_len+3]-'0';
      }
      else {
        dxl_angle = c[cmd1_len+2]-'0';
      }
        SerialUSB.println( dxl_angle );
        Dxl.writeWord(ID_NUM_2, GOAL_POSITION, dxl_angle); //Dynamixel への動作指令
      }
    receiveCmd = 0;//次のコマンド待ちへ
    i = 0;
}


