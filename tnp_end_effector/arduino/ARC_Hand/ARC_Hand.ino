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
//  ARC_Hand Serial commmunication version
//

bool gripperOpenClose, gripperAdv, suctionAdv, itemSuctioned, drawerOpenClose, blowerOnOff, 
    suctionOnOff, blowerOnOff2 = 0, suctionNeck, itemGripped, drawerStatus, drawerStucked, 
    suctionContactDetect, gripperContactDetect, drawerShake, Trigger;
double  LPFa;
 
// IO channels
#define Gripper_baseP_Ch  DAC1 // Analog out channel to pessure regulator channel for base pressure
#define Gripper_Ch    DAC0 // Analog out channel to gripper open close fluid valve
#define Suction_Psc_Ch A5 // Analog input channel of suction pressure near suction cup
#define Suction_Pbw_Ch A6 // Analog input channel of suction pressure near blower
#define Gripper_Potentio_Ch  A2 // Analog input channel of  Potentiometer for Gripper
#define Gripper_P1_Ch  A3 // Analog input channel of Pressure sensor for Gripper
#define Gripper_P2_Ch  A4 // Analog input channel of Pressure sensor for Gripper
#define Gripper_sheetsensor1_Ch A0 // Analog input channel of sheet sensor 
#define Gripper_sheetsensor2_Ch A1 // Analog input channel of sheet sensor 
#define Suction_Root_Ch    12 // Digital out channel for Suction ON-OFF valve r(rootline)
#define Suction_Branch_Ch 13  //   Digital out channel for Suction ON-OFF valve r(branch line)
#define Blower_Ch     11 // Digital out channel for Blower power ON-OFF
#define GripperAdv_Ch  9 // Digital out channel to gripper advance valve
#define SuctionAdv_Ch  8 // Digital out channel to suction advance valve
#define SmpMonitor_Ch 32  // Digital out channel for Sampling time monitoring
#define IAI_SON_Ch 35 // IAI Servo ON
#define IAI_PC1_Ch 23 // IAI PC1
#define IAI_PC2_Ch 25 // IAI PC2
#define IAI_PC4_Ch 27 // IAI PC4
#define IAI_PC8_Ch 29 // IAI PC8
#define IAI_PEND_Ch 45 // IAI PEND 設定位置へ到着
#define IAI_PM1_Ch  53 // IAI PM1
#define IAI_PM2_Ch  51 // IAI PM2
#define IAI_PM4_Ch  49 // IAI PM4
#define IAI_PM8_Ch  47 // IAI PM8
#define IAI_STP_Ch  31 // IAI STP とりあえず false
#define IAI_LOAD_Ch  39 // IAI LOAD
#define IAI_SV_Ch  43 // IAI Servo Enable
#define IAI_CSTR_Ch 37  // IAI CSTR 
#define BendSensor_S_Ch  A8  // Bending sensor for suction contact
#define BendSensor_G_Ch  A9  // Bending sensor for gripper contact
#define LED_Ch  50  // LED for Hear Beat
#define RST_CM904_Ch 48 // OpenCM Hardware RESET 

void setup() {
 
  Serial.begin(115200);
  Serial2.begin(57600);
  Serial3.begin(57600);

  pinMode(RST_CM904_Ch, OUTPUT);
  digitalWrite(RST_CM904_Ch, HIGH );
  pinMode(Suction_Root_Ch, OUTPUT);
  pinMode(Suction_Branch_Ch, OUTPUT);
  pinMode(Blower_Ch, OUTPUT);
  pinMode(GripperAdv_Ch, OUTPUT);
  digitalWrite( GripperAdv_Ch, HIGH);
  pinMode(SuctionAdv_Ch, OUTPUT);
  digitalWrite( SuctionAdv_Ch, HIGH);
  pinMode(SmpMonitor_Ch, OUTPUT);
  //IAI初期設定
  pinMode( IAI_SON_Ch, OUTPUT);
  digitalWrite( IAI_SON_Ch, LOW);
  pinMode( IAI_STP_Ch, OUTPUT);
  digitalWrite( IAI_STP_Ch, LOW);
  pinMode( IAI_PC1_Ch, OUTPUT);
  digitalWrite( IAI_PC1_Ch, HIGH);
  pinMode( IAI_PC2_Ch, OUTPUT);
  digitalWrite( IAI_PC2_Ch, HIGH);
  pinMode( IAI_PC4_Ch, OUTPUT);
  digitalWrite( IAI_PC4_Ch, HIGH);
  pinMode( IAI_PC8_Ch, OUTPUT);
  digitalWrite( IAI_PC8_Ch, HIGH);
  pinMode( IAI_PEND_Ch, INPUT);
  pinMode( IAI_PM1_Ch, INPUT);
  pinMode( IAI_PM2_Ch, INPUT);
  pinMode( IAI_PM4_Ch, INPUT);
  pinMode( IAI_PM8_Ch, INPUT);
  pinMode( IAI_LOAD_Ch, INPUT);
  pinMode( IAI_SV_Ch, INPUT);
  pinMode( IAI_CSTR_Ch, OUTPUT);
  digitalWrite( IAI_CSTR_Ch, HIGH);
  pinMode( LED_Ch, OUTPUT);
  digitalWrite(RST_CM904_Ch, LOW);

  analogWriteResolution(12);    // DACの分解能を12bitに設定
  analogWrite(Gripper_baseP_Ch, 4095 ); // 圧力制御弁の圧力設定

  LPFa=1.0 - exp(-2.0*3.141592*20*0.001); //1.0 - exp(-2.0*M_PI*f*dt)
  
  // 初期状態の設定
  gripperOpenClose = LOW;//Hand Closde
  blowerOnOff = LOW;//ブロワーOFF
  gripperAdv = LOW;//後退
  suctionAdv = LOW;//後退
  suctionOnOff = LOW;//吸引OFF
  drawerOpenClose = HIGH; //引き出し閉
  drawerStatus = 0; // 引き出し閉
  drawerStucked = 0; // 引き出しスタック = NO
  itemSuctioned = 0; // 物品吸着＝NO
  suctionNeck = LOW;  // ストレート
  itemGripped = 0;  // 物品把持 =NO 
  drawerShake = LOW;
}

void loop() {
  unsigned long cur_time; 
  static unsigned long prv_1msec_time, prv_10msec_time, prv_100msec_time;
  static int Ps1d = 25, gripperOpenWidth = 1000, gripperGraspF = 100;

  cur_time = micros();
  if ( cur_time - prv_1msec_time >= 1000 ) { //Sampling time control 1msec のループ
    gripper_control( gripperOpenWidth, (double)gripperGraspF, cur_time ); // グリッパーの制御
    suctionContactDetect = scontact_detect( cur_time, BendSensor_S_Ch, 1.3, 1.6 );
    gripperContactDetect = gcontact_detect( cur_time, BendSensor_G_Ch, 1.3, 1.6 );
    if ( cur_time - prv_10msec_time >= 10000 ) { //Interval time control for suction check　吸着状態は10msecごとに判断（チャタリング防止の為）
      suction_control( Ps1d ); // 吸着ハンドの制御
      changer_control( );   //吸着・グリッパー切換
      prv_10msec_time = cur_time;
    }
    if ( cur_time - prv_100msec_time >= 100000 ) {
      drawer_control( );  // 引き出しの制御
      heart_beat( 5 );
      prv_100msec_time = cur_time;
    }
    
    ROS_subscribe( &Ps1d, &gripperOpenWidth, &gripperGraspF ); // ROSからのコマンドの読み込み
    ROS_publish( ); // ROSへのstatusの発信
   prv_1msec_time = cur_time;
  }
  else if ( cur_time - prv_1msec_time >= 500 ) {
    digitalWrite(SmpMonitor_Ch, 0 );    
  }
}

void  suction_control( int Ps1d ) {

  //#define SuctionThresh 0//-8もよさそう。要調整
  double  Ps1, Ps2;
  static double SuctionThresh = 10, Ps1f = 0.0;
  unsigned int dxl1_angle;
  int openCM = 0;
  if (blowerOnOff == HIGH){  
    blowerOnOff2=LOW; 
  } 
  else {  
    blowerOnOff2 = HIGH;  
  }

    digitalWrite(Blower_Ch, blowerOnOff2 );    // ブロワーのON-OFF
    digitalWrite(Suction_Root_Ch, suctionOnOff );  // 吸着のON-OFF root line
    digitalWrite(Suction_Branch_Ch, !suctionOnOff );  // 吸着のON-OFF branch line

    Ps1 = analogRead(Suction_Psc_Ch); // 真空センサ（ハンド側）の読み込み
    Ps2 = analogRead(Suction_Pbw_Ch);
    Ps1 = -(Ps1*3.3/0.6/1023-1)*25.25*1.1608;  // kPaへ変換 1.1608はキーエンスセンサへの合わせ込み係数
    Ps2 = -(Ps2*3.3/0.6/1023-1)*25.25*1.1608;

    if ( Ps1d >= 40 ) { // ブロワー保護のため吸着圧は-40kPa以上に制限
      Ps1d = 40;
    }
    double Pse = (-(double)Ps1d) - Ps1; // 真空圧誤差
    double Kps = (24-6)/40*abs(Pse)+6;  // 真空圧制御比例ゲイン　ゲインスケジューリング
    dxl1_angle = (860-600)/(68-10)*(-(double)Ps1d + 10 ) + 860 + Kps*Pse; // Waste Gate Valve の制御 積分制御は不安定
    if ( suctionOnOff == LOW ) {
      dxl1_angle = 300; // Suction Off のときは全閉
    }
    else {
      if ( dxl1_angle > 1050 ) { // 開き角の制限
        dxl1_angle = 1050; // 全開　最低吸着力
      }
      else if ( dxl1_angle < 300 ) {
        dxl1_angle = 300; // 全閉　最高吸着量
      }
    }

    openCM = Serial3.read( );
    if ( openCM == '1' ) {// openCM が受信可なら送信
      Serial3.print(String("#" + String(dxl1_angle) + "/"));    // Wasete Gate Valve のDynamixelを制御するOpenCM9.04に指令
      while(Serial3.available() ) {//バッファーをクリア
        Serial3.read();
      }
    }      

    // 吸着完了の判断
    SuctionThresh = 10.0/35.0*(-(double)Ps1d + 1.5); //　目標吸着厚 Ps1d に応じてスレッシュホールドを可変
    Ps1f += ( Ps1 - Ps1f )*LPFa;
    
    //if (SuctionThresh > -8.0){SuctionThresh = -8.0;} //　平常時の誤検知抑制
    if ( itemSuctioned ) {  // 吸着状態にある時に
      if ( Ps1f > SuctionThresh ){  //吸着がはずれたら
        itemSuctioned = LOW;    //吸着フラグを０にする   
      }
    }
    else {  //非吸着状態にある時に
        if ( Ps1f <= SuctionThresh ) {   // 吸着が完了したら
        itemSuctioned = HIGH;          //吸着フラグを1にする   
      }
    }
    //Serial.println(String(Ps1d)+" "+String(Ps1)+" "+String(Ps1f));
}

void  gripper_control( int openWidth, double Fd, unsigned long cur_time ) {

  #define Kp  0.03 // Pressure difference gain 
  #define GripperOCspeed 0.004 // Gripper Open Close speed
  #define max_qd  3.4 // Open
  #define min_qd  2.3 // Close 閉じ切らないときはここを調整
  #define max_I_Fe  1000 // 力制御積分飽和値
  #define grasp_NG_qd 2.65 // これより小さいときは把持失敗と判断 2.5
  
  double dqd, dP, vc, open_qd, sheet_sensor1, sheet_sensor2, q_raw;
  static double q = max_qd, qd = max_qd, P1 = 0, P2 = 0, Kq = 400, F, I_Fe;
  static bool gripper_stop = 0;

  open_qd = min_qd + (max_qd - min_qd) * openWidth / 112; // 設定された開き幅を開き角に変換
  if (open_qd > max_qd) {                                 // 開き角の最大最小を超えないように制限
    open_qd = max_qd;
  }
  else if (open_qd < min_qd) {
    open_qd = min_qd;
  }

  sheet_sensor1 = analogRead(Gripper_sheetsensor1_Ch);
  sheet_sensor2 = analogRead(Gripper_sheetsensor2_Ch);
  F += (sheet_sensorV2F((double)max(sheet_sensor1, sheet_sensor2)) - F) * LPFa; //シートセンサーの電圧値から力[gf]を計算 デジタルフィルLPF適用
  double Fdd = 40;
  if (F >= 0.8 * Fdd) { // 把持可否の判断 目標把持力の８割で把持完了
    itemGripped = 1;
  }
  else if (F <= 0.8 * Fdd) { //把持なしの判断はヒステリシスを持たせる <-- 戻らない原因？
    itemGripped = 0;
  }
  //Serial.println( sheet_sensor );

  q_raw = 3.3 / 1023 / 0.6 * analogRead(Gripper_Potentio_Ch);
  q += (q_raw - q) * LPFa;
  P1 += ((analogRead(Gripper_P1_Ch) * 3.3 / 0.6 / 1023 - 1) * 250 - P1) * LPFa; // デジタルLPF採用
  P2 += ((analogRead(Gripper_P2_Ch) * 3.3 / 0.6 / 1023 - 1) * 250 - P2) * LPFa;
  //Serial.println( String( q ) + " " + String(P1) + " " + String(P2) );
  dP = P1 - P2;
  if (open_qd < qd) {
    dqd = -GripperOCspeed; // 開状態時に現在の開き幅より小さな値が設定されたときの対応
  }
  else {
    dqd = GripperOCspeed; // Gripper 開方向
  }
  if (gripperOpenClose == 1) { // 閉動作＝力制御/位置制御
    I_Fe += Fd - F;
    if (I_Fe > max_I_Fe) {
      I_Fe = max_I_Fe;
    }
    else if (I_Fe < -max_I_Fe) {
      I_Fe = -max_I_Fe;
    }
    if (F > 10) {                    // 力を検出したら（物品に接触したら）力制御をおこなう
      qd = qd - 0.000002 * (Fd - F); //力制御則（比例）
    }
    else {
      qd = qd - dqd; // 物品との非接触時は位置制御にて閉じる
    }
  }
  else { // 開動作
    qd = qd + dqd;
  }
  if (qd >= max_qd) { // 目標関節角qdが最大角max_qdを超えないよう制限
    qd = max_qd;
    gripper_stop = 1;
  }
  else if (qd >= open_qd) { // 目標関節角qdが設定角open_qdを超えないよう制限
    qd = open_qd;
    gripper_stop = 1;
  }
  else if (qd <= min_qd) { // 目標関節角qdが最小角min_qdを下回らないよう制限
    qd = min_qd;
    gripper_stop = 1;
  }
  else {
    gripper_stop = 0;
  }
  if (qd <= grasp_NG_qd) { //何も掴まなかった場合、失敗
    itemGripped = 0;
    //grasp_NG = 1;
  }
  if (gripper_stop == 1) {
    gripperContactDetect = gripper_contact_detect(q, cur_time);
  }
  vc = Kp * (Kq * (qd - q) - 200 - dP) + 5;
  if (vc > 10) { //　電圧指令vcを０～１０Vに制限
    vc = 10;
  }
  else if (vc < 0) {
    vc = 0;
  }
  // Serial.println( qd );
  analogWrite(Gripper_Ch, 409.5*vc ); // グリッパーの開閉（流量制御弁への電圧指令）
}

bool gripper_contact_detect( double cur_q, unsigned long cur_time ) {//ポテンショメータによる接触検知、グリッパーが停止時のみ機能

  static bool contactDetect = 0;
  static int  contactDetect_ON = 0;
  static double rec_q;
  static unsigned long  prv_time;
  double  qe;

  if ( contactDetect_ON == 0 ) {
     contactDetect_ON = 1;
    prv_time = cur_time;
    
  }
  else if ( contactDetect_ON == 1 ) {
    if ( cur_time - prv_time  > 2000000 ) {
      contactDetect_ON = 2;
      rec_q = cur_q;
    }
  }
  else {
    if ( contactDetect == 0 ) {
      qe = abs( cur_q - rec_q );
      if ( qe > 0.02 ) {
        contactDetect = 1;
        prv_time = cur_time;
        //Serial.println("###########");
      }
    }
    else {  // contactDetect = 1
      if ( cur_time - prv_time > 3000000 ) {// 接触検知後3秒間はホールドし解除
        contactDetect = 0;
        contactDetect_ON= 0;
      }
    }
  }
  //Serial.println( String(contactDetect)+" "+String(contactDetect_ON)+" "+String(cur_q)+" "+String(rec_q)+" "+String(qe));
  return( contactDetect );
}



void  measure_F( double F ) {
  static double maxF = 0.0, minF = 3000.;
  if ( Trigger == HIGH ) {
    if ( F > maxF ) {
      maxF = F;
    }
    else if ( F < minF ) {
      minF = F;
    }
  }
  else {
    maxF = 0.0;
    minF = 3000;
  }
  Serial.println(String(minF+String(" ")+maxF+String(" ")+F));
}

double  sheet_sensorV2F(  double V ) {//Sheetセンサーの電圧値から力[gf]への変換
  double F;
  double V2 = V*V;
  double V3 = V2*V;
  double V4 = V3*V;
  double V5 = V4*V;
   F = 1.2838e-11*V5 -1.8193e-08*V4 +1.0361e-05*V3  -0.0027366*V2  +0.4965*V -1.3182;//Sheetセンサーの計測結果より5次の多項式補完式を作成
 return( F );
}

bool  scontact_detect( unsigned long cur_time, int sensor_Ch, double min_thresh, double max_thresh ) {//２つに分離しました
  double bend_sensor;
  static unsigned long prv_time;
  static bool scontactDetect = 0;

    if ( scontactDetect == 0 ) {
      bend_sensor = analogRead( sensor_Ch )*3.3/1023;
      if ( bend_sensor < min_thresh || bend_sensor > max_thresh ) {
        scontactDetect = 1;
        prv_time = cur_time;
      }
    }
    else {  // contactDetect = 1
      if ( cur_time - prv_time > 3000000 ) {// 接触検知後3秒間はホールドし解除
        scontactDetect = 0;
      }
    }
    return( scontactDetect );
    //Serial.println( String(String(contactDetect) + " "+ String(bend_sensor)) );
}

bool  gcontact_detect( unsigned long cur_time, int sensor_Ch, double min_thresh, double max_thresh ) {//２つに分離しました
  double bend_sensor;
  static unsigned long prv_time;
  static bool gcontactDetect = 0;

    if ( gcontactDetect == 0 ) {
      bend_sensor = analogRead( sensor_Ch )*3.3/1023;
      if ( bend_sensor < min_thresh || bend_sensor > max_thresh ) {
        gcontactDetect = 1;
        prv_time = cur_time;
      }
    }
    else {  // contactDetect = 1
      if ( cur_time - prv_time > 3000000 ) {// 接触検知後3秒間はホールドし解除
        gcontactDetect = 0;
      }
    }
    return( gcontactDetect );
    //Serial.println( String(String(contactDetect) + " "+ String(bend_sensor)) );
}

void  changer_control( ) {
  
    digitalWrite(SuctionAdv_Ch, suctionAdv);  // 吸着の前進後退（５ポート弁）
    digitalWrite(GripperAdv_Ch, gripperAdv);  // グリッパーの前進後退（５ポート弁）

}

void  drawer_control( ) {

  bool  PEND, PM1, PM2, PM4, PM8, LOAD, SV;
  static bool CSTR = HIGH, CSTR_ON = HIGH, prv_drawerOpenClose = HIGH, prv_drawerShake = LOW, init_drawer = 0, shake_dir = 0;
  static int  shake_counter = 0;
  
    PEND = digitalRead( IAI_PEND_Ch );
    PM1 = digitalRead( IAI_PM1_Ch );
    PM2 = digitalRead( IAI_PM2_Ch );
    PM4 = digitalRead( IAI_PM4_Ch );
    PM8 = digitalRead( IAI_PM8_Ch );
    LOAD = digitalRead( IAI_LOAD_Ch );
    SV = digitalRead( IAI_SV_Ch );

  if ( CSTR == LOW ) {
    if ( CSTR_ON == HIGH ) {
      digitalWrite( IAI_CSTR_Ch, LOW );// 動作指令
      CSTR_ON = LOW;
    }
    else if ( CSTR_ON == LOW && PEND == LOW ) {//PENDが落ちたことを確認しCSTRをHIGHに
      CSTR = HIGH;
      CSTR_ON = HIGH;
      digitalWrite( IAI_CSTR_Ch, HIGH );
    }
  }
  else {  //CSTR == HIGH　以降　動作設定
    if ( SV == LOW  ) {// IAIの電源投入時対応 電源OFF時はサーボOFF
      init_drawer = 1;  // 初期化モード発行
    }
    if ( init_drawer == 1 ) {//初期化モード
      if ( SV == HIGH ) { //SVがONになったら初期化設定
        digitalWrite( IAI_PC1_Ch, HIGH );//Close指令
        digitalWrite( IAI_PC2_Ch, LOW ); 
        digitalWrite( IAI_PC4_Ch, HIGH );
        digitalWrite( IAI_PC8_Ch, HIGH );
        CSTR = LOW;   //動作指令へ
        init_drawer = 0;
        drawerOpenClose = HIGH;
      }
    }
    if ( drawerOpenClose == LOW ) { // Openの動作設定
      if ( drawerOpenClose != prv_drawerOpenClose ) { //  Close から Open 指令に切り替わった時
        digitalWrite( IAI_PC1_Ch, LOW );//Open設定
        digitalWrite( IAI_PC2_Ch, HIGH ); 
        digitalWrite( IAI_PC4_Ch, HIGH );
        digitalWrite( IAI_PC8_Ch, HIGH );
        CSTR = LOW;
      }
      else if ( PEND == HIGH && PM1 == HIGH) {
        drawerStatus = 1; // 引き出し開完了
        drawerStucked = 0;
      }
    } 
    else {  // Closeの動作設定
      if ( drawerOpenClose != prv_drawerOpenClose ) {
        digitalWrite( IAI_PC1_Ch, HIGH );//Close設定
        digitalWrite( IAI_PC2_Ch, LOW ); 
        digitalWrite( IAI_PC4_Ch, HIGH );
        digitalWrite( IAI_PC8_Ch, HIGH );
        CSTR = LOW;
      }
      else if ( PM2 == HIGH ) {
        if ( LOAD == HIGH ) {
          drawerStucked = 1; 
        }
        drawerStatus = 0; // 引き出し閉完了
      }
    }
    if ( drawerShake == HIGH ) {
      if ( drawerShake != prv_drawerShake ) {
        if ( shake_dir == 0 ) {
          digitalWrite( IAI_PC1_Ch, LOW );//Close指令
          digitalWrite( IAI_PC2_Ch, LOW ); 
          digitalWrite( IAI_PC4_Ch, HIGH );
          digitalWrite( IAI_PC8_Ch, HIGH );
          CSTR = LOW;
        }
        else if ( shake_dir == 1 ) {
          digitalWrite( IAI_PC1_Ch, HIGH );//Close指令
          digitalWrite( IAI_PC2_Ch, HIGH ); 
          digitalWrite( IAI_PC4_Ch, LOW );
          digitalWrite( IAI_PC8_Ch, HIGH );
          CSTR = LOW;
        }
        prv_drawerShake = HIGH;
      }
      else if ( PEND == HIGH && PM1 == HIGH && PM2 == HIGH  ) {
        shake_counter++;
        shake_dir = 1;
        prv_drawerShake = LOW;
      }
      else if ( PEND == HIGH && PM4 == HIGH ) {
        shake_counter++;
        shake_dir = 0; 
        prv_drawerShake = LOW;
      }
      if ( shake_counter >= 5 ) {
        drawerShake = LOW;
        shake_counter = 0;
        shake_dir = 0;
        drawerOpenClose = LOW;
      }      
    }
  }
  prv_drawerOpenClose = drawerOpenClose;  
  //Serial.println( String(drawerOpenClose)+" "+String(init_drawer)+" "+String(SV)+" "+String(CSTR)+" "+String(PEND)+" "+String(PM1)+" "+String(PM2)+" "+String(LOAD)+" "+String(drawerStatus)+" "+String(drawerStucked));
  //Serial.println( String(drawerShake)+" "+String(shake_dir)+" "+String(shake_counter)+" "+String(CSTR)+" "+String(PEND)+" "+String(PM1)+" "+String(PM2)+" "+String(PM4)+" "+String(drawerStatus)+" "+String(drawerStucked));
}

void  heart_beat( int interval ) {
  static int count=0;

  count++;
  if ( count <= interval ) {
   digitalWrite( LED_Ch, HIGH );
  }
  else if ( count > interval && count <= 2*interval ) {
    digitalWrite( LED_Ch, LOW );
  }
  else {
    count = 0;
  }
  //Serial.println( String(interval)+" "+String(count));   
}

void ROS_subscribe( int *Ps1d, int *gripperOpenWidth, int *gripperGraspF ) { // ROSコアからのシリアル通信を受ける。シリアルモニターによる手動入力も可能（ set Num )
  
  String command; // Local変数として宣言することでcommandの文字列が毎回初期化される
                  // command のフォーマッt "set 機能番号 数値/"
  int value;

  if (Serial.available() > 0){
    while (Serial.available()) {
      char cc = Serial.read();  //gets one byte from serial buffer
      command += cc;            //makes the String readString
      delay(1);                 //slow looping to allow buffer to fill with next character
    }
     if(command.substring(0, 3) == "set"){
     int com1 = command.substring(5,6).toInt();
     int com2 = command.substring(6,7).toInt();
     int com3 = command.substring(7,8).toInt();
     int com4 = command.substring(8,9).toInt();
     int com5 = command.substring(9,10).toInt();
     int com6 = command.substring(10,11).toInt();

     //Gripper
     if (com1 == 0){
        gripperOpenClose = LOW;
        *gripperOpenWidth = command.substring(12,16).toInt();
     }
     if (com1 == 1){
        gripperOpenClose = HIGH;
        *gripperGraspF = command.substring(12,16).toInt();
     }
     //Suction
     if (com2 == 0){
        suctionOnOff = LOW;
     }
     if (com2 == 1){
        suctionOnOff = HIGH;
        *Ps1d = command.substring(17,19).toInt();
     }
     //Liner actuator gripper
     if (com3 == 0){
        gripperAdv = LOW;
     }
     if (com3 == 1){
        gripperAdv = HIGH;
     }
     //Liner actuator suction
     if (com4 == 0){
        suctionAdv = LOW;
     }    
     if (com4 == 1){
        suctionAdv = HIGH;
     }
     //Drawer
     if (com5 == 0){
        drawerOpenClose = LOW;
     }    
     if (com5 == 1){
        drawerOpenClose = HIGH;
     }
     if (com5 == 2){
        drawerShake = HIGH;
     }
     //Blower
     if (com6 == 0){
        blowerOnOff = LOW;
     }    
     if (com6 == 1){
        blowerOnOff = HIGH;
     }
    }
  }
}

void  ROS_publish( ) {
  static int prv_status;
  int status;

  //並び替えました0715
  status = drawerStucked + 2*suctionContactDetect +  4*gripperContactDetect + 8*itemSuctioned + 16*itemGripped + 32*blowerOnOff + 64*drawerStatus +128*suctionAdv + 256*gripperAdv + 512*suctionOnOff + 1024*gripperOpenClose + 2048;

  if ( prv_status != status ) {
    Serial.print("status ");
    Serial.println(status, BIN);
    prv_status = status;
  }
}

void ROS_subscribe2( int *Ps1d, int *gripperOpenWidth, int *gripperGraspF ) { // ROSコアからのシリアル通信を受ける。シリアルモニターによる手動入力も可能（ set Num )
  
  String command; // Local変数として宣言することでcommandの文字列が毎回初期化される
                  // command のフォーマッt "set 機能番号 数値/"
  int value;

  if (Serial.available() > 0){
    while (Serial.available()) {
      char cc = Serial.read();  //gets one byte from serial buffer
      command += cc;            //makes the String readString
      delay(1);                 //slow looping to allow buffer to fill with next character
    }
     if(command.substring(0, 3) == "set"){
     //int c = command.substring(4,5).toInt();
     char c = command.charAt(4);
     //Serial.println(c);
      if ( c == '0' ) { 
        gripperOpenClose = LOW;
        *gripperOpenWidth = command.substring(6).toInt();
      }
      else if ( c == '1' ) {
        gripperOpenClose = HIGH;
        *gripperGraspF = command.substring(6).toInt();
      }
      else if ( c == '2' ) {
        gripperAdv = LOW;
      } 
      else if ( c == '3' ) {
        gripperAdv = HIGH;
      } 
      else if ( c == '4' ) {
        blowerOnOff = LOW;
      } 
      else if ( c == '5' ) {
        blowerOnOff = HIGH;
      } 
      else if ( c == '6' ) {
        suctionOnOff = LOW;
      } 
      else if ( c == '7' ) {
        suctionOnOff = HIGH;
        *Ps1d = command.substring(6).toInt();
       // Serial.println( *Ps1d );
      } 
      else if ( c == '8' ) {
        suctionAdv = LOW;
      }
      else if ( c == '9' ) {
        suctionAdv = HIGH;
      } 
      else if ( c == 'a' ) {
        drawerOpenClose = LOW;
      } 
      else if ( c == 'b' ) {
        drawerOpenClose = HIGH;
      }
      else if ( c == 'c' ) {
        suctionNeck = LOW;
      }
      else if ( c == 'd' ) {
        suctionNeck = HIGH;
      }
      else if ( c == 'e' ) {
        drawerShake = HIGH;
      }
      else if ( c == 'f' ) {
        drawerShake = LOW;
      }
      else if ( c == 'g' ) {
        Trigger = HIGH;
      }
    }
  }
}



