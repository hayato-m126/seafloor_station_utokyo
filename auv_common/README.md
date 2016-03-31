# auv_common
AUVが共用で使うメッセージと関数をまとめたもの  

## Description

海底ステーションで使うセンサのデータ形式を定義している。  
海底ステーション以外のAUVでも、これを読みこませることでデータ形式を統一する  
共通で使う関数もここで定義しておき、使い回す  

## Requirement

- ROS Indigo

## Usage
通信メッセージの型はmsgにファイルを追加する  
CMakeLists.txtのadd_message_filesにファイル名を追加してcatkin_makeする  

関数はsrc/auvfunc.pyに書く  

## メッセージ内容

AlocData: Alocから上がってくるデータ形式  
AlocRequest: Alocにデータ送信依頼するデータ形式
CurrentData: 電磁流速計から上がってくるデータ形式  
DepthData: 圧力センサから上がってくるデータ形式  
GyroData: 方位計から上がってくるデータ形式  
LedColor: LEDの色、RGB  
LedData: LEDアレイをコントロールするデータ形式  
ModemData: ハイレベルからModemに送信依頼するデータ形式  
MpuData: MPU9250で拾える方位情報と温度、圧力のデータ形式  
SerialDebug: シリアル通信の結果をログしておくための形式  
StationData: stationがAlocと通信したデータやその内容のデータ形式  
WaterData: 水の状態,ALOCはこのデータから音速を拾う  

## Author

[Hayato Mizushima](https://twitter.com/hayato_m126)  
Takumi Matsuda  
Toshihiro Maki  

## License

MIT
