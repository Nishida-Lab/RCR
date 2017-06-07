import processing.serial.*;
Serial myPort;  // myPortのところは好きな名前でOK
String datastr;  // datastrという名前の文字列型変数を宣言(この記事の下の方で出てきます)
PrintWriter output;  // PrintWriter型のオブジェクトを宣言
int count = 1;  // カウンタを宣言して初期化

void setup()
{
  println(Serial.list());  // 使えるポート一覧
  myPort = new Serial(this, Serial.list()[0], 115200); // myPortを最初のシリアルポート、9600bpsに設定
  myPort.clear();
  output = createWriter("log.dat");  // ファイル名log.txtでファイルを開く
}

void draw() {
  if (count>500) {  // カウンタが100超えていたら終了
    output.close();  // ファイルを閉じる
    exit();
  }
  count++; // カウントアップ

  if ( myPort.available() > 0) {
    delay(20);
    datastr = myPort.readString();
    int[] tempdata = int(split(datastr, ','));
    // String datetimestr = nf(year(),2)+"/"+nf(month(),2)+"/"+nf(day(),2)+" "+nf(hour(),2) + ":" + nf(minute(),2) + ":" + nf(second(),2);
    // String tempstr = String.format("% 4.1f,% 4.1f",tempdata[0],tempdata[1]) ;
    // output.println(tempdata[0] + ","+ tempdata[1] + "," + tempdata[2]);
    output.print(datastr);
    output.flush();  // 出力バッファに残っているデータを全て書き出し
  }
}

