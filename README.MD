# 環境データ収集

環境データを測定してMQTTでpublishする。
WIFI-SSID,WIFI-PASS,MQTT_HOSTのみ書き換えて使用してください。

## Device

### マイコン
[m5stack atom lite](https://docs.m5stack.com/en/core/atom_lite)
対応ドライバーは[FTDI driver](https://slx.jp/blog/2021/08/14/m5stack-atom-lite%E7%94%A8%E3%81%AE%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E6%A7%8B%E7%AF%89.html)
他のM5シリーズとは異なるため注意。

### Sensor
[M5Stack用温湿度気圧センサユニット Ver.3（ENV Ⅲ）](https://www.switch-science.com/products/7254)

## 環境
windows 11
Arduino 2.3.2
Vscode Arduino extention

### Arduino設定

- 追加のボードマネージャ
 -基本設定>追加のボードマネージャURLに書きを添付
 https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/arduino/package_m5stack_index.json

- Library
 -M5Atom
 -M5UNIT-ENV (0.0.7) ※バージョン注意, 気圧測定用にqmp6988のみ使用
 -Adafruit SHT31 (2.2.2)　※温湿度用
 -wifi(1.2.7)
 -PubSubClient(2.8)



## 参考サイト

- [arduinoの設定](https://slx.jp/blog/2021/08/14/m5stack-atom-lite%E7%94%A8%E3%81%AE%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E6%A7%8B%E7%AF%89.html)

- [Vscode arduinoの設定](https://roboin.io/article/2023/12/16/how-to-use-arduino-with-vscode/)