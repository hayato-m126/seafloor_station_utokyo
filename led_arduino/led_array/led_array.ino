#include <Adafruit_NeoPixel.h>
#include <ros.h>
#include <auv_msgs/LedData.h>

#define PIN 6
Adafruit_NeoPixel strip = Adafruit_NeoPixel(4, PIN);

ros::NodeHandle node;

void led_handle(const auv_msgs::LedData &led_data){
  if(led_data.on_flag){
    strip.setPixelColor(0, led_data.node[0].r, led_data.node[0].g, led_data.node[0].b);
    strip.setPixelColor(1, led_data.node[1].r, led_data.node[1].g, led_data.node[1].b);
    strip.setPixelColor(2, led_data.node[2].r, led_data.node[2].g, led_data.node[2].b);
    strip.setPixelColor(3, led_data.node[3].r, led_data.node[3].g, led_data.node[3].b);
    //strip.setPixelColor(0, 0, 32, 0); //水中コネクタに接続される一番手前
    //strip.setPixelColor(1, 0, 32, 0);
    //strip.setPixelColor(2, 0, 128, 0); //フロントだけ明るくする
    //strip.setPixelColor(3, 0, 32, 0);
  }else{
    strip.setPixelColor(0, 0, 0, 0);
    strip.setPixelColor(1, 0, 0, 0);
    strip.setPixelColor(2, 0, 0, 0);
    strip.setPixelColor(3, 0, 0, 0);
  }
  strip.show();
}

ros::Subscriber<auv_msgs::LedData> sub("led", &led_handle);

void setup() {
  strip.begin();
  strip.setBrightness(255);
  node.initNode();
  node.subscribe(sub);
}

void loop() {
  node.spinOnce();
  delay(1); 
}
