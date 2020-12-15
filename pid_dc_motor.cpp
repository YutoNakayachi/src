#include "mbed.h" 
#include <math.h>

#define PI 3.14159265358979323846

Ticker ticker; 
Timer timer; 


DigitalOut myled(LED1); 
PwmOut in1(p23);
PwmOut in2(p24);
AnalogIn ain(p20);

float pwm_freq = 10000.0f; // PWM周期 : 10k[HZ]
float p_target; // 目標量：範囲：0~100[mm]
float delta_time; // 微分用.
float guard_value; // ITermが大きく(小さく)なりすぎたとき用

float Kvp = 100.f; // voltage[V] to positional [mm] gain 
float Kp; // proportinal gain
float Ki; // intagral gain
float Kd; // differential gain

float last_ep; // 1つ前のシーケンスの偏差
float PTerm; // P制御量
float ITerm; // I制御量
float DTerm; // D制御量
float output; // PID制御量

// テスト用
float ts;
float mx=0.0f;
float mn=-0.0f;
int count_ = 0; 

void clear(){ 
    PTerm = 0.0f;
    ITerm = 0.0f;
    DTerm = 0.0f;
    last_ep = 0.0f; 
    delta_time = 0.001f;
    guard_value = 20.0f;
    output = 0.0f;
}

// テスト用
// 雑位置初期化
void move_init(){
    float av; 
    float p_now; 
    float p_t; 
    float error; 

    do{
    av = ain.read(); 
    p_now = Kvp * av; 
    p_t = 50.0f; 
    error = p_t - p_now; 
    }
    while(error>=1.0f);

    for(int i=0;i<10;i++){
        myled = 1;
        wait(0.25); 
        myled = 0;
        wait(0.25); 
    }

}

void init(float kp,float ki,float kd){
    Kp = kp; // proportinal gain 
    Ki = ki;  // intagral gain 
    Kd = kd;  // differential gain
    clear(); 
    /**
    Kp = 0.3f; // proportinal gain 
    Ki = 0.3f;  // intagral gain 
    Kd = 0.002f;  // differential gain
    **/
}

void move(){
    // テスト用パラメータ
    float t;   // current time[ms] 
    float tn; 
    float F;   // sin wave frequency [Hz] 
    float T;   // sin wave cycle [s] 
    float A;   // sin wave amplitude 
    float B; 
    //

    float av;  // analog voltage[V]
    float p_now; // current position[mm]
    float ep; // 偏差 = 目標量 - 制御量
    float delta_ep; // delta_ep = 現在の偏差 - 1つ前の偏差
    float feedback_value; // 制御量の和
    float s; // outputを正規化した値

    // テストとして, sin波を目標量とする.
    tn = timer.read_ms();
    t = tn - ts;   // current time[ms]
    F = 0.05f;               // frequency [Hz] ... sin
    A = 30.0f;               // amplitude ... sin 
    B = 50.0f;
    T = (1.0f/F) * 1000.0f; // 周期 [ms]
    
    if((t/T)>=1.0f){
        ts += T;
        t = t - T;
    }
    // 50mm地点を中心に振幅30mmのsin運動をさせる.
    p_target = A * sin(2.0f*PI*F*t/1000.0f) + B; // 範囲：0~100[mm]

    av = ain.read(); // スライダーの現在位置の電圧　範囲：0.0 ~ 1.0 
    p_now = Kvp * av; // current position　範囲：0.0~100.0[mm]
    ep = p_target - p_now; // positional error　範囲：-200.0~+200.0[mm]
    PTerm = ep; // P制御量　範囲：-60.0 ~ +60.0 

    delta_ep = ep - last_ep;  // 範囲：?
    ITerm += ep * delta_time; // I制御量　範囲：-20.0 ~ +20.0 

    if(ITerm > guard_value){
        ITerm = guard_value;
    }
    if(ITerm < -guard_value){
        ITerm = - guard_value;
    }

    DTerm = delta_ep / delta_time; // D制御量　範囲：
    last_ep = ep; 

    // 制御量outputの量を-0.5~0.5に正規化させる.
    // これが分からない.

    Kd = 0.0f;
    output =(Kp * ep) + (Ki * ITerm) + (Kd * DTerm); // 制御量 範囲：-0.5 ~ +0.5
    
    /**
    if(output>mx){
        mx = output; 
    }
    if(output<mn){
        mn = output; 
    }
    **/
    //s = (output - mn)/(mx-mn) - 0.5f; //s:-0.5 ~ +0.5
    s = output;
    if(s<-0.5f){
        s = -0.5f; 
    }
    if(0.5f<s){
        s = 0.5f; 
    }

    in1.write(1.0);
    // 0 : left | 0.5 : stop | 1.0 : right 
    in2.write(s + 0.5f); 

    count_ += 1;
    if(count_%1000==0){
    
    
    // <-----------デバッグ----------------->
    
    /**
    // Tを出力
    printf("T"); 
    printf("\r\n"); 
    printf("%f",T);
    printf("\r\n");
    // tを出力
    printf("t"); 
    printf("\r\n"); 
    printf("%f",t);
    printf("\r\n"); 
    // tsを出力
    printf("ts"); 
    printf("\r\n"); 
    printf("%f",ts);
    printf("\r\n"); 
    // tnを出力
    printf("tn"); 
    printf("\r\n"); 
    printf("%f",tn);
    printf("\r\n"); 
    
    **/
    /**
    // p_nowを出力
    printf("p_now"); 
    printf("\r\n"); 
    printf("%f",p_now);
    printf("\r\n"); 
    // p_targetを出力
    printf("p_target"); 
    printf("\r\n"); 
    printf("%f",p_target);
    printf("\r\n"); 
    
    // アナログ電圧を出力
    printf("av"); 
    printf("\r\n"); 
    printf("%f",av);
    printf("\r\n"); 
    // 制御量を出力
    printf("output"); 
    printf("\r\n"); 
    printf("%f",output);
    printf("\r\n"); 
    // 偏差を出力.
    printf("ep"); 
    printf("\r\n"); 
    printf("%f",ep);
    printf("\r\n");
    
    printf("\r\n");
    **/ 
    }

}

int main(){

    in1.period(1.0f/pwm_freq);
    in2.period(1.0f/pwm_freq); 

    //move_init();
    
    // 5秒間まつ. 
    wait(5.0); 

    // PIDパラメータの設定.
    init(0.6f,0.6f,0.3f);

    timer.start();
    ts = timer.read_ms();
    ticker.attach_us(&move,1000.0f); // controll period should be 1ms.

    return 0;
}