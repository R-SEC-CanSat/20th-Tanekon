#include <Arduino.h>

//Complex.hの中身
template<class T>
class Complex {
public:
    T real;
    T imag;

    Complex(T r = 0.0, T i = 0.0);

    Complex operator+(Complex obj);
    Complex operator-(Complex obj);
    Complex operator*(Complex obj);
    Complex operator/(Complex obj);

    static T absolute(Complex obj);
    static T arg(Complex obj);
};

template<class T>
Complex<T>::Complex(T r, T i) : real(r), imag(i) {}

template<class T>
Complex<T> Complex<T>::operator+(Complex obj) {
    return Complex(real + obj.real, imag + obj.imag);
}

template<class T>
Complex<T> Complex<T>::operator-(Complex obj){
    return Complex(real - obj.real, imag - obj.imag);
}

template<class T>
Complex<T> Complex<T>::operator*(Complex obj){
    return Complex(real * obj.real - imag * obj.imag, real * obj.imag + imag * obj.real);
}

template<class T>
Complex<T> Complex<T>::operator/(Complex obj){
    T denom = obj.real * obj.real + obj.imag * obj.imag;
    return Complex((real * obj.real + imag * obj.imag) / denom, (imag * obj.real - real * obj.imag) / denom);
}

template<class T>
T Complex<T>::absolute(Complex obj) {
    return sqrt(obj.real * obj.real + obj.imag * obj.imag);
}

template<class T>
T Complex<T>::arg(Complex obj) {
    return atan2(obj.imag, obj.real);
}
//Complex.hの中身ここまで

typedef struct{
    float x;
    float y;
} XY;

typedef struct{
    float radius;
    float theta;
} RT;

RT XY_to_RT(XY point, XY origin){
    Complex<float> complex_point(point.x,point.y);
    Complex<float> complex_origin(origin.x,origin.y);
    RT result;
    result.radius = Complex<float>::absolute(complex_point);
    result.theta = Complex<float>::arg(complex_point / complex_origin);
    return result;
}

void setup() {
    Serial.begin(9600);

    // 座標を設定
    XY point = {3.0, 4.0};
    XY origin = {0.0, 1.0};

    // 座標変換を行う
    RT result = XY_to_RT(point, origin);

    // 結果を出力
    Serial.print("Radius: ");
    Serial.println(result.radius);
    Serial.print("Theta: ");
    Serial.println(result.theta);
}

void loop() {
    // 何もしない
}



