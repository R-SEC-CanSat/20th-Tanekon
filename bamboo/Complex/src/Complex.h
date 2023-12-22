#ifndef COMPLEX_h
#define COMPLEX_h

#include <Arduino.h>

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

#endif