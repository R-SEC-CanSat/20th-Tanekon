#include "Complex.h"

void setup() {
    Serial.begin(9600);

    Complex<double> num1(3.0, 4.0);
    Complex<double> num2(1.0, 2.0);

    Complex<double> sum = num1 + num2;
    Complex<double> diff = num1 - num2;
    Complex<double> prod = num1 * num2;
    Complex<double> quot = num1 / num2;

    double abs1 = Complex<double>::absolute(num1);
    double arg1 = Complex<double>::arg(num1);

    Serial.print("Sum: "); Serial.print(sum.real); Serial.print(", "); Serial.println(sum.imag);
    Serial.print("Difference: "); Serial.print(diff.real); Serial.print(", "); Serial.println(diff.imag);
    Serial.print("Product: "); Serial.print(prod.real); Serial.print(", "); Serial.println(prod.imag);
    Serial.print("Quotient: "); Serial.print(quot.real); Serial.print(", "); Serial.println(quot.imag);
    Serial.print("Absolute value of num1: "); Serial.println(abs1);
    Serial.print("Argument of num1: "); Serial.println(arg1);
}

void loop() {
    // put your main code here, to run repeatedly:
}