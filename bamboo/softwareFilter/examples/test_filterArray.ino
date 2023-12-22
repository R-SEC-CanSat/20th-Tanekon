#include "softwareFilter.h"

void setup() {
    Serial.begin(9600);

    softwareFilter<int> filters[10]; // 10 elements

    for(int i = 0; i < 10; i++) {
        filters[i] = softwareFilter<int>(10);
        for(int j = 0; j < 10; j++) {
            filters[i].dataAdd(j);
        }
        int average = filters[i].filter();
        Serial.print("Average of filter "); Serial.print(i); Serial.print(": "); Serial.println(average);
    }
}

void loop() {
    // put your main code here, to run repeatedly:
}