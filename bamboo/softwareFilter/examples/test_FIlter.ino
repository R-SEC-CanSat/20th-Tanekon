#include "softwareFilter.h"

void setup() {
    Serial.begin(9600);

    softwareFilter<int> filter(10);

    for(int i = 0; i < 10; i++) {
        filter.dataAdd(i);
    }

    int average = filter.filter();

    Serial.print("Average: "); Serial.println(average);
}

void loop() {
    // put your main code here, to run repeatedly:
}