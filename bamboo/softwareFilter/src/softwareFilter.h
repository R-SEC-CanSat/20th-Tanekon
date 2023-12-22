#ifndef SOFTWARESERIAL_h
#define SOFTWARESERIAL_h

#include <Arduino.h>

template<class T>
class softwareFilter{
    protected:
        T* data;
        int SIZE;
        int count;
        int elementCount;
    public:
        softwareFilter(int SIZE);
        ~softwareFilter();
        void dataAdd(T value);
        T filter();
};

template<class T>
softwareFilter<T>::softwareFilter(int SIZE) : SIZE(SIZE) {
    data = new T[SIZE];
    count = 0;
    elementCount = 0;
}

template<class T>
softwareFilter<T>::~softwareFilter() {
    delete[] data;
}

template<class T>
void softwareFilter<T>::dataAdd(T value) {
    data[count] = value;
    count++;
    if(count >= SIZE) count = 0;
    if(elementCount <= SIZE) elementCount++;
}

template<class T>
T softwareFilter<T>::filter() {
    T sum = 0;
    for(int i = 0; i < SIZE; i++) {
        sum += data[i];
    }
    return sum / elementCount;
}



#endif