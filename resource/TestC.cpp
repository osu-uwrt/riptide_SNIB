#include <stdlib.h>
#include <iostream>

#include "TestC.h"

using namespace std;


int main(){
    double a = 0;
    double b = 4;
    double c = 0;
    double d = 0;

    foo(a, -b, &c, &d);

    cout << c << endl;
    cout << d << endl;

}


void foo(double a, double b, double *c, double *d){
    *c = a + b;
    *d = a - b;
}