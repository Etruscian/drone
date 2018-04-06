#include <math.h>
#include <iostream>

class Kalman{
    public:
        void init(void){
            Q = 0.1;
            R = 0.1;
            xEstimatedPrevious = 0;
            xEstimatedDerived = 0;
            pPrevious = 0;            
        };
        void calculate(float * value){
            xTempEstimated = xEstimatedPrevious + xEstimatedDerived;
            pTemp= pPrevious + Q;
            K = pTemp /(pTemp + R);
            xEstimated = xTempEstimated + K*(*value - xTempEstimated);
            *value = xEstimated;
            xEstimatedDerived = xEstimatedPrevious - xEstimated;
            xEstimatedPrevious = xEstimated;
            P = (1-K) * pTemp;
            pPrevious = P;
        };
    private:
        float xEstimatedPrevious, pPrevious, Q, R, K, P, pTemp, xTempEstimated, xEstimated, xEstimatedDerived;
};