

#include "path_planner.h"








double path_planner::taylor(VectorXd& ders, double time){
    double result = 0;
    for(int i = 0; i < ders.size(); i++){
        result += ders(i)*pow(time, i)/factorial(i);
    }
    return result;
}

int path_planner::factorial(int n) {
    return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}







