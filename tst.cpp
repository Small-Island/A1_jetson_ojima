#include <cstdlib>
#include <cstdio>
#include <cmath>

int main(void) {
    printf("%d\n", abs((int8_t)-90));
    printf("%lf\n", M_PI);
    double z = 1.0, x = -1.0;
    double r = sqrt(z*z + x*x);
    printf("%lf\n", acos(z/r)*(-1.0)*x/fabs(x) / M_PI * 180.0);
    return 0;
}
