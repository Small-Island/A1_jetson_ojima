#include <cstdlib>
#include <cstdio>
#include <cmath>

int main(void) {
    struct Kozo {
        int x;
    };
    struct Kozo r = {0};
    printf("%d\n", r.x);
    printf("%lf\n", M_PI);
    return 0;
}
