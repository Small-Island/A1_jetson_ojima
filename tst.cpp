#include <cstdlib>
#include <cstdio>

int main(void) {
    struct Kozo {
        int x;
    };
    struct Kozo r = {0};
    printf("%d\n", r.x);
    return 0;
}
