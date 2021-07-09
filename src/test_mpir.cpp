#include <iostream>
#include <mpirxx.h>

int main() {
    mpz_class a, b = 123;
    a.set_str("123456789123456789123456789", 10);
    std::cout << a + b << std::endl;
}
