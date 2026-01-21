
#include <cstdio>
#include <cmath>

int main()
{
    printf("sin_table[] = {\n");

    const double pi = 3.1415926535897932384646;

    for (int i = 0; i <= 1800; i++)
    {
        printf("float_number{%.12f},", sin( static_cast<double>(i)/ 20 * pi / 180 ));
        if (!(i % 20))
            printf("\n\r\r");
    }

    printf("};\n");
}