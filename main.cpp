#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "quadrature-encoder.h"

QuadratureEncoder *QE;

int main(int argc, char *argv[])
{
    QE = new QuadratureEncoder();
    
    return EXIT_SUCCESS;
}
