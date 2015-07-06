#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include "quadrature-encoder.h"

QuadratureEncoder *QE;

int main(int argc, char *argv[])
{
    QE = new QuadratureEncoder();

    for(;;) {
        QE->GetPosition();
        usleep(200000);
    }
    
    return EXIT_SUCCESS;
}
