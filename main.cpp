#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include "QuadratureEncoder.h"

QuadratureEncoder *QE0, *QE1;

int main(int argc, char *argv[])
{
    QE0 = new QuadratureEncoder();
    QE1 = new QuadratureEncoder();

    for(;;) {
        QE0->GetPosition();
        QE1->GetPosition();
        usleep(200000);
    }
    
    return EXIT_SUCCESS;
}
