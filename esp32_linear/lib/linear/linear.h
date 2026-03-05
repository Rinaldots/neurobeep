#ifndef LINEAR_H
#define LINEAR_H

#include <Arduino.h>

enum Estado {
        PARADO,
        HOMING,
        INDO,
        VOLTANDO
    };
enum Comando {
        PLAY,
        STOP
};

class LinearCar {
public:
    LinearCar();
    void setup();
    void step_loop();
    void setTargetStep(long steps);
    Comando comando = STOP;
    Estado estado = PARADO;
    Estado ultimoEstado = PARADO;
    long steps = 0;

private:
    long stepDelta = 0;
    long stepTarget = 0;
    bool endTrigged = false;
    const int limitePassos = 1750;

};

extern LinearCar linearCar;

#endif