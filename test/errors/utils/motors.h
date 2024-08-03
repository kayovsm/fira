#ifndef MOTORS_H
#define MOTORS_H

void setupPins();
void acelera(float vel_esquerda, float vel_direita);
void ajuste(float referencia, float valor_acelera);
void move(int use = 0);

#endif // MOTORS_H