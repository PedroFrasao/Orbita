#pragma once

// Representa o estado físico do satélite, restrito a rotação em 1 eixo.
// Penso nisso como um "pião" que pode girar livremente, sofrendo
// torque de atuadores e (por enquanto) nenhum atrito/perturbação externa.
class Satellite {
public:
    // Momento de inércia: resistência do satélite a mudar sua rotação.
    // Valores maiores = satélite "mais pesado" de girar.
    double momentOfInertia = 1.0;

    // Estado atual
    double angle = 0.0;           // orientação atual, em radianos
    double angularVelocity = 0.0; // velocidade angular atual, em rad/s

    // Avança a simulação física em um passo fixo de tempo (dt),
    // aplicando um torque externo (que virá do atuador, futuramente).
    void step(double torque, double dt) {
        double angularAcceleration = torque / momentOfInertia;

        // Integração de Euler semi-implícito:
        // atualiza velocidade PRIMEIRO, depois usa a velocidade já atualizada
        // pra atualizar a posição. Isso é mais estável numericamente do que
        // Euler explícito puro (que usaria a velocidade "antiga").
        angularVelocity += angularAcceleration * dt;
        angle += angularVelocity * dt;
    }
};