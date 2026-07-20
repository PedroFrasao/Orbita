#pragma once

// Representa um atuador físico (ex: reaction wheel) que aplica torque
// ao satélite. Nenhum atuador real tem força infinita -- este simula
// essa limitação através da saturação (clamping) do torque pedido.
class Actuator {
public:
    // Torque máximo que o atuador consegue fisicamente produzir (em qualquer direção).
    double maxTorque = 0.5;

    // Recebe o torque "desejado" (vindo do PID) e devolve o torque
    // que o atuador REALMENTE consegue entregar.
    double apply(double desiredTorque) {
        double actualTorque = desiredTorque;

        if (actualTorque > maxTorque) {
            actualTorque = maxTorque;
        } else if (actualTorque < -maxTorque) {
            actualTorque = -maxTorque;
        }

        return actualTorque;
    }
};