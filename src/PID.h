#pragma once

class PID {
public:
    double Kp = 1.0;
    double Ki = 0.0;
    double Kd = 0.0;

    // Limites de saída (simula saturação do atuador -- nenhum atuador real
    // consegue aplicar torque infinito). Vamos usar isso a sério no próximo passo.
    double outputMin = -1e9;
    double outputMax = 1e9;

    double update(double setpoint, double measuredValue, double dt) {
    double error = setpoint - measuredValue;

    //  Termo Proporcional 
    double pTerm = Kp * error;

    // Calculamos a integral "tentativa" primeiro, sem ainda comprometer o estado.
    // Só decidimos aceitar essa atualização depois de saber se vamos saturar.
    double tentativeIntegral = integral + error * dt;
    double iTerm = Ki * tentativeIntegral;

    // Termo Derivativo 
    double derivative = (error - previousError) / dt;
    double dTerm = Kd * derivative;

    double output = pTerm + iTerm + dTerm;

    // Saturação 
    bool saturated = false;
    if (output > outputMax) {
        output = outputMax;
        saturated = true;
    } else if (output < outputMin) {
        output = outputMin;
        saturated = true;
    }

    // ---- Anti-windup (clamping) ----
    // Só bloqueia a integral se: está saturado E o erro está "empurrando"
    // na mesma direção da saturação (ou seja, acumular mais só pioraria).
    bool wouldWorsenSaturation =
        (output >= outputMax && error > 0) ||
        (output <= outputMin && error < 0);

    if (!(saturated && wouldWorsenSaturation)) {
        integral = tentativeIntegral; // aceita a atualização da integral
    }
    // caso contrário, a integral fica "congelada" nesse passo

    previousError = error;
    return output;
}

    void reset() {
        integral = 0.0;
        previousError = 0.0;
    }

private:
    double integral = 0.0;
    double previousError = 0.0;
};