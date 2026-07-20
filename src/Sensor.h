#pragma once
#include <random>

// Simula um sensor de orientação (ex: um giroscópio/sensor de atitude simplificado).
// Nunca devolve o valor "verdadeiro" do satélite -- sempre uma versão com ruído,
// assim como um sensor real nunca é perfeito.
class Sensor {
public:
    // Desvio padrão do ruído gaussiano (em radianos).
    // Quanto maior, mais "chiado" aleatório nas leituras.
    double noiseStdDev = 0.01;

    // Bias: erro sistemático fixo da calibração do sensor (em radianos).
    // Simula um sensor real que nunca está perfeitamente calibrado.
    double bias = 0.0;

    Sensor() : gen(std::random_device{}()), noiseDist(0.0, 1.0) {}

    // Lê o ângulo "verdadeiro" e devolve uma versão ruidosa dele.
    double read(double trueAngle) {
        double noise = noiseDist(gen) * noiseStdDev;
        return trueAngle + noise + bias;
    }

private:
    std::mt19937 gen;                              // gerador de números pseudoaleatórios
    std::normal_distribution<double> noiseDist;    // distribuição gaussiana (média 0, desvio 1)
};