#pragma once
#include <vector>

// Buffer circular simples para armazenar um histórico de valores,
// usado para plotar gráficos ao longo do tempo no ImGui.
class PlotBuffer {
public:
    explicit PlotBuffer(int size) : data(size, 0.0f), maxSize(size) {}

    void push(float value) {
        data[offset] = value;
        offset = (offset + 1) % maxSize;
        if (count < maxSize) count++;
    }

    // Ponteiro para os dados e informações necessárias pro ImGui::PlotLines,
    // que precisa saber o array, quantos pontos válidos existem, e o offset
    // "circular" para desenhar na ordem cronológica correta.
    const float* rawData() const { return data.data(); }
    int size() const { return maxSize; }
    int currentOffset() const { return offset; }

private:
    std::vector<float> data;
    int maxSize;
    int offset = 0;
    int count = 0;
};