#pragma once
#include <glad/glad.h>

// Representa a geometria de um cubo simples (1x1x1, centrado na origem).
// Encapsula o VAO/VBO/EBO -- os detalhes de "como os dados ficam na GPU".
class Cube {
public:
    Cube() {
        setup();
    }

    void draw() {
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }

private:
    unsigned int VAO, VBO, EBO;

    void setup() {
        // 8 vértices únicos do cubo (X, Y, Z)
        float vertices[] = {
            -0.5f, -0.5f,  0.5f, // 0
             0.5f, -0.5f,  0.5f, // 1
             0.5f,  0.5f,  0.5f, // 2
            -0.5f,  0.5f,  0.5f, // 3
            -0.5f, -0.5f, -0.5f, // 4
             0.5f, -0.5f, -0.5f, // 5
             0.5f,  0.5f, -0.5f, // 6
            -0.5f,  0.5f, -0.5f, // 7
        };

        // Cada face = 2 triângulos = 6 índices. 6 faces x 6 = 36 índices.
        unsigned int indices[] = {
            0, 1, 2,  2, 3, 0,  // frente
            1, 5, 6,  6, 2, 1,  // direita
            5, 4, 7,  7, 6, 5,  // trás
            4, 0, 3,  3, 7, 4,  // esquerda
            3, 2, 6,  6, 7, 3,  // topo
            4, 5, 1,  1, 0, 4   // base
        };

        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

        // Descreve o layout: atributo 0 = posição (3 floats, sem espaçamento extra)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glBindVertexArray(0); // "desconecta" o VAO, evitando alterações acidentais depois
    }
};