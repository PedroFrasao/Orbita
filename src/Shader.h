#pragma once
#include <glad/glad.h>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

class Shader {
public:
    unsigned int ID;

    Shader(const char* vertexPath, const char* fragmentPath) {
        std::string vertexCode = readFile(vertexPath);
        std::string fragmentCode = readFile(fragmentPath);

        unsigned int vertex = compile(vertexCode, GL_VERTEX_SHADER, "VERTEX");
        unsigned int fragment = compile(fragmentCode, GL_FRAGMENT_SHADER, "FRAGMENT");

        ID = glCreateProgram();
        glAttachShader(ID, vertex);
        glAttachShader(ID, fragment);
        glLinkProgram(ID);
        checkLinkErrors(ID);

        // Depois de linkados no programa, os shaders individuais não são mais necessários
        glDeleteShader(vertex);
        glDeleteShader(fragment);
    }

    void use() {
        glUseProgram(ID);
    }

    void setMat4(const std::string& name, const float* matrixPtr) {
        int location = glGetUniformLocation(ID, name.c_str());
        glUniformMatrix4fv(location, 1, GL_FALSE, matrixPtr);
    }

    void setVec3(const std::string& name, float x, float y, float z) {
        int location = glGetUniformLocation(ID, name.c_str());
        glUniform3f(location, x, y, z);
    }

private:
    std::string readFile(const char* path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            std::cerr << "Falha ao abrir arquivo de shader: " << path << "\n";
            return "";
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }

    unsigned int compile(const std::string& source, unsigned int type, const char* label) {
        unsigned int shader = glCreateShader(type);
        const char* src = source.c_str();
        glShaderSource(shader, 1, &src, nullptr);
        glCompileShader(shader);

        int success;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            char infoLog[512];
            glGetShaderInfoLog(shader, 512, nullptr, infoLog);
            std::cerr << "Erro ao compilar shader (" << label << "):\n" << infoLog << "\n";
        }
        return shader;
    }

    void checkLinkErrors(unsigned int program) {
        int success;
        glGetProgramiv(program, GL_LINK_STATUS, &success);
        if (!success) {
            char infoLog[512];
            glGetProgramInfoLog(program, 512, nullptr, infoLog);
            std::cerr << "Erro ao linkar shader program:\n" << infoLog << "\n";
        }
    }
};