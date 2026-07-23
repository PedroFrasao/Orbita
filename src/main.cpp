#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <chrono>
#include "Satellite.h"
#include "Sensor.h"
#include "PID.h"
#include "Actuador.h"
#include "PlotBuffer.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "Shader.h"
#include "Cube.h"

int main() {
    const double fixedDt = 1.0 / 100.0; // 100 Hz -> 0.01s por passo de simulação
    double accumulator = 0.0;

    if (!glfwInit()) {
        std::cerr << "Falha ao inicializar GLFW\n";
        return -1;
    }

    
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(800, 600, "Satellite Sim", nullptr, nullptr);
    if (!window) {
        std::cerr << "Falha ao criar janela GLFW\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // ativa VSync: trava a renderização na taxa de atualização do monitor

    
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Falha ao inicializar GLAD\n";
        return -1;
    }
    glEnable(GL_DEPTH_TEST);

    
    double previousTime = glfwGetTime();

    //  Setup do Dear ImGui 
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    std::cout << "OpenGL versao: " << glGetString(GL_VERSION) << "\n";


    Satellite satellite;
    satellite.momentOfInertia = 1.0;
    

    Sensor sensor;
    sensor.noiseStdDev = 0.02;
    sensor.bias = 0.05;

    PID pid;
    pid.Kp = 2.0;
    pid.Ki = 0.5;
    pid.Kd = 1.0;

    double setpoint = 1.0; // ângulo alvo, em radianos (~57 graus)

    Actuator actuator;
    actuator.maxTorque = 0.5;

    pid.outputMin = -actuator.maxTorque;
    pid.outputMax = actuator.maxTorque;

    const int plotHistorySize = 300; // ~3 segundos de histórico a 100Hz
    PlotBuffer angleHistory(plotHistorySize);
    PlotBuffer setpointHistory(plotHistorySize);
    PlotBuffer torqueHistory(plotHistorySize);

    float pidKp = (float)pid.Kp;
    float pidKi = (float)pid.Ki;
    float pidKd = (float)pid.Kd;
    float setpointFloat = (float)setpoint;

    float sensorNoiseStdDev = (float)sensor.noiseStdDev;
    float sensorBias = (float)sensor.bias;
    float actuatorMaxTorque = (float)actuator.maxTorque;

    Shader shader("shaders/basic.vert", "shaders/basic.frag");
    Cube satelliteCube;

    

    glm::mat4 view = glm::lookAt(
        glm::vec3(2.0f, 2.0f, 3.0f), // posição da câmera no espaço 3D
        glm::vec3(0.0f, 0.0f, 0.0f), // ponto para onde ela está olhando (origem, onde o cubo está)
        glm::vec3(0.0f, 1.0f, 0.0f)  // vetor "para cima" (define o que é "topo" da câmera)
    );

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        // Recalcula a projeção com o tamanho ATUAL da janela, evitando distorção ao redimensionar
        int framebufferWidth, framebufferHeight;
        glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
        glViewport(0, 0, framebufferWidth, framebufferHeight);

        float aspectRatio = (framebufferHeight != 0)
            ? (float)framebufferWidth / (float)framebufferHeight
            : 1.0f;

        glm::mat4 projection = glm::perspective(glm::radians(45.0f), aspectRatio, 0.1f, 100.0f);

        // Controle de tempo
        double currentTime = glfwGetTime();
        double frameTime = currentTime - previousTime;
        previousTime = currentTime;

        // Proteção: se o frame demorou demais (ex: janela foi arrastada,
        // debug travou), evita "explosão" tentando compensar tempo demais de uma vez
        if (frameTime > 0.25) {
            frameTime = 0.25;
        }

        accumulator += frameTime;

        //loop de simulação
        int stepsThisFrame = 0;
        double measuredAngle = 0.0;
        double torqueDesired = 0.0;
        double torqueApplied = 0.0;

        while (accumulator >= fixedDt) {
            measuredAngle = sensor.read(satellite.angle);

            torqueDesired = pid.update(setpoint, measuredAngle, fixedDt);
            torqueApplied = actuator.apply(torqueDesired);

            satellite.step(torqueApplied, fixedDt);

            angleHistory.push((float)satellite.angle);
            setpointHistory.push((float)setpoint);
            torqueHistory.push((float)torqueApplied);

            accumulator -= fixedDt;
            stepsThisFrame++;
        }

        // ImGui 
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(350, 400), ImGuiCond_FirstUseEver);
        ImGui::Begin("Satellite Control Panel");
        

        ImGui::Text("Ganhos do PID:");
        ImGui::SliderFloat("Kp", &pidKp, 0.0f, 10.0f);
        ImGui::SliderFloat("Ki", &pidKi, 0.0f, 5.0f);
        ImGui::SliderFloat("Kd", &pidKd, 0.0f, 5.0f);

        ImGui::Separator();
        ImGui::SliderFloat("Setpoint (rad)", &setpointFloat, -3.14f, 3.14f);
        
        ImGui::Separator();
        ImGui::Text("Sensor:");
        ImGui::SliderFloat("Ruido (desvio padrao)", &sensorNoiseStdDev, 0.0f, 0.2f);
        ImGui::SliderFloat("Bias", &sensorBias, -0.3f, 0.3f);

        ImGui::Separator();
        ImGui::Text("Atuador:");
        ImGui::SliderFloat("Torque maximo", &actuatorMaxTorque, 0.05f, 2.0f);

        

        ImGui::Separator();

        // Sincroniza os valores dos sliders (float) de volta para o sistema real (double)
        pid.Kp = pidKp;
        pid.Ki = pidKi;
        pid.Kd = pidKd;
        setpoint = setpointFloat;

        sensor.noiseStdDev = sensorNoiseStdDev;
        sensor.bias = sensorBias;

        actuator.maxTorque = actuatorMaxTorque;
        pid.outputMin = -actuatorMaxTorque;
        pid.outputMax = actuatorMaxTorque;

        ImGui::Text("Setpoint: %.4f rad", setpoint);
        ImGui::Text("Angulo real: %.4f rad", satellite.angle);
        ImGui::Text("Angulo medido (sensor): %.4f rad", measuredAngle);
        ImGui::Text("Torque desejado (PID): %.4f", torqueDesired);
        ImGui::Text("Torque aplicado (Atuador): %.4f", torqueApplied);
        ImGui::Text("Saturando? %s", (torqueDesired != torqueApplied) ? "SIM" : "nao");

        ImGui::Separator();
        ImGui::Text("Angulo (rad) ao longo do tempo:");
        ImGui::PlotLines(
            "##angle",
            angleHistory.rawData(),
            angleHistory.size(),
            angleHistory.currentOffset(),
            nullptr,
            -2.0f, 2.0f,      // escala fixa do eixo Y (min, max) -- ajuste se necessário
            ImVec2(0, 80)
        );

        ImGui::Text("Torque aplicado ao longo do tempo:");
        ImGui::PlotLines(
            "##torque",
            torqueHistory.rawData(),
            torqueHistory.size(),
            torqueHistory.currentOffset(),
            nullptr,
            -0.6f, 0.6f,      // um pouco além do maxTorque (0.5), pra enxergar saturação
            ImVec2(0, 80)
        );
        ImGui::End();

        // Renderiza a cena
        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Transformação geral do satélite 
        glm::mat4 satelliteTransform = glm::rotate(
            glm::mat4(1.0f),
            (float)satellite.angle,        // ângulo em radianos, vindo direto da simulação
            glm::vec3(0.0f, 1.0f, 0.0f)    // eixo de rotação: Y (vertical)
        );

        shader.use();
        shader.setMat4("view", glm::value_ptr(view));
        shader.setMat4("projection", glm::value_ptr(projection));

        // ---- Corpo do satélite ----
        {
            glm::mat4 local = glm::scale(glm::mat4(1.0f), glm::vec3(0.6f)); // cubo menor
            glm::mat4 model = satelliteTransform * local;

            shader.setMat4("model", glm::value_ptr(model));
            shader.setVec3("objectColor", 0.75f, 0.75f, 0.75f); // cinza metálico
            satelliteCube.draw();
        }

        // ---- Painel solar esquerdo ----
        {
            glm::mat4 local = glm::translate(glm::mat4(1.0f), glm::vec3(-1.0f, 0.0f, 0.0f));
            local = glm::scale(local, glm::vec3(1.4f, 0.5f, 0.05f)); // fino e longo
            glm::mat4 model = satelliteTransform * local;

            shader.setMat4("model", glm::value_ptr(model));
            shader.setVec3("objectColor", 0.1f, 0.2f, 0.6f); // azul escuro (painel solar)
            satelliteCube.draw();
        }

        // ---- Painel solar direito ----
        {
            glm::mat4 local = glm::translate(glm::mat4(1.0f), glm::vec3(1.0f, 0.0f, 0.0f));
            local = glm::scale(local, glm::vec3(1.4f, 0.5f, 0.05f));
            glm::mat4 model = satelliteTransform * local;

            shader.setMat4("model", glm::value_ptr(model));
            shader.setVec3("objectColor", 0.1f, 0.2f, 0.6f);
            satelliteCube.draw();
        }
        

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();
    return 0;
}