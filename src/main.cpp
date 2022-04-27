#include <iostream>
#include <string>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <physics.h>
#include "utils/imgui.h"


void draw(const std::vector<Rect*> rectangles, utils::ImguiCanvas& canvas) {
    for (const auto r : rectangles) {
        canvas.drawRectangle(r->getPoints());
        const auto p = r->getPoints()[0];
        const auto v = r->getParticuleVelocity(p);
        //canvas.drawLine({p, v+p});
    }
}
void draw(utils::ImguiCanvas& canvas, const Rect::IntersectionResult& r) {
	for (const auto& p : r.contactPoints) {
		canvas.drawPoint(p);
	}
	//if (r.contactPoints.size() > 0) {
 //       canvas.drawLine({ r.contactPoints[0], r.contactPoints[0] + 50 * r.normal });
	//}
	//std::cout << r.contactPoints.size() << " " << r.depth << std::endl;

};

static void intializeSimulation(std::vector<Rect*>& rectangles) {
    rectangles.push_back(new Rect{ 100, 500 , 100, 100 });
    rectangles.push_back(new Rect{ 300, 100, 500, 100 });
    rectangles[0]->rotate(pi/6);
    //rectangles[0]->setAngularVelocity(pi);
    rectangles[0]->setLinearVelocity({ 0, -150 });
    rectangles[1]->mass = 10;

}

static void simulationLoop(std::vector<Rect*>& rectangles, utils::ImguiCanvas& canvas) {
    const float deltaTime = ImGui::GetIO().DeltaTime;
    //rectangles[0]->setPosition(canvas.mousePos);
    CollisionResolver collisionResolver;
    for (unsigned i = 0; i < rectangles.size(); ++i) {
        for (unsigned j = i+1; j < rectangles.size(); ++j) {
            const auto result = rectangles[j]->intersects(*rectangles[i]);
            //draw(canvas, result);
            //apply da physics mr white
            if (result.intersects) {
                collisionResolver.resolveInterPenetration(*rectangles[j], *rectangles[i], result);
                collisionResolver.resolveMovement(*rectangles[j], *rectangles[i], result);
                //collisionResolver.resolveInterPenetration(*rectangles[j], *rectangles[i], result);
            }

        }
    }
    rectangles[0]->addLinearVelocity({ 0, -9 });

    update(rectangles, deltaTime);
    draw(rectangles, canvas);
}

int main()
{
    constexpr unsigned int width = 1280;
    constexpr unsigned int height = 780;
    GLFWwindow* window;

    if (!glfwInit()) {
        return -1;
    }

    window = glfwCreateWindow(width, height, "Hello World", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    utils::ImGuiWrapper imguiWrapper(window, "#version 130");
    glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL 
    glfwSwapInterval(1);

    gladLoadGL();
    glViewport(0, 0, width, height);
    auto& canvas = imguiWrapper.createCanvas(600, 600);
    std::vector<Rect*> rectangles;
    intializeSimulation(rectangles);
    while (!glfwWindowShouldClose(window)) {
        Rect rect{ 100, 100, 500, 100 };
        auto [p0, p1, p2, p3] = rect.getPoints();
        const auto callback = [&rectangles](auto& canvas) { simulationLoop(rectangles, canvas); };
        canvas.drawCallback = callback;
        imguiWrapper.render();
        glfwSwapBuffers(window);
        glfwPollEvents();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    glfwTerminate();

	return 0;
}


