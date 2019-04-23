#ifndef MJ_RENDER_H
#define MJ_RENDER_H

#include <iostream>

#include "mujoco.h"
#include "glfw3.h"

class MjRender {
public:
    /// Constructor
    MjRender(mjModel* model, mjData* data);
    /// Destructor
    ~MjRender();
    /// Rendering function
    void render();
    /// Rendering variables
    static GLFWwindow* window;              // rendering window
    static mjrRect viewport;                // viewport
    static mjvCamera cam;                   // abstract camera
    static mjvOption opt;                   // visualization options
    static mjvScene scn;                    // abstract scene
    static mjrContext con;                  // custom GPU context
    /// Key flags
    static bool paused, showFullScreen;
private:
    /// Callback functions
    static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
    static void mouseButton(GLFWwindow* window, int button, int act, int mods);
    static void mouseMove(GLFWwindow* window, double xpos, double ypos);
    static void scroll(GLFWwindow* window, double xoffset, double yoffset);
    /// MuJoCo variables
    static mjModel* m;                      // MuJoCo model
    static mjData* d;                       // MuJoCo data
    /// Mouse interaction variables
    static bool buttonLeft, buttonMiddle, buttonRight;
    static double lastX, lastY;
    /// Simulation status
    char status[1000] = "";
};



#endif //MJ_RENDER_H
