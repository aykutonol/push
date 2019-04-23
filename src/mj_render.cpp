#include "mj_render.h"

GLFWwindow* MjRender::window;
mjrRect MjRender::viewport;
mjvCamera MjRender::cam;
mjvOption MjRender::opt;
mjvScene MjRender::scn;
mjrContext MjRender::con;
mjModel* MjRender::m;
mjData* MjRender::d;
bool MjRender::buttonLeft, MjRender::buttonMiddle, MjRender::buttonRight;
double MjRender::lastX, MjRender::lastY;
bool MjRender::paused=false, MjRender::showFullScreen=false;

MjRender::MjRender(mjModel* model, mjData* data)
{
    // initialize variables
    m = model;
    d = data;
    buttonLeft   = false;
    buttonMiddle = false;
    buttonRight  = false;
    lastX = 0.0;
    lastY = 0.0;
    // initialize GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");
    // create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1280, 800, "Simulation Rendering", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);
    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouseMove);
    glfwSetMouseButtonCallback(window, mouseButton);
    glfwSetScrollCallback(window, scroll);
    // get framebuffer viewport
    viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
}

MjRender::~MjRender() {
    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    // free MuJoCo model and data
    mj_deleteData(d);
    mj_deleteModel(m);
}


void MjRender::render()
{
    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);
    // status: time & contacts
    sprintf(status, "%-5.4f\n%d", d->time, d->ncon);
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport,
                "Time\nContacts", status, &con);
    // running status
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMRIGHT, viewport, paused ? "PAUSED" : "RUNNING", NULL, &con);
    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);
    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

// keyboard callback
void MjRender::keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // do not act on release
    if( act==GLFW_RELEASE )
        return;
    // key configurations
    switch( key )
    {
        // backspace: reset simulation
        case GLFW_KEY_BACKSPACE:
            mj_resetData(m, d);
            mj_forward(m, d);
        // space: pause
        case GLFW_KEY_SPACE:
            paused = !paused;
            break;
        // F5: toggle full screen
        case GLFW_KEY_F5:
            showFullScreen = !showFullScreen;
            if( showFullScreen )
                glfwMaximizeWindow(window);
            else
                glfwRestoreWindow(window);
            break;
        // cycle over frame rendering modes
        case ';':
            opt.frame = mjMAX(0, opt.frame-1);
            break;
         // cycle over frame rendering modes
        case '\'':
            opt.frame = mjMIN(mjNFRAME-1, opt.frame+1);
            break;
        // cycle over label rendering modes
        case '.':
            opt.label = mjMAX(0, opt.label-1);
            break;
        // cycle over label rendering modes
        case '/':
            opt.label = mjMIN(mjNLABEL-1, opt.label+1);
            break;
        // general toggle flag
        default:
            // toggle visualization flag
            for( int i=0; i<mjNVISFLAG; i++ )
                if( key==mjVISSTRING[i][2][0] )
                    opt.flags[i] = !opt.flags[i];
            // toggle rendering flag
            for( int i=0; i<mjNRNDFLAG; i++ )
                if( key==mjRNDSTRING[i][2][0] )
                    scn.flags[i] = !scn.flags[i];
            // toggle geom/site group
            for( int i=0; i<mjNGROUP; i++ )
                if( key==i+'0')
                {
                    if( mods & GLFW_MOD_SHIFT )
                        opt.sitegroup[i] = !opt.sitegroup[i];
                    else
                        opt.geomgroup[i] = !opt.geomgroup[i];
                }
    }
}

// mouse button callback
void MjRender::mouseButton(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    buttonLeft   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    buttonMiddle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    buttonRight  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastX, &lastY);
}


// mouse move callback
void MjRender::mouseMove(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !buttonLeft && !buttonMiddle && !buttonRight )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastX;
    double dy = ypos - lastY;
    lastX = xpos;
    lastY = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( buttonRight )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( buttonLeft )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void MjRender::scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}