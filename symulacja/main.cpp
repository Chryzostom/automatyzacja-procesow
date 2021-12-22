// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "main.hpp"

using namespace Eigen;
using namespace std;

#define read_gravity !(m->opt.disableflags & (mjDSBL_GRAVITY))
#define gravity_off  m->opt.disableflags |= mjDSBL_GRAVITY
#define gravity_on  m->opt.disableflags &=~ mjDSBL_GRAVITY

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// OpenGL rendering and UI
GLFWvidmode vmode;
int windowpos[2];
int windowsize[2];
GLFWwindow* window = NULL;
mjuiState uistate;
mjUI ui0, ui1;
// info strings
char info_title_l[1000];
char info_content_l[1000];
char info_title_r[1000];
char info_content_r[1000];

float target_pos = 0.0;
float new_target_pos = 0.0;
float target_angle = 1.57;
float I,elast,xpos_last;
int i=0;


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    (void)window;
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
    else if( act==GLFW_PRESS && key==GLFW_KEY_B )
    {
        if(opt.frame != mjFRAME_NONE)opt.frame = mjFRAME_NONE;
        else opt.frame = mjFRAME_BODY;
    }
    else if( act==GLFW_PRESS && key==GLFW_KEY_G )
    {
        if(read_gravity != 0)gravity_off;
        else gravity_on;
    }
    else if( act==GLFW_PRESS && key==GLFW_KEY_UP )
    {
        new_target_pos += -0.5;
    }
    else if( act==GLFW_PRESS && key==GLFW_KEY_DOWN )
    {
        new_target_pos += 0.5;
    }
    // else
    // {
    //     target_pos = 0.0;
    // }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    (void)xoffset;
    (void)yoffset;
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


void mycontroller(const mjModel* m, mjData* d)
{

        Quaternion<float> q((float)d->xquat[4], (float)d->xquat[5], (float)d->xquat[6],(float)d->xquat[7]);
        auto euler = q.toRotationMatrix().eulerAngles(0,1,2);

        Matrix4f transf;
        transf << d->xmat[9],d->xmat[10],d->xmat[11],d->xpos[3],
                 d->xmat[12],d->xmat[13],d->xmat[14],d->xpos[4],
                 d->xmat[15],d->xmat[16],d->xmat[17],d->xpos[5],
                          0,         0,         0,   1.0f;

        Vector4f a;
        a << euler(0), euler(1), euler(2), 1.0f;

        Vector4f local;

        local = transf * a;

        
        strcpy(info_title_l, "roll [rad]\npitch [rad]\nyaw [rad]\ntarget_pos\n actual pos");
        // sprintf(info_content_l, "%.3f\n%.3f\n%.3f", local(0), local(1), local(2));
        sprintf(info_content_l, "%.3f\n%.3f\n%.3f\n%.3f\n%.3f", euler(0)+3.14, euler(1), euler(2),target_pos,d->xpos[4]);


       
        if(i%5)
        {
            target_pos = 0.99f * target_pos + 0.01f * new_target_pos;

            target_angle = (1.57f - 0.6f*(target_pos - d->xpos[4])) - 180.0f*(xpos_last - d->xpos[4]);
            i=0;
            xpos_last = d->xpos[4];
        }

        float kp = 4.5f;
        float ki = 0.0f;

        float e = euler(0)-target_angle;

        I+= (e + elast)/2.0f;

        float I_max = 1.0f;
        if(I>I_max)I=I_max;
        else if(I<-I_max)I=-I_max;

        d->ctrl[0] = -kp*(e) + ki*(I);
        d->ctrl[1] = kp*(e) + ki*(I);

        elast = e;

        i++;
}


// main function
int main(int argc, const char** argv)
{
    //load and compile model
    char error[1000] = "Could not load binary model";
    m = mj_loadXML("C:/Users/klonyyy/piotrek_moje/studia/Magisterka/Systemy_automatyzacji/projekt/sim/tttrochim/symulacja/model/wahadlo.xml", NULL, error, 1000);
    if( !m )mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
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
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

        // init state and uis
    // memset(&uistate, 0, sizeof(mjuiState));
    // memset(&ui0, 0, sizeof(mjUI));
    // memset(&ui1, 0, sizeof(mjUI));
    // ui0.spacing = mjui_themeSpacing(0);
    // ui0.color = mjui_themeColor(0x551122);
    // ui0.rectid = 1;
    // ui0.auxid = 0;
    // uiModify(window, &ui0, &uistate, &con);
    m->opt.gravity[0] = 0;
    m->opt.gravity[1] = 0;
    m->opt.gravity[2] = -9.81;

    mjcb_control = mycontroller;

    // run main loop, target real-time simulation and 60 fps rendering
    while( !glfwWindowShouldClose(window) )
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
        {
            mj_step(m, d);
        }

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        strcpy(info_title_r, "gravity:");
        sprintf(info_content_r, "%d", !(m->opt.disableflags & (mjDSBL_GRAVITY)));
        /* overaly data on the scene */
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,info_title_l, info_content_l, &con);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport,info_title_r, info_content_r, &con);
        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);
        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data
    mj_deleteData(d);
    mj_deleteModel(m);

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}