#include "simulation.h"

#include <GLFW/glfw3.h>
#include <mujoco/mjvisualize.h>

#include <utility>
#include <Eigen/Core>
#include "Eigen/src/Core/Matrix.h"

static void keyboard_callback(GLFWwindow* window, int key, int scancode, int act, int mods) {
  Simulation* sim = reinterpret_cast<Simulation*>(glfwGetWindowUserPointer(window));
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
    mj_resetData(sim->model, sim->data);
    mj_forward(sim->model, sim->data);
  }
}

static void mouse_button_callback(GLFWwindow* window, int button, int act, int mods) {
  Simulation* sim = reinterpret_cast<Simulation*>(glfwGetWindowUserPointer(window));
  // Update button state
  sim->button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  sim->button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  sim->button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // Update mouse position
  double xpos, ypos;
  glfwGetCursorPos(window, &xpos, &ypos);
  sim->lastx = xpos;
  sim->lasty = ypos;
}

static void mouse_move_callback(GLFWwindow* window, double xpos, double ypos) {
  Simulation* sim = reinterpret_cast<Simulation*>(glfwGetWindowUserPointer(window));
  // Compute mouse displacement, save
  double dx = xpos - sim->lastx;
  double dy = ypos - sim->lasty;
  sim->lastx = xpos;
  sim->lasty = ypos;

  // No buttons down: nothing to do
  if (!sim->button_left && !sim->button_middle && !sim->button_right) {
    return;
  }

  // Get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // Get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
                   (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // Determine action based on mouse button
  int action;
  if (sim->button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (sim->button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  mjv_moveCamera(sim->model, action, dx / height, dy / height, &sim->scene, &sim->cam);
}

static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
  Simulation* sim = reinterpret_cast<Simulation*>(glfwGetWindowUserPointer(window));
  int action = mjMOUSE_ZOOM;
  mjv_moveCamera(sim->model, action, 0.0, -0.05 * yoffset, &sim->scene, &sim->cam);
}

Simulation::Simulation(const std::string& xml_path) {
  // Load model from XML file
  char error[1000];
  model = mj_loadXML(xml_path.c_str(), nullptr, error, 1000);
  if (!model) {
    std::cerr << "Could not load model: " << error << std::endl;
    throw std::runtime_error("Model loading failed");
  }
  // Create data structure for simulation
  data = mj_makeData(model);
  // Set up camera and visualization options
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  // Initialize GLFW
  if (!glfwInit()) {
    std::cerr << "Could not initialize GLFW." << std::endl;
    throw std::runtime_error("GLFW initialization failed");
  }
  // Create window
  window = glfwCreateWindow(1200, 900, "Mujoco", nullptr, nullptr);
  if (!window) {
    glfwTerminate();
    std::cerr << "Could not create GLFW window." << std::endl;
    throw std::runtime_error("Window creation failed");
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  glfwSetWindowUserPointer(window, reinterpret_cast<void*>(this));
  // Initialize visualization data structures
  mjv_defaultScene(&scene);
  mjv_makeScene(model, &scene, 10000);
  mjr_defaultContext(&context);
  mjr_makeContext(model, &context, mjFONTSCALE_150);

  // Install GLFW callbacks (you need to implement these functions)
  glfwSetKeyCallback(window, keyboard_callback);
  glfwSetCursorPosCallback(window, mouse_move_callback);
  glfwSetMouseButtonCallback(window, mouse_button_callback);
  glfwSetScrollCallback(window, scroll_callback);
  glfwGetFramebufferSize(window, &viewport_width, &viewport_height);
  // enable contact force visualization
  opt.flags[mjtVisFlag::mjVIS_CONTACTFORCE] = 1;
}

Simulation::~Simulation() {
  mj_deleteData(data);
  mj_deleteModel(model);
  glfwDestroyWindow(window);
  glfwTerminate();
}

void Simulation::Step(Eigen::VectorXd tau) {
  // set control torque
  for (int i = 0; i < tau.size(); i++) {
    data->ctrl[i] = tau[i];
  }
  mj_step(model, data);
  
  for(int i=0;i<10;i++){
    qpos(i) = data->qpos[i];
    qvel(i) = data->qvel[i];
  }
  
  // visualization
  counter++;
  if (counter >= 20) {
    // Get framebuffer viewport
    mjrRect viewport = {0, 0, viewport_width, viewport_height};
    // Update scene and render
    mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, &scene);
    mjr_render(viewport, &scene, &context);
    // Swap buffers and poll IO events
    glfwSwapBuffers(window);
    glfwPollEvents();
    counter = 0;
  }
}