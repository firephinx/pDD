#pragma once
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include "units.h"
#include "utils.h"

// Include GLEW
#include "GL/glew.h"

// Include GLFW
#include "glfw3.h"

// Include GLM
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

#include <png.h>

using namespace std;


// some helpers
namespace OpenGLHelper {
GLuint LoadTexturePNG(const char * file_name, int * width, int * height);
GLuint LoadShaders();  
}

class OBJModel {
public:
  OBJModel();
  void Init(const vector<SpaceCoord> &vertices, const vector<int> &triangles, 
    const vector<ImCoord> &uvs, const vector<int> &uv_triangles, 
    const vector<int> &texture_assignment, const vector<Material> &materials);
  void RenderOBJ(cv::Mat &image, Pose pose, Intrinsics intrinsics);
  bool isOK();
  void Release();
  ~OBJModel();
private:
  bool isInit;
  GLFWwindow* window;
  GLuint vertexbuffer, uvbuffer, Texture;
  // shader variables
  GLuint programID, MatrixID, vertexPosition_modelspaceID, vertexUVID, 
    TextureID;
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec2> uvs;
};

