/*
 * SDL2 skeleton for lab assignments 1–3 of the KTH course DH2323,
 * Computer Graphics and Interaction (and also SU DA3001, Datalogi II).
 *
 * See README.md for details.
 */

#include <iostream>
#include </usr/local/include/glm/glm.hpp>
#include <vector>
#include "SDL2Auxiliary.h"
#include "TestModel.h"

using namespace std;
using glm::mat3;
using glm::vec3;

// --------------------------------------------------------
// GLOBAL VARIABLES

const int SCREEN_WIDTH = 500;
const int SCREEN_HEIGHT = 500;
SDL2Aux *sdlAux;
SDL_Surface *screen;
int t;

vector<Triangle> triangles;

struct Intersection
{
  vec3 position;
  float distance;
  int triangleIndex;
};

float focalLength = SCREEN_HEIGHT;
vec3 cameraPos(0, 0, -3);

mat3 R = mat3(vec3(1, 0, 0), vec3(0, 1, 0), vec3(0, 0, 1)); // rotation matrix for camera
float yaw;                                                  // stored angle for camera to rotate around y axis
float pi = 3.14159265359;

vec3 lightPos(0, -0.5, -0.7);
vec3 lightColor = 14.f * vec3(1, 1, 1);

vec3 indirectLight = 0.5f * vec3(1, 1, 1);

// --------------------------------------------------------
// FUNCTION DECLARATIONS

void Draw();
void Update();
bool ClosestIntersection(
    vec3 start,
    vec3 dir,
    const vector<Triangle> &triangles,
    Intersection &closestIntersection);
void Rotate();
vec3 DirectLight(const Intersection &i);

// --------------------------------------------------------
// FUNCTION DEFINITIONS

int main(int argc, char *argv[])
{
  sdlAux = new SDL2Aux(SCREEN_WIDTH, SCREEN_HEIGHT);
  t = SDL_GetTicks(); // Set start value for timer.
  LoadTestModel(triangles);

  while (!sdlAux->quitEvent())
  {
    Update();
    Draw();
  }

  // sdlAux->saveBMP("screenshot.bmp");
  return 0;
}

void Draw()
{
  // loop through all pixels and compute the corresponding ray direction.
  // Then call the function ClosestIntersection to get the closest intersection in that direction.
  // If there was an intersection the color of the pixel should be set to the color of that triangle.
  // Otherwise it should be black.
  sdlAux->clearPixels();

  for (int y = 0; y < SCREEN_HEIGHT; ++y)
  {
    for (int x = 0; x < SCREEN_WIDTH; ++x)
    {
      // vector d pointing in the direction where the light reaching pixel (x,y) comes from
      vec3 direction((x - SCREEN_WIDTH / 2), (y - SCREEN_HEIGHT / 2), focalLength);
      Intersection intersection;
      direction = R * glm::normalize(direction); // update direction of camera with rotation matrix, so that rays go from the right place
      if (ClosestIntersection(cameraPos, direction, triangles, intersection))
      {
        vec3 light = DirectLight(intersection);
        vec3 color = triangles[intersection.triangleIndex].color;
        vec3 colornlight = color * (light + indirectLight);
        sdlAux->putPixel(x, y, colornlight);
      }
      else
      {
        sdlAux->putPixel(x, y, vec3(0, 0, 0));
      }
    }
  }

  sdlAux->render();
}

bool ClosestIntersection(vec3 start,
                         vec3 dir,
                         const vector<Triangle> &triangles,
                         Intersection &closestIntersection)
{
  // If an intersection occurred it should return true. Otherwise false.
  // In the case of an intersection it should also return some information about the closest intersection.

  float m = std::numeric_limits<float>::max();
  bool intersect = false;

  for (int i = 0; i < triangles.size(); i++)
  {
    using glm::mat3;
    using glm::vec3;
    vec3 v0 = triangles[i].v0;
    vec3 v1 = triangles[i].v1;
    vec3 v2 = triangles[i].v2;
    vec3 e1 = v1 - v0;
    vec3 e2 = v2 - v0;
    vec3 b = start - v0;
    mat3 A(-dir, e1, e2);
    vec3 x = glm::inverse(A) * b;

    // cramers rule (?)
    float t = x[0];
    float u = x[1];
    float v = x[2];

    // equations 7,8,9 & 11, they all have to be true
    if (u >= 0 && v >= 0 && u + v <= 1 && t >= 0)
    {
      vec3 intersection = start + (t * dir);
      float distance = glm::distance(intersection, start);
      if (distance < m)
      {
        // here, m is updated with the distance. if not done, objects will not be placed in right order.
        m = distance;
        closestIntersection.distance = distance;
        closestIntersection.position = intersection;
        closestIntersection.triangleIndex = i;
      }
      intersect = true;
    }
  }
  return intersect;
}

/*void Update()
{
  // Compute frame time:
  int t2 = SDL_GetTicks();
  float dt = float(t2 - t);
  t = t2;
  // cout << "Render time: " << dt << " ms." << endl;
}*/

void Rotate()
{
  R = mat3(cos(yaw), 0, sin(yaw), 0, 1, 0, -sin(yaw), 0, cos(yaw));
}

void Update()
{
  // Compute frame time:
  int t2 = SDL_GetTicks();
  float dt = float(t2 - t);
  t = t2;
  cout << "Render time: " << dt << " ms." << endl;

  const Uint8 *state = SDL_GetKeyboardState(NULL);
  float anglechange = ((2.0f * pi) / 1000.0f) * dt;

  // from instructions: vectors representing the axes directions that we update R for
  vec3 right(R[0][0], R[0][1], R[0][2]);   // x axis
  vec3 down(R[1][0], R[1][1], R[1][2]);    // y axis
  vec3 forward(R[2][0], R[2][1], R[2][2]); // z axis

  // controls camera. i am not sure of the signs for change, + or - gives different directions.
  if (state[SDL_SCANCODE_UP])
  {
    cameraPos += anglechange * forward;
  }
  if (state[SDL_SCANCODE_DOWN])
  {
    cameraPos -= anglechange * forward;
  }
  if (state[SDL_SCANCODE_RIGHT])
  {
    cameraPos += anglechange * right;
  }
  if (state[SDL_SCANCODE_LEFT])
  {
    cameraPos -= anglechange * right;
  }
  if (state[SDL_SCANCODE_Z])
  {
    yaw += anglechange;
    // cameraPos += anglechange * right;
    Rotate();
  }
  if (state[SDL_SCANCODE_X])
  {
    yaw -= anglechange;
    // cameraPos += anglechange * right;
    Rotate();
  }

  if (state[SDL_SCANCODE_W])
  {
    lightPos += anglechange * forward;
  }

  if (state[SDL_SCANCODE_S])
  {
    lightPos -= anglechange * forward;
  }

  if (state[SDL_SCANCODE_A])
  {
    lightPos += anglechange * right;
  }

  if (state[SDL_SCANCODE_D])
  {
    lightPos -= anglechange * right;
  }

  if (state[SDL_SCANCODE_Q])
  {
    lightPos += anglechange * down;
  }

  if (state[SDL_SCANCODE_E])
  {
    lightPos -= anglechange * down;
  }
}

vec3 DirectLight(const Intersection &i)
{
  // Let n̂ be a unit vector describing the normal pointing out from the surface
  // and let r̂ be a unit vector describing the direction from the surface point to the light source.
  vec3 n = triangles[i.triangleIndex].normal;
  vec3 r = lightPos - i.position;
  vec3 rnorm = glm::normalize(r);
  vec3 D;

  float r_length = glm::length(r);

  // surface area A = 4 * pi * r^2

  // We get the fraction between D and B by the projection of r̂ on n̂, i.e. a scalar product of two unit
  // vectors. Since we do not allow negative light we take the max of the projected value and zero. If the
  // angle between the surface normal and direction to the light source is larger than 90 degrees it does
  // not receive any direct illumination.
  Intersection intersection;
  if (ClosestIntersection(lightPos, -rnorm, triangles, intersection))
  {
    // if (angle > 90)
    if (intersection.distance < glm::distance(lightPos, i.position) - 0.001f)
    {
      return vec3(0, 0, 0);
    }
  }

  // P is lightColor, with power for each color component
  // D = B max(r̂ . n̂ , 0) = (P max (r̂ . n̂ , 0))/4πr^2
  D = vec3(lightColor * glm::max(glm::dot(rnorm, n), 0.0f)) / (4.0f * glm::pow(r_length, 2.0f) * pi);

  return D;
};