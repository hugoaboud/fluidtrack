/*
*   FluidMap
*   Open Source library for tracking objects on terrain fluid flows
*
*   usage:
*   ./fluidtrack <terrain.csv> <flow.flow> <output.csv>
*
*   @author: Hugo Aboud
*/

#include "reactphysics3d.h"
using namespace reactphysics3d;

#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

/**
  ReactPhysics3D
**/

rp3d::RigidBody* createRigidBody(rp3d::DynamicsWorld* world, rp3d::Vector3 position, rp3d::Quaternion orientation, BodyType bodyType, float bounciness = 1, float frictionCoeff = 0) {

  rp3d::RigidBody* body;
  rp3d::Transform transform(position, orientation);
  body = world->createRigidBody(transform);
  body->setType(bodyType);

  rp3d::Material& material = body->getMaterial();
  material.setBounciness(rp3d::decimal(bounciness));
  material.setFrictionCoefficient(rp3d::decimal(frictionCoeff));

  return body;
}

bool vertexSort(float* a, float* b) {
  if (a[1] < b[1]) return true;
  else return a[1] == b[1] && a[0] < b[0];
  return false;
}

/**
  Terrain
**/

struct Terrain {
  int cols;
  int rows;
  float minHeight;
  float maxHeight;
  float* heights;
};

Terrain* parseTerrain(std::string csv_path, int cols, int rows) {

  // Open CSV (vertex) file
  std::ifstream file(csv_path);
  std::cout << "Parsin Terrain from file: " << csv_path << std::endl;
  std::cout << "\tw: " << cols << ", h: " << rows << std::endl;

  std::vector<float*> vertex;
  float maxHeight = std::numeric_limits<float>::min();
  float minHeight = std::numeric_limits<float>::max();

  // Parse CSV file
  // also computes min/max height
  std::string line;
  while (std::getline(file, line)) {
    float* v = new float[3]();
    sscanf(line.c_str(), "%f,%f,%f", &v[0], &v[1], &v[2]);
    if (v[2] < minHeight) minHeight = v[2];
    if (v[2] > maxHeight) maxHeight = v[2];
    vertex.push_back(v);
  }

  // Debug Output
  std::cout << "\tVertex count: " << vertex.size() << std::endl;
  if (int(vertex.size()) != cols * rows) {
    std::cout << "\tWrong vertex count!" << std::endl;
    return NULL;
  }

  std::cout << "\tMin. Height: " << minHeight << std::endl;
  std::cout << "\tMax. Height: " << maxHeight << std::endl;

  // Sort vertex on y/x axis
  std::sort(vertex.begin(), vertex.end(), vertexSort);

  // Create binary:
  Terrain* terrain = new Terrain();
  terrain->cols = cols;
  terrain->rows = rows;
  terrain->minHeight = minHeight;
  terrain->maxHeight = maxHeight;
  terrain->heights = new float[vertex.size()]();
  for (int v = 0; v < int(vertex.size()); v++) terrain->heights[v] = vertex[v][2];

  // Delete vertex
  for (float* v : vertex) delete[] v;

  return terrain;
}

rp3d::RigidBody* createTerrain(rp3d::DynamicsWorld* world, Terrain* binary, float bounciness, float frictionCoeff) {
  std::cout << "Creating Terrain from binary" << std::endl;
  rp3d::RigidBody* terrain = createRigidBody(world, rp3d::Vector3(0,0,0), rp3d::Quaternion::identity(), BodyType::STATIC, bounciness, frictionCoeff);
  rp3d::HeightFieldShape* shape = new rp3d::HeightFieldShape(
    binary->cols,
    binary->rows,
    rp3d::decimal(binary->minHeight),
    rp3d::decimal(binary->maxHeight),
    binary->heights,
    rp3d::HeightFieldShape::HeightDataType::HEIGHT_FLOAT_TYPE,
    2, 1.0f,
    rp3d::Vector3(1.5,1.5,1)
  );

  terrain->addCollisionShape(shape, rp3d::Transform::identity(), rp3d::decimal(0.0));
  return terrain;
}

/**
  Flow
**/

typedef std::vector<std::vector<float*>*> Flow;

Flow* parseFlow(std::string flow_path, float* timeStep) {
  // Open flow file
  std::ifstream file(flow_path, std::ios::binary);
  std::cout << "Parsing Flow from file: " << flow_path << std::endl;

  int frames;
  file.read((char*)&frames, 4);

  int anim_time;
  file.read((char*)&anim_time, 4);

  Flow* flow = new Flow();
  for (int f = 0; f < frames; f++) {
    int vcount;
    file.read((char*)&vcount, 4);
    std::vector<float*>* frame = new std::vector<float*>();
    for (int v = 0; v < vcount; v++) {
      float* vertex = new float[6]();
      file.read((char*)&vertex[0], 4);
      file.read((char*)&vertex[1], 4);
      file.read((char*)&vertex[2], 4);
      file.read((char*)&vertex[3], 4);
      file.read((char*)&vertex[4], 4);
      file.read((char*)&vertex[5], 4);
      frame->push_back(vertex);
    }
    flow->push_back(frame);
  }

  std::cout << "\tFrame count: " << flow->size() << std::endl;
  std::cout << "\tVertex count: [";
  for (std::vector<float*>* frame : *flow) {
    std::cout << frame->size() << ",";
  }
  std::cout << "]" << std::endl;

  *(timeStep) = float(anim_time)/frames;
  std::cout << "\tTime Step: " << float(anim_time)/frames << " seconds" << std::endl;

  return flow;
}

void updateFlow(Flow* flow, rp3d::RigidBody* object, int f) {
  rp3d::Vector3 obj_pos = object->getTransform().getPosition();
  std::vector<float*> actors;
  for (float* vertex : *((*flow)[f])) {
    float dist = sqrt(
      pow((vertex[0]-obj_pos.x),2) +
      pow((vertex[1]-obj_pos.y),2) +
      pow((vertex[2]-obj_pos.z),2)
    );
    if (dist <= 0.5) actors.push_back(vertex);
  }

  if (!actors.size()) return;
  float pressure = 1/sqrt(actors.size());
  for (float* actor : actors) {
      object->applyForce(
        rp3d::Vector3(actor[3]*pressure, actor[4]*pressure, actor[5]*pressure),
        rp3d::Vector3(actor[0], actor[1], actor[2])
      );
  }
}

/**
  Main
**/

int main(int argc, char *argv[]) {

  // Gravity
  rp3d::Vector3 gravity(0.0, 0.0, -9.81);

  // Create the dynamics world
  rp3d::DynamicsWorld world(gravity);
  world.setNbIterationsVelocitySolver(15);
  world.setNbIterationsPositionSolver(8);

  if (argc < 2) {
    printf("No terrain (.csv) file specified (arg 1)\n");
    return 1;
  }
  std::string terrain_path = argv[1];

  if (argc < 3) {
    printf("No flow file specified (arg 2)\n");
    return 1;
  }
  std::string flow_path = argv[2];

  if (argc < 4) {
    printf("No output file specified (arg 3)\n");
    return 1;
  }
  std::string output_path = argv[3];
  std::ofstream outputFile(output_path);

  Terrain* terrainBinary = parseTerrain(terrain_path, 5, 5);
  createTerrain(&world, terrainBinary, 0.05, 0.6);

  // Parse the flow file
  float timeStep = 0;
  Flow* flow = parseFlow(flow_path, &timeStep);

  // Create the Sample Object
  rp3d::RigidBody* object = createRigidBody(&world, rp3d::Vector3(-1.2,1,0.5), rp3d::Quaternion::identity(), BodyType::DYNAMIC, 0.1, 0.3);
  rp3d::SphereShape objectShape(rp3d::decimal(0.5));
  object->addCollisionShape(&objectShape, rp3d::Transform::identity(), rp3d::decimal(1));

  // Update World by time step and write position to output file
  for (int f=0; f<int(flow->size()); f++) {
    updateFlow(flow, object, f);
    world.update(timeStep);

    // Print the updated transform of the body
    rp3d::Vector3 curPosition = object->getTransform().getPosition();
    rp3d::Quaternion curRotation = object->getTransform().getOrientation();
    printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", curPosition.x, curPosition.y, curPosition.z, curRotation.w, curRotation.x, curRotation.y, curRotation.z);
    outputFile << curPosition.x << "," << curPosition.y << "," << curPosition.z << "," << curRotation.w << "," << curRotation.x << "," << curRotation.y << "," << curRotation.z << std::endl;
  }

  // Cleanup before leaving
  delete terrainBinary;

  for (std::vector<float*>* frame : *flow) {
    for (float* vertex : *frame) delete[] vertex;
    delete frame;
  }
  delete flow;

  return 0;
}
