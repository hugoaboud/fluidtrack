/*
*   FluidMap
*   Open Source library for tracking objects on terrain fluid flows
*
*   usage:
*   ./fluidtrack <terrain.csv> <flow.flow> <output.csv>
*
*   @author: Hugo Aboud/
*/

#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "reactphysics3d.h"
using namespace reactphysics3d;

#include "picojson.h"

/**
  ReactPhysics3D
**/

rp3d::RigidBody* createRigidBody(rp3d::DynamicsWorld* world, rp3d::Vector3 position, rp3d::Quaternion orientation, BodyType bodyType, float bounciness = 1, float friction = 0, float rollingResistance = 0) {

  rp3d::RigidBody* body;
  rp3d::Transform transform(position, orientation);
  body = world->createRigidBody(transform);
  body->setType(bodyType);

  rp3d::Material& material = body->getMaterial();
  material.setBounciness(rp3d::decimal(bounciness));
  material.setFrictionCoefficient(rp3d::decimal(friction));
  material.setRollingResistance(rp3d::decimal(rollingResistance));

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
  float scalex;
  float scaley;
  float minHeight;
  float maxHeight;
  float* heights;
};

Terrain* parseTerrain(std::string csv_path, int cols, int rows, float scalex, float scaley) {

  // Open CSV (vertex) file
  std::ifstream file(csv_path);
  std::cout << "Parsing Terrain from file: " << csv_path << std::endl;
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
    //return NULL;
  }

  std::cout << "\tMin. Height: " << minHeight << std::endl;
  std::cout << "\tMax. Height: " << maxHeight << std::endl;

  // Sort vertex on y/x axis
  std::sort(vertex.begin(), vertex.end(), vertexSort);

  // Create binary:
  Terrain* terrain = new Terrain();
  terrain->cols = cols;
  terrain->rows = rows;
  terrain->scalex = (scalex/2)/((cols-1)/2.0);
  terrain->scaley = (scaley/2)/((rows-1)/2.0);
  terrain->minHeight = minHeight;
  terrain->maxHeight = maxHeight;
  terrain->heights = new float[vertex.size()]();
  for (int v = 0; v < int(vertex.size()); v++) terrain->heights[v] = vertex[v][2];

  // Delete vertex
  for (float* v : vertex) delete[] v;

  return terrain;
}

rp3d::RigidBody* createTerrain(rp3d::DynamicsWorld* world, Terrain* terrain, float bounciness, float friction, float rollingResistance) {
  std::cout << "Creating Terrain" << std::endl;
  std::cout << "\tbounciness: " << bounciness << std::endl;
  std::cout << "\tfriction: " << friction << std::endl;
  std::cout << "\trollingResistance: " << rollingResistance << std::endl;
  rp3d::RigidBody* rb = createRigidBody(world, rp3d::Vector3(0,0,0), rp3d::Quaternion::identity(), BodyType::STATIC, bounciness, friction, rollingResistance);
  rp3d::HeightFieldShape* shape = new rp3d::HeightFieldShape(
    terrain->cols,
    terrain->rows,
    rp3d::decimal(terrain->minHeight),
    rp3d::decimal(terrain->maxHeight),
    terrain->heights,
    rp3d::HeightFieldShape::HeightDataType::HEIGHT_FLOAT_TYPE,
    2, 1.0f,
    rp3d::Vector3(terrain->scalex,terrain->scaley,1)
  );

  rp3d::Vector3 min, max;
  shape->getLocalBounds(min, max);

  printf("min: %.3f, %.3f, %.3f\n", min.x, min.y, min.z);
  printf("max: %.3f, %.3f, %.3f\n", max.x, max.y, max.z);

  rb->addCollisionShape(shape, rp3d::Transform::identity(), rp3d::decimal(0.0));
  return rb;
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

void updateFlow(Flow* flow, rp3d::RigidBody* object, float thresh, int f) {
  rp3d::Vector3 obj_pos = object->getTransform().getPosition();
  std::vector<float*> actors;
  for (float* vertex : *((*flow)[f])) {
    float dist = sqrt(
      pow((vertex[0]-obj_pos.x),2) +
      pow((vertex[1]-obj_pos.y),2) +
      pow((vertex[2]-obj_pos.z),2)
    );
    if (dist <= thresh) actors.push_back(vertex);
  }

  if (!actors.size()) return;
  float pressure = 0.01/sqrt(actors.size());
  for (float* actor : actors) {
      object->applyForce(
        rp3d::Vector3(actor[3]*pressure, actor[4]*pressure, actor[5]*pressure),
        rp3d::Vector3(actor[0], actor[1], actor[2])
      );
  }
}

/**
  Object
**/

rp3d::RigidBody* createSphereObject(rp3d::DynamicsWorld* world, rp3d::Vector3 position, float radius, float mass, float bounciness, float friction, float rollingResistance) {
  std::cout << "Creating Sphere Object" << std::endl;
  std::cout << "\tposition: " << position.x << "," << position.y << "," << position.z << std::endl;
  std::cout << "\tradius: " << radius << std::endl;
  std::cout << "\tmass: " << mass << std::endl;
  std::cout << "\tbounciness: " << bounciness << std::endl;
  std::cout << "\tfriction: " << friction << std::endl;
  std::cout << "\trollingResistance: " << rollingResistance << std::endl;
  rp3d::RigidBody* object = createRigidBody(world, position, rp3d::Quaternion::identity(), BodyType::DYNAMIC, bounciness, friction, rollingResistance);
  rp3d::SphereShape* objectShape = new rp3d::SphereShape(rp3d::decimal(radius));
  object->addCollisionShape(objectShape, rp3d::Transform::identity(), rp3d::decimal(mass));
  return object;
}

/**
  Main
**/

int main(int argc, char *argv[]) {

  // Load configuration JSON
  picojson::object config;
  if (argc < 2) {
    printf("No config file specified (arg 1)\n");
    return 1;
  }
  else {
    picojson::value parser;
    std::ifstream t(argv[1]);
    std::stringstream buffer;
    buffer << t.rdbuf();
    std::string err = picojson::parse(parser, buffer.str());
    if (!err.empty()) {
      std::cerr << err << std::endl;
      exit(1);
    }
    if (!parser.is<picojson::object>()) {
      std::cerr << "JSON is not an object" << std::endl;
      exit(2);
    }
    config = parser.get<picojson::object>();
  }

  // Gravity
  picojson::object config_world = config["world"].get<picojson::object>();
  picojson::array config_gravity = config_world["gravity"].get<picojson::array>();
  rp3d::Vector3 gravity(
    config_gravity[0].get<double>(),
    config_gravity[1].get<double>(),
    config_gravity[2].get<double>()
  );
  printf("Gravity: [%.3f, %.3f, %.3f]\n", gravity.x, gravity.y, gravity.z);

  // Create the dynamics world
  rp3d::DynamicsWorld world(gravity);
  world.setNbIterationsVelocitySolver(15);
  world.setNbIterationsPositionSolver(8);

  picojson::object config_input = config["input"].get<picojson::object>();
  std::string flow_path = config_input["flow"].to_str();
  std::string output_path = config["output"].to_str();

  picojson::object config_terrain = config["terrain"].get<picojson::object>();
  Terrain* terrainStruct = parseTerrain(
    config_input["terrain"].to_str(),
    config_terrain["cols"].get<double>(),
    config_terrain["rows"].get<double>(),
    config_terrain["scalex"].get<double>(),
    config_terrain["scaley"].get<double>()
  );
  if (!terrainStruct) return 1;

  createTerrain(&world, terrainStruct,
    config_terrain["bounciness"].get<double>(),
    config_terrain["friction"].get<double>(),
    config_terrain["rollingResistance"].get<double>()
  );

  // Create the Sample Object
  picojson::object config_object = config["object"].get<picojson::object>();
  picojson::array object_pos = config_object["position"].get<picojson::array>();

  rp3d::RigidBody* object = createSphereObject(&world,
    rp3d::Vector3(
      object_pos[0].get<double>(),
      object_pos[1].get<double>(),
      object_pos[2].get<double>()),
    config_object["radius"].get<double>(),
    config_object["mass"].get<double>(),
    config_object["bounciness"].get<double>(),
    config_object["friction"].get<double>(),
    config_object["rollingResistance"].get<double>()
  );
  rp3d::SphereShape objectShape(rp3d::decimal(0.5));
  object->addCollisionShape(&objectShape, rp3d::Transform::identity(), rp3d::decimal(1));

  // Parse the flow file
  picojson::object config_flow = config["flow"].get<picojson::object>();
  float timeStep = 0;
  Flow* flow = parseFlow(flow_path, &timeStep);

  // Update World by time step and write position to output file
  std::ofstream outputFile(output_path);
  for (int f=0; f<int(flow->size()); f++) {
    updateFlow(flow, object, config_flow["threshold"].get<double>(), f);
    world.update(timeStep);

    // Print the updated transform of the body
    rp3d::Vector3 curPosition = object->getTransform().getPosition();
    rp3d::Quaternion curRotation = object->getTransform().getOrientation();
    printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", curPosition.x, curPosition.y, curPosition.z, curRotation.w, curRotation.x, curRotation.y, curRotation.z);
    outputFile << curPosition.x << "," << curPosition.y << "," << curPosition.z << "," << curRotation.w << "," << curRotation.x << "," << curRotation.y << "," << curRotation.z << std::endl;
  }

  // Cleanup before leaving
  delete terrainStruct;
  for (std::vector<float*>* frame : *flow) {
    for (float* vertex : *frame) delete[] vertex;
    delete frame;
  }
  delete flow;

  return 0;
}
