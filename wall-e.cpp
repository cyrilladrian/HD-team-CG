/*
 * Task 3: Cubism Animal via Hierarchical Modelling
 *
 * Build a blocky animal (or robot) from transformed unit cubes arranged
 * in a parent-child tree. Export the result as a Wavefront OBJ file.
 *
 * Build:  mkdir build && cd build && cmake .. && make && ./task3_cubism_animal
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// ---- Data Structures ----

struct Triangle
{
    int v0, v1, v2; // 0-based vertex indices
};

struct Mesh
{
    vector<Vector3f> vertices;
    vector<Triangle> faces;
};

struct ModelNode
{
    string name;
    Vector3f position;
    Vector3f rotation;
    Vector3f scale;

    vector<unique_ptr<ModelNode>> children;

    ModelNode(const string &name,
              const Vector3f &pos = Vector3f::Zero(),
              const Vector3f &rot = Vector3f::Zero(),
              const Vector3f &scl = Vector3f::Ones())
        : name(name), position(pos), rotation(rot), scale(scl) {}

    ModelNode *addChild(const string &name,
                        const Vector3f &pos,
                        const Vector3f &scl,
                        const Vector3f &rot = Vector3f::Zero())
    {
        children.push_back(make_unique<ModelNode>(name, pos, rot, scl));
        return children.back().get();
    }
};

// ---- Transformation Matrices ----

// TODO
Matrix4f translationMatrix(const Vector3f &t)
{
    Matrix4f M = Matrix4f::Identity();
    M(0, 3) = t.x();
    M(1, 3) = t.y();
    M(2, 3) = t.z();
    return M;
}

// TODO
Matrix4f scalingMatrix(const Vector3f &s)
{
    Matrix4f M = Matrix4f::Identity();
    M(0, 0) = s.x();
    M(1, 1) = s.y();
    M(2, 2) = s.z();
    return M;
}

// TODO (remember to convert degrees to radians)
Matrix4f rotationX(float degrees)
{
    float r = degrees * M_PI / 180.0f;
    Matrix4f M = Matrix4f::Identity();
    M(1, 1) = cos(r);
    M(1, 2) = -sin(r);
    M(2, 1) = sin(r);
    M(2, 2) = cos(r);
    return M;
}

// TODO
Matrix4f rotationY(float degrees)
{
    float r = degrees * M_PI / 180.0f;
    Matrix4f M = Matrix4f::Identity();
    M(0, 0) = cos(r);
    M(0, 2) = sin(r);
    M(2, 0) = -sin(r);
    M(2, 2) = cos(r);
    return M;
}

// TODO
Matrix4f rotationZ(float degrees)
{
    float r = degrees * M_PI / 180.0f;
    Matrix4f M = Matrix4f::Identity();
    M(0, 0) = cos(r);
    M(0, 1) = -sin(r);
    M(1, 0) = sin(r);
    M(1, 1) = cos(r);
    return M;
}

Matrix4f rotationMatrix(const Vector3f &eulerDeg)
{
    return rotationZ(eulerDeg.z()) * rotationY(eulerDeg.y()) * rotationX(eulerDeg.x());
}

// ---- Unit Cube ----

// TODO: unit cube centered at origin, 8 verts, 12 tris (CCW winding)
Mesh createUnitCube()
{
    Mesh mesh;
    mesh.vertices = {
        // TODO
        Vector3f(-0.5f, -0.5f, -0.5f),
        Vector3f(0.5f, -0.5f, -0.5f),
        Vector3f(0.5f, 0.5f, -0.5f),
        Vector3f(-0.5f, 0.5f, -0.5f),
        Vector3f(-0.5f, -0.5f, 0.5f),
        Vector3f(0.5f, -0.5f, 0.5f),
        Vector3f(0.5f, 0.5f, 0.5f),
        Vector3f(-0.5f, 0.5f, 0.5f),
    };
    mesh.faces = {
        {0, 2, 1}, {0, 3, 2}, // back (-z)
        {4, 5, 6}, {4, 6, 7}, // front (+z)
        {1, 2, 6}, {1, 6, 5}, // right (+x)
        {0, 7, 3}, {0, 4, 7}, // left (-x)
        {3, 6, 2}, {3, 7, 6}, // top (+y)
        {0, 1, 5}, {0, 5, 4}  // bottom (-y)
    };

    return mesh;
}

// ---- Hierarchical Mesh Collection ----

// TODO: traverse the tree, transform each node's cube, collect into outVerts/outFaces.
// Important: pass jointWorld (T*R only, no scale) to children, not meshWorld.
void collectMeshes(const ModelNode *node,
                   const Matrix4f &parentJointWorld,
                   vector<Vector3f> &outVerts,
                   vector<Triangle> &outFaces)
{
    // TODO
    Mesh cube = createUnitCube();

    Matrix4f T = translationMatrix(node->position);
    Matrix4f R = rotationMatrix(node->rotation);
    Matrix4f S = scalingMatrix(node->scale);

    Matrix4f jointWorld = parentJointWorld * T * R;
    Matrix4f meshWorld = jointWorld * S;

    int baseIndex = (int)outVerts.size();

    for (const auto &v : cube.vertices)
    {
        Vector4f hv(v.x(), v.y(), v.z(), 1.0f);
        Vector4f tv = meshWorld * hv;
        outVerts.push_back(tv.head<3>());
    }

    for (const auto &f : cube.faces)
    {
        outFaces.push_back({f.v0 + baseIndex, f.v1 + baseIndex, f.v2 + baseIndex});
    }

    for (const auto &child : node->children)
    {
        collectMeshes(child.get(), jointWorld, outVerts, outFaces);
    }
}

// ---- OBJ Export ----

// TODO: write verts and faces to .obj (OBJ indices are 1-based!)
void exportOBJ(const string &filename,
               const vector<Vector3f> &vertices,
               const vector<Triangle> &faces)
{
    ofstream ofs(filename);
    if (!ofs.is_open())
    {
        cerr << "Error: cannot open " << filename << " for writing." << endl;
        return;
    }

    // TODO
    for (const auto &v : vertices)
    {
        ofs << "v " << v.x() << " " << v.y() << " " << v.z() << endl;
    }
    for (const auto &f : faces)
    {
        ofs << "f " << (f.v0 + 1) << " " << (f.v1 + 1) << " " << (f.v2 + 1) << "\n";
    }
    ofs.close();
    cout << "Exported: " << filename << endl;
}

// ---- Print Tree ----

void printHierarchy(const ModelNode *node, int depth = 0)
{
    string indent(depth * 2, ' ');
    cout << indent << "|- " << node->name
         << "  pos(" << node->position.transpose() << ")"
         << "  rot(" << node->rotation.transpose() << ")"
         << "  scale(" << node->scale.transpose() << ")" << endl;
    for (const auto &child : node->children)
    {
        printHierarchy(child.get(), depth + 1);
    }
}

// ---- Build Your Robot ----

// TODO: build your robot here (at least 6-8 parts)
unique_ptr<ModelNode> buildRobotModel()
{
    // Torso
    auto body = make_unique<ModelNode>(
        "WallE_Body",
        Vector3f(0.00f, 0.50f, 0.00f),
        Vector3f(0.00f, 0.00f, 0.00f),
        Vector3f(1.00f, 1.00f, 1.00f));

    // Neck
    auto neck = body->addChild(
        "Neck",
        Vector3f(0.00f, 0.55f, 0.00f),
        Vector3f(0.20f, 0.40f, 0.20f),
        Vector3f(0.00f, 0.00f, 0.00f));

    // Head
    auto headBase = neck->addChild(
        "HeadBase",
        Vector3f(0.00f, 0.25f, 0.00f),
        Vector3f(1.00f, 0.10f, 0.50f));

    // Left Eye
    headBase->addChild(
        "LeftEye",
        Vector3f(-0.30f, 0.05f, 0.10f),
        Vector3f(0.40f, 0.30f, 0.60f),
        Vector3f(0.00f, 0.00f, 0.00f));

    // Right Eye
    headBase->addChild(
        "RightEye",
        Vector3f(0.30f, 0.05f, 0.10f),
        Vector3f(0.40f, 0.30f, 0.60f),
        Vector3f(0.00f, 0.00f, 0.00f));

    // Left arm
    auto leftShoulder = body->addChild(
        "LeftShoulder",
        Vector3f(-0.55f, 0.20f, 0.00f),
        Vector3f(0.15f, 0.40f, 0.15f));
    
    leftShoulder->addChild(
        "LeftHand",
        Vector3f(0.00f, -0.25f, 0.10f),
        Vector3f(0.20f, 0.10f, 0.30f));

    // Right arm
    auto rightShoulder = body->addChild(
        "RightShoulder",
        Vector3f(0.55f, 0.20f, 0.00f),
        Vector3f(0.15f, 0.40f, 0.15f));

    rightShoulder->addChild(
        "RightHand",
        Vector3f(0.00f, -0.25f, 0.10f),
        Vector3f(0.20f, 0.10f, 0.30f));

    // Left Tread
    body->addChild(
        "LeftTread",
        Vector3f(-0.65f, -0.50f, 0.00f),
        Vector3f(0.30f, 0.50f, 1.20f));

    // Right Tread
    body->addChild(
        "RightTread",
        Vector3f(0.65f, -0.50f, 0.00f),
        Vector3f(0.30f, 0.50f, 1.20f));

    return body;
}

// ---- Main ----
int main()
{
    cout << "========================================" << endl;
    cout << " Cubism Robot" << endl;
    cout << " Hierarchical Modelling" << endl;
    cout << "========================================" << endl
         << endl;

    auto robot = buildRobotModel();

    cout << "--- Model Hierarchy ---" << endl;
    printHierarchy(robot.get());
    cout << endl;

    vector<Vector3f> allVertices;
    vector<Triangle> allFaces;
    collectMeshes(robot.get(), Matrix4f::Identity(), allVertices, allFaces);

    exportOBJ("../model/cubism_robot.obj", allVertices, allFaces);

    int V = (int)allVertices.size();
    int F = (int)allFaces.size();
    int E = F * 3 / 2;

    cout << endl;
    cout << "--- Model Statistics ---" << endl;
    cout << "Total vertices  (V): " << V << endl;
    cout << "Total edges     (E): " << E << endl;
    cout << "Total faces     (F): " << F << endl;
    cout << "Euler: V - E + F = " << (V - E + F) << endl;

    cout << endl
         << "Done. Open model/cubism_robot.obj in MeshLab to visualize." << endl;
    return 0;
}
