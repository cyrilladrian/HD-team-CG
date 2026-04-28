#include <GL/glut.h>
#include <GL/glu.h>
#include <cstdio>
#include <cmath>
#include <string>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <cstring>

using namespace Eigen;

struct ModelNode {
    std::string name;
    Vector3f position;
    Vector3f rotation;   // Euler degrees (X, Y, Z applied Z→Y→X)
    Vector3f scale;
    std::vector<std::unique_ptr<ModelNode>> children;

    ModelNode(const std::string& n,
              const Vector3f& pos = Vector3f::Zero(),
              const Vector3f& rot = Vector3f::Zero(),
              const Vector3f& scl = Vector3f::Ones())
        : name(n), position(pos), rotation(rot), scale(scl) {}

    ModelNode* addChild(const std::string& n, const Vector3f& pos,
                        const Vector3f& scl,
                        const Vector3f& rot = Vector3f::Zero()) {
        children.push_back(std::make_unique<ModelNode>(n, pos, rot, scl));
        return children.back().get();
    }
};

struct RobotPose {
    std::unique_ptr<ModelNode> root;
    ModelNode* pelvis        = nullptr;
    ModelNode* torso         = nullptr;
    ModelNode* leftUpperArm  = nullptr;
    ModelNode* rightUpperArm = nullptr;
    ModelNode* leftThigh     = nullptr;
    ModelNode* leftShin      = nullptr;
    ModelNode* leftFoot      = nullptr;
    ModelNode* rightThigh    = nullptr;
    ModelNode* rightShin     = nullptr;
    ModelNode* rightFoot     = nullptr;
    ModelNode* antenna       = nullptr;
    ModelNode* heart         = nullptr;
};

RobotPose buildRobot() {
    RobotPose p;

    p.root = std::make_unique<ModelNode>(
        "Pelvis",
        Vector3f(0.0f, 0.80f, 0.0f),
        Vector3f::Zero(),
        Vector3f(0.60f, 0.30f, 0.40f));
    p.pelvis = p.root.get();

    auto torso = p.pelvis->addChild(
        "Torso",
        Vector3f(0.0f, 0.45f, 0.0f),
        Vector3f(0.70f, 0.80f, 0.45f));
    p.torso = torso;

    // Heart emblem: invisible pivot + three cubes that read as a heart shape
    auto heart = torso->addChild(
        "HeartPivot",
        Vector3f(0.0f, 0.08f, 0.26f),
        Vector3f(0.01f, 0.01f, 0.01f));  // near-zero so the pivot box is invisible
    p.heart = heart;
    heart->addChild("HeartLobeL",
        Vector3f(-0.04f,  0.03f, 0.0f),
        Vector3f(0.07f, 0.07f, 0.04f));
    heart->addChild("HeartLobeR",
        Vector3f( 0.04f,  0.03f, 0.0f),
        Vector3f(0.07f, 0.07f, 0.04f));
    heart->addChild("HeartTip",
        Vector3f( 0.0f,  -0.04f, 0.0f),
        Vector3f(0.06f,  0.06f, 0.04f),
        Vector3f(0.0f,   0.0f,  45.0f));

        auto neck = torso->addChild(
            "Neck",
            Vector3f(0.0f, 0.50f, 0.0f),
            Vector3f(0.18f, 0.25f, 0.18f));

            auto head = neck->addChild(
                "Head",
                Vector3f(0.0f, 0.22f, 0.0f),
                Vector3f(0.50f, 0.45f, 0.50f));

                head->addChild("LeftEye",
                    Vector3f(-0.12f, 0.05f, 0.26f),
                    Vector3f(0.18f, 0.18f, 0.05f));
                head->addChild("RightEye",
                    Vector3f( 0.12f, 0.05f, 0.26f),
                    Vector3f(0.18f, 0.18f, 0.05f));

                auto antenna = head->addChild(
                    "AntennaStick",
                    Vector3f(0.0f, 0.30f, 0.0f),
                    Vector3f(0.06f, 0.32f, 0.06f));
                p.antenna = antenna;
                antenna->addChild(
                    "AntennaBall",
                    Vector3f(0.0f, 0.22f, 0.0f),
                    Vector3f(0.16f, 0.16f, 0.16f));

        auto leftArm = torso->addChild(
            "LeftUpperArm",
            Vector3f(-0.48f, 0.20f, 0.0f),
            Vector3f(0.15f, 0.38f, 0.15f));
        p.leftUpperArm = leftArm;
        leftArm->addChild(
            "LeftForearm",
            Vector3f(0.0f, -0.28f, 0.0f),
            Vector3f(0.12f, 0.30f, 0.12f));

        auto rightArm = torso->addChild(
            "RightUpperArm",
            Vector3f( 0.48f, 0.20f, 0.0f),
            Vector3f(0.15f, 0.38f, 0.15f));
        p.rightUpperArm = rightArm;
        rightArm->addChild(
            "RightForearm",
            Vector3f(0.0f, -0.28f, 0.0f),
            Vector3f(0.12f, 0.30f, 0.12f));

    auto leftThigh = p.pelvis->addChild(
        "LeftThigh",
        Vector3f(-0.22f, -0.22f, 0.0f),
        Vector3f(0.20f, 0.40f, 0.20f));
    p.leftThigh = leftThigh;

    auto leftShin = leftThigh->addChild(
        "LeftShin",
        Vector3f(0.0f, -0.28f, 0.0f),
        Vector3f(0.17f, 0.35f, 0.17f));
    p.leftShin = leftShin;

    auto leftFoot = leftShin->addChild(
        "LeftFoot",
        Vector3f(0.0f, -0.22f, 0.08f),
        Vector3f(0.24f, 0.09f, 0.36f));
    p.leftFoot = leftFoot;

    auto rightThigh = p.pelvis->addChild(
        "RightThigh",
        Vector3f( 0.22f, -0.22f, 0.0f),
        Vector3f(0.20f, 0.40f, 0.20f));
    p.rightThigh = rightThigh;

    auto rightShin = rightThigh->addChild(
        "RightShin",
        Vector3f(0.0f, -0.28f, 0.0f),
        Vector3f(0.17f, 0.35f, 0.17f));
    p.rightShin = rightShin;

    auto rightFoot = rightShin->addChild(
        "RightFoot",
        Vector3f(0.0f, -0.22f, 0.08f),
        Vector3f(0.24f, 0.09f, 0.36f));
    p.rightFoot = rightFoot;

    return p;
}

void setNodeColor(const std::string& name) {
    if (name.find("Heart") != std::string::npos) {
        GLfloat mat[] = { 0.90f, 0.15f, 0.18f, 1.0f };
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat);
        return;
    }
    if (name == "AntennaBall") {
        GLfloat mat[] = { 1.00f, 0.82f, 0.10f, 1.0f };
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat);
        return;
    }
    if (name == "AntennaStick") {
        GLfloat mat[] = { 0.75f, 0.75f, 0.82f, 1.0f };
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat);
        return;
    }
    unsigned h = 0;
    for (char c : name) h = h * 31 + (unsigned char)c;
    float r = 0.4f + 0.5f * ((h & 0xFF)       / 255.0f);
    float g = 0.4f + 0.5f * ((h >> 8 & 0xFF)  / 255.0f);
    float b = 0.4f + 0.5f * ((h >> 16 & 0xFF) / 255.0f);
    GLfloat mat[] = { r, g, b, 1.0f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat);
}

void renderNode(const ModelNode* node) {
    glPushMatrix();

    glTranslatef(node->position.x(), node->position.y(), node->position.z());
    glRotatef(node->rotation.z(), 0, 0, 1);
    glRotatef(node->rotation.y(), 0, 1, 0);
    glRotatef(node->rotation.x(), 1, 0, 0);

    glPushMatrix();
        glScalef(node->scale.x(), node->scale.y(), node->scale.z());
        setNodeColor(node->name);
        glutSolidCube(1.0);
    glPopMatrix();

    for (const auto& child : node->children)
        renderNode(child.get());

    glPopMatrix();
}

static const int   WIN_W    = 1280;
static const int   WIN_H    = 720;
static const float FPS      = 30.0f;
static const float DURATION = 15.0f;  // seconds of animation

static const float STRIDE_PERIOD  = 1.0f;
static const float HIP_SWING      = 22.0f;
static const float KNEE_BEND      = 18.0f;
static const float ANKLE_COMP     = 10.0f;
static const float BODY_BOB       = 0.04f;
static const float BODY_ROCK      = 4.0f;
static const float ARM_SWING      = 28.0f;
static const float WALK_SPEED     = 0.6f;

static int   g_frame   = 0;
static float g_time    = 0.0f;
static bool  g_capture = false;  // set true via argv to export PPM frames
static RobotPose g_robot;

void updatePose(RobotPose& p, float t) {
    const float PI = 3.14159265f;
    float phase = 2.0f * PI * t / STRIDE_PERIOD;

    p.pelvis->position.z() = WALK_SPEED * t;
    p.pelvis->position.y() = 0.80f + BODY_BOB * std::abs(std::sin(phase));

    p.torso->rotation.z() = BODY_ROCK * std::sin(2.0f * phase);

    p.leftThigh->rotation.x()  =  HIP_SWING * std::sin(phase);
    p.leftShin->rotation.x()   =  KNEE_BEND * std::max(0.0f, std::sin(phase + PI / 4.0f));
    p.leftFoot->rotation.x()   = -ANKLE_COMP * std::sin(phase);

    p.rightThigh->rotation.x() =  HIP_SWING * std::sin(phase + PI);
    p.rightShin->rotation.x()  =  KNEE_BEND * std::max(0.0f, std::sin(phase + PI + PI / 4.0f));
    p.rightFoot->rotation.x()  = -ANKLE_COMP * std::sin(phase + PI);

    p.leftUpperArm->rotation.x()  = -ARM_SWING * std::sin(phase);
    p.rightUpperArm->rotation.x() = -ARM_SWING * std::sin(phase + PI);

    // Spinning antenna: 255 deg/s → ~1.4 s per full revolution
    p.antenna->rotation.y() = std::fmod(255.0f * t, 360.0f);

    // Bouncing heart: 1.8 Hz heartbeat — position bounce + XY scale pulse
    static const float HEART_FREQ = 1.8f;
    float hb = std::abs(std::sin(2.0f * PI * HEART_FREQ * t));
    p.heart->position.y() = 0.08f + 0.010f * hb;
}

void display() {
    updatePose(g_robot, g_time);

    glClearColor(0.15f, 0.15f, 0.20f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)WIN_W / WIN_H, 0.1, 100.0);

    // Camera: side view, robot walks along +Z
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    float pz = g_robot.pelvis ? g_robot.pelvis->position.z() : 0.0f;
    gluLookAt(3.5, 2.5, pz + 5.0,   // eye: to the right and behind
              0.0, 1.2, pz,           // look at robot's hip height
              0.0, 1.0, 0.0);

    // Simple lighting
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat lightPos[] = { 3.0f, 6.0f, 5.0f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    GLfloat white[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
    GLfloat ambient[] = { 0.3f, 0.3f, 0.3f, 1.0f };
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);

    // Draw ground plane (grey)
    glDisable(GL_LIGHTING);
    glColor3f(0.35f, 0.35f, 0.35f);
    glBegin(GL_QUADS);
        glVertex3f(-5, 0, -2);
        glVertex3f(-5, 0, 50);
        glVertex3f( 5, 0, 50);
        glVertex3f( 5, 0, -2);
    glEnd();
    glEnable(GL_LIGHTING);

    if (g_robot.root) renderNode(g_robot.root.get());

    glutSwapBuffers();
}

void timer(int) {
    g_time  = g_frame / FPS;
    g_frame++;
    glutPostRedisplay();
    if (g_time < DURATION)
        glutTimerFunc((unsigned)(1000.0f / FPS), timer, 0);
}

int main(int argc, char** argv) {
    g_capture = (argc > 1 && std::string(argv[1]) == "--capture");

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(WIN_W, WIN_H);
    glutCreateWindow("Walking Robot");

    glEnable(GL_DEPTH_TEST);

    g_robot = buildRobot();

    glutDisplayFunc(display);
    glutTimerFunc(0, timer, 0);
    glutMainLoop();
    return 0;
}
