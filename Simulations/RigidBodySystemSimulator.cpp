#include "RigidBodySystemSimulator.h"

#include "collisionDetect.h"

#define GRAVITATIONAL_CONSTANT 0.01

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
    m_iTestCase = 0;
    myTimer.get();
    m_pRigidBodySystem = new RigidBodySystem();
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
    return "Demo 1(A simple one-step test),Demo 2(Simple single body simulation),Demo 3(Two-rigid-body collision scene),Demo 4(Complex simulation)";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;
    switch (m_iTestCase)
    {
    case 0:
        break;
    case 1:
        // TwAddVarRW(DUC->g_pTweakBar, "box rotation", TW_TYPE_QUAT4F, &m_pRigidBodySystem->Boxes[0].Orientation, "label='Object rotation' opened=true help='Change the object orientation.' axisz=-z");
        break;
    case 2:
        TwAddVarRW(DUC->g_pTweakBar, "lower box restitution", TW_TYPE_FLOAT, &m_pRigidBodySystem->Boxes[0].Restitution, "min=0 max=1 step=0.01");
        TwAddVarRW(DUC->g_pTweakBar, "upper box restitution", TW_TYPE_FLOAT, &m_pRigidBodySystem->Boxes[1].Restitution, "min=0 max=1 step=0.01");
        break;
    case 3:
        break;
    }
}

void RigidBodySystemSimulator::reset()
{
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

    std::cout << "===================Reset Scene===================" << std::endl;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    switch (m_iTestCase)
    {
    case 0:
        DrawRigidBodySystem();
        break;
    case 1:
        DrawRigidBodySystem();
        break;
    case 2:
        DrawRigidBodySystem();
        break;
    case 3:
        DrawRigidBodySystem();
        break;
    }
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
    m_iTestCase = testCase;
    switch (m_iTestCase)
    {
    case 0:
        cout << "======================Demo 1: A simple one-step test======================\n";
        m_pRigidBodySystem->InitSystemWithOneBox();
        {
            const auto& box = m_pRigidBodySystem->Boxes[0];
            Vec3 pos = box.Position;
            std::cout << "center position: " << box.Position << std::endl;
            SimulateRigidMotion(2.0);
            std::cout << "linear velocity: " << box.Velocity << std::endl;
            std::cout << "angular velocity: " << box.AngularVelocity << std::endl;
            std::cout << "world space velocity: " << box.Velocity + cross(box.AngularVelocity, (Vec3(-0.3, -0.5, -0.25) - pos)) << std::endl;
        }
        break;
    case 1:
        cout << "======================Demo 2: Simple single body simulation======================\n";
        m_pRigidBodySystem->InitSystemWithOneBox();
        break;
    case 2:
        cout << "======================Demo 3: Two-rigid-body collision scene======================\n";
        m_pRigidBodySystem->InitSystemWithTwoBoxes();
        break;
    case 3:
        cout << "======================Demo 4: Complex simulation======================\n";
        m_pRigidBodySystem->InitSystemWithFourBoxes();
        break;
    }
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
    m_accumFrameTime += myTimer.update().time / 1000.0f;
    // std::cout << m_accumFrameTime << " " << timeStep << std::endl;
    if (m_accumFrameTime < timeStep) return;
    int count = (int)(m_accumFrameTime / timeStep);

    switch (m_iTestCase)
    {
    case 0:
        // Apply force
        //SimulateRigidMotion(timeStep);
        break;
    case 1:
        SimulateRigidMotion(timeStep);
        break;
    case 2:
        UpdateCollisionForce(timeStep);
        SimulateRigidMotion(timeStep);
        break;
    case 3:
        UpdateCollisionForce(timeStep);
        SimulateRigidMotion(timeStep);
        break;
    }

    m_accumFrameTime -= count * timeStep;
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
    m_oldtrackmouse.x = x;
    m_oldtrackmouse.y = y;
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
    return m_pRigidBodySystem->Boxes.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
    if (m_pRigidBodySystem->Boxes.size() > i && i >= 0)
        return m_pRigidBodySystem->Boxes[i].Position;
    return Vec3();
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
    if (m_pRigidBodySystem->Boxes.size() > i && i >= 0)
        return m_pRigidBodySystem->Boxes[i].Velocity;
    return Vec3();
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
    if (m_pRigidBodySystem->Boxes.size() > i && i >= 0)
        return m_pRigidBodySystem->Boxes[i].AngularVelocity;
    return Vec3();
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
    if (m_pRigidBodySystem->Boxes.size() > i && i >= 0)
    {
        m_pRigidBodySystem->Boxes[i].ExternalForces.push_back({ loc, force });
    }
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
    RigidBox box(position, size, Quat(), Vec3(), Vec3(), mass);
    m_pRigidBodySystem->Boxes.push_back(box);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
    if (m_pRigidBodySystem->Boxes.size() > i && i >= 0)
        m_pRigidBodySystem->Boxes[i].Orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
    if (m_pRigidBodySystem->Boxes.size() > i && i >= 0)
        m_pRigidBodySystem->Boxes[i].Velocity = velocity;
}

void RigidBodySystemSimulator::DrawRigidBodySystem()
{
    DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
    
    for (auto& box : m_pRigidBodySystem->Boxes)
    {
        Mat4 translate, rotation, scale;
        translate.initTranslation(box.Position.x, box.Position.y, box.Position.z);
        rotation = XMMatrixRotationQuaternion(XMQuaternionNormalize(box.Orientation.toDirectXQuat()));
        scale.initScaling(box.Size.x, box.Size.y, box.Size.z);

        DUC->drawRigidBody(scale * rotation * translate);
    }
}

void RigidBodySystemSimulator::SimulateRigidMotion(float ts)
{
    for (auto& box : m_pRigidBodySystem->Boxes)
    {
        Force joinForce;
        joinForce.Position = box.Position;

        if (box.IsGround)
            continue;

        if (box.EnableGravity && !box.IsOnGround)
        {
            joinForce.Direction += Vec3(0, box.Mass * -GRAVITATIONAL_CONSTANT, 0);
        }

        Mat4 rotationMatrix = box.Orientation.getRotMat();
        Mat4 inverseRotationMatrix = rotationMatrix.inverse();
        
        Mat4 rotationMatrixTranspose = rotationMatrix;
        rotationMatrixTranspose.transpose();

        Mat4 I = rotationMatrixTranspose * box.Iref * rotationMatrix;
        Mat4 invI = I.inverse();
            
        Vec3 torque = Vec3();
        for (auto& force : box.ExternalForces)
        {
            Vec3 worldRi = force.Position - box.Position;
            Vec3 localRi = inverseRotationMatrix.transformVector(worldRi);
            if (abs(localRi.x) > box.Size.x || abs(localRi.y) > box.Size.y || abs(localRi.z) > box.Size.z)
                continue;

            joinForce.Direction += force.Direction;
            // std::cout << "localRi: " << localRi.x << ", " << localRi.y << ", " << localRi.z << std::endl;
            torque += cross(worldRi, force.Direction);
        }
        for (auto& force : box.CollisionForces)
        {
            Vec3 worldRi = force.Position - box.Position;
            Vec3 localRi = inverseRotationMatrix.transformVector(worldRi);
            if (abs(localRi.x) > box.Size.x || abs(localRi.y) > box.Size.y || abs(localRi.z) > box.Size.z)
                continue;

            joinForce.Direction += force.Direction;
            // std::cout << "localRi: " << localRi.x << ", " << localRi.y << ", " << localRi.z << std::endl;
            torque += cross(worldRi, force.Direction);
        }

        // Update velocity and position
        box.Velocity += ts * joinForce.Direction / box.Mass;
        box.Position += ts * box.Velocity;

        // Update angularVelocity and orientation
        box.AngularVelocity = box.AngularVelocity + ts * invI.transformVector(torque);
        box.Orientation = box.Orientation + ts * 0.5 * Quat(box.AngularVelocity.x, box.AngularVelocity.y, box.AngularVelocity.z, 0) * box.Orientation;

        box.Orientation = box.Orientation.unit();
    }
}

void RigidBodySystemSimulator::UpdateCollisionForce(float ts)
{
    std::vector<Mat4> mats;
    std::vector<Mat4> invIs;

    for (int i = 0; i < m_pRigidBodySystem->Boxes.size(); i++)
    {
        const auto& box = m_pRigidBodySystem->Boxes[i];
        Mat4 translate, rotation, scale;
        Quat orient = box.Orientation;
        translate.initTranslation(box.Position.x, box.Position.y, box.Position.z);
        rotation = XMMatrixRotationQuaternion(XMQuaternionNormalize(orient.toDirectXQuat()));
        scale.initScaling(box.Size.x, box.Size.y, box.Size.z);
        
        mats.push_back(scale * rotation * translate);
        
        Mat4 rotaionT = rotation;
        rotaionT.transpose();
        Mat4 I = rotaionT * box.Iref * rotation;
        invIs.push_back(I.inverse());

        //
        m_pRigidBodySystem->Boxes[i].CollisionForces.clear();
        m_pRigidBodySystem->Boxes[i].IsOnGround = false;
    }

    for (int a = 0; a < m_pRigidBodySystem->Boxes.size(); a++)
    {
        for (int b = 0; b < m_pRigidBodySystem->Boxes.size(); b++)
        {
            if (a == b)
                continue;

            auto& A = m_pRigidBodySystem->Boxes[a];
            auto& B = m_pRigidBodySystem->Boxes[b];

            // whether vertex of B knock into A
            auto info = collisionTools::checkCollisionSATHelper(mats[a].toDirectXMatrix(), mats[b].toDirectXMatrix(), A.Size.toDirectXVector(), B.Size.toDirectXVector());

            if (!info.isValid)
                continue;

            // A is ground
            if (A.IsGround)
                B.IsOnGround = true;

            Vec3 velA = A.Velocity + cross(A.AngularVelocity, info.collisionPointWorld - A.Position);
            Vec3 velB = B.Velocity + cross(B.AngularVelocity, info.collisionPointWorld - B.Position);
            Vec3 n = -info.normalWorld;
            float vRel = dot(n, velB - velA);

            if (vRel > 0)
                continue;

            Vec3 cA = cross(invIs[a].transformVector(cross(info.collisionPointWorld, n)), info.collisionPointWorld);
            Vec3 cB = cross(invIs[b].transformVector(cross(info.collisionPointWorld, n)), info.collisionPointWorld);
            float J = (-vRel) / (1.0 / A.Mass + 1.0 / B.Mass + dot((cA + cB), n));

            /*B.Velocity = B.Velocity + (1 + B.Restitution) * J * n / B.Mass;
            A.Velocity = A.Velocity - (1 + A.Restitution) * J * n / A.Mass;

            B.AngularVelocity = B.AngularVelocity + invIs[b].transformVector(cross(info.collisionPointWorld, (1 + B.Restitution) * J * n));
            A.AngularVelocity = A.AngularVelocity - invIs[a].transformVector(cross(info.collisionPointWorld, (1 + A.Restitution) * J * n));*/
            A.CollisionForces.push_back({info.collisionPointWorld, -(1 + A.Restitution) * J * n});
            B.CollisionForces.push_back({info.collisionPointWorld, (1 + B.Restitution) * J * n });
        }
    }
}

void RigidBodySystem::InitSystemWithOneBox()
{
    Boxes.clear();

    RigidBox box(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), Quat(Vec3(0, 0, 1), M_PI / 2.0), Vec3(0, 0, 0), Vec3(0, 0, 0), 2);
    Force force = { Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0) };
    box.ExternalForces.push_back(force);
    
    Boxes.push_back(box);
    
}

void RigidBodySystem::InitSystemWithTwoBoxes()
{
    Boxes.clear();

    RigidBox box1(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), Quat(Vec3(0.0f, 0.0f, 1.0f), (float)(M_PI) * 0.25f), Vec3(0.0f, 0.1f, 0.05f), Vec3(0, 0, 0), 100.0f);
    RigidBox box2(Vec3(0.0f, 0.2f, 0.0f), Vec3(0.4f, 0.2f, 0.2f), Quat(0, 0, 0, 1), Vec3(0, 0, 0), Vec3(0, 0, 0), 100.0f);

    Boxes.push_back(box1);
    Boxes.push_back(box2);
}

void RigidBodySystem::InitSystemWithFourBoxes()
{
    Boxes.clear();

    RigidBox box1(Vec3(-0.1f, -0.2f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), Quat(Vec3(0.0f, 0.5f, 1.0f), (float)(M_PI) * 0.25f), Vec3(0, -0.08f, 0), Vec3(0, 0, 0), 40.0f);
    //box1.EnableGravity = true;
    box1.Restitution = 1.0f;
    RigidBox box2(Vec3(-0.1f, 0.4f, 0.1f), Vec3(0.4f, 0.2f, 0.2f), Quat(Vec3(0.0f, 0.2f, 1.0f), (float)(M_PI) * 0.5f), Vec3(0, -0.04f, 0), Vec3(0, 0, 0), 10.0f);
    //box2.EnableGravity = true;
    box2.Restitution = 1.0f;
    RigidBox box3(Vec3(-0.0f, -1.5f, 0.0f), Vec3(2.0f, 1.5f, 2.0f), Quat(0, 0, 0, 1), Vec3(0, 0, 0), Vec3(0, 0, 0), 200.0f);
    box3.IsGround = true;
    RigidBox box4(Vec3(-0.1f, 0.8f, 0.1f), Vec3(0.2f, 0.1f, 0.3f), Quat(Vec3(0.0f, 0.5f, 1.0f), (float)(M_PI) * 0.7f), Vec3(0, -0.06f, 0), Vec3(0, 0.0f, 0), 5.0f);
    //box4.EnableGravity = true;
    box4.Restitution = 1.0f;

    Boxes.push_back(box1);
    Boxes.push_back(box2);
    Boxes.push_back(box3);
    Boxes.push_back(box4);
}

RigidBox::RigidBox(const Vec3& pos, const Vec3& size, const Quat& orient, const Vec3& vel, const Vec3& ang, float mass)
    : Position(pos), Size(size), Velocity(vel), AngularVelocity(ang), Mass(mass), Orientation(orient)
{
    RigidPoints.push_back(0.5 * Vec3(size.x, size.y, size.z));
    RigidPoints.push_back(0.5 * Vec3(size.x, size.y, -size.z));
    RigidPoints.push_back(0.5 * Vec3(size.x, -size.y, size.z));
    RigidPoints.push_back(0.5 * Vec3(size.x, -size.y, -size.z));
    RigidPoints.push_back(0.5 * Vec3(-size.x, size.y, size.z));
    RigidPoints.push_back(0.5 * Vec3(-size.x, size.y, -size.z));
    RigidPoints.push_back(0.5 * Vec3(-size.x, -size.y, size.z));
    RigidPoints.push_back(0.5 * Vec3(-size.x, -size.y, -size.z));

    float x2 = size.x * size.x, y2 = size.y * size.y, z2 = size.z * size.z;
    Iref = Mat4(mass * (y2 + z2) / 12.0, 0, 0, 0, 
                0, mass * (x2 + z2) / 12.0, 0, 0,
                0, 0, mass * (x2 + y2) / 12.0, 0,
                0, 0, 0, 1);
}
