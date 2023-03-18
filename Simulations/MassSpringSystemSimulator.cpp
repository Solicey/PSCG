#include "MassSpringSystemSimulator.h"


MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
	myTimer.get();
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Demo 1.1(Euler-OneStep),Demo 1.2(Midpoint-OneStep),Demo 2(Euler-Simple),Demo 3(Midpoint-Simple),Demo 4.1 (Euler-ComplexSystem),Demo 4.2 (Midpoint-ComplexSystem)";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:
		break;
	case 1:
		break;
	case 2:
		TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=1 step=1");
		TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.1 step=0.1");
		break;
	case 3:
		TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=1 step=1");
		TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.1 step=0.1");
		break;
	case 4:
		TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=1 step=1");
		TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.1 step=0.1");
		TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "min=0.1 step=0.1");
		break;
	case 5:
		TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=1 step=1");
		TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.1 step=0.1");
		TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_FLOAT, &m_fGravity, "min=0.1 step=0.1");
		break;
	default:break;
	}
}

void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

	std::cout << "===================Reset Scene===================" << std::endl;
	// notifyCaseChanged(m_iTestCase);
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0:
		break;
	case 1:
		break;
	case 2:
		drawMassPoints();
		drawSprings();
		break;
	case 3:
		drawMassPoints();
		drawSprings();
		break;
	case 4:
		drawMassPoints();
		drawSprings();
		break;
	case 5:
		drawMassPoints();
		drawSprings();
		break;
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		clearMassSpringSystem();
		initSimpleSystem();
		setMass(10);
		setStiffness(40);
		setDampingFactor(0);
		setIntegrator(Euler);

		cout << "======================Demo 1.1 Euler-Onestep======================\n";
		useIntegrator(0.1, true);
		break;
	case 1:
		clearMassSpringSystem();
		initSimpleSystem();
		setMass(10);
		setStiffness(40);
		setDampingFactor(0);
		setIntegrator(Midpoint);

		cout << "======================Demo 1.2 Midpoint-Onestep======================\n";
		useIntegrator(0.1, true);
		break;
	case 2:
		clearMassSpringSystem();
		initSimpleSystem();
		setMass(10);
		setStiffness(40);
		setDampingFactor(0.2);
		setIntegrator(Euler);

		cout << "======================Demo 2 Euler-SimpleSystem======================\n";
		break;
	case 3:
		clearMassSpringSystem();
		initSimpleSystem();
		setMass(10);
		setStiffness(40);
		setDampingFactor(0.2);
		setIntegrator(Midpoint);

		cout << "======================Demo 3 Midpoint-SimpleSystem======================\n";
		break;
	case 4:
		clearMassSpringSystem();
		initComplexSystem();
		setMass(10);
		setStiffness(60);
		setDampingFactor(0.5);
		setIntegrator(Euler);
		setGravity(0.15);
		cout << "======================Demo 4.1 Euler-ComplexSystem======================\n";
		break;
	case 5:
		clearMassSpringSystem();
		initComplexSystem();
		setMass(10);
		setStiffness(60);
		setDampingFactor(0.5);
		setIntegrator(Midpoint);
		setGravity(0.15);
		cout << "======================Demo 4.2 Midpoint-ComplexSystem======================\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iTestCase)
	{// handling different cases
	case 0:
		break;
	case 1:
		break;
	case 2:
		useIntegrator(timeStep);
		break;
	case 3:
		useIntegrator(timeStep);
		break;
	case 4:
		useIntegrator(timeStep);
		break;
	case 5:
		useIntegrator(timeStep);
		break;
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::setMass(float mass)
{
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

void MassSpringSystemSimulator::setGravity(float gravity)
{
	m_fGravity = gravity;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	int id = m_massPoints.size();
	m_massPoints.push_back({ position, Velocity, isFixed });
	// std::cout << "current id: " << id << std::endl;
	return id;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	m_springs.push_back({ masspoint1, masspoint2, initialLength });
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return m_massPoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	if (index < 0 || index >= m_massPoints.size())
		return Vec3();

	return m_massPoints[index].position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	if (index < 0 || index >= m_massPoints.size() || m_massPoints[index].isFixed)
		return Vec3(0, 0, 0);

	return m_massPoints[index].velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
}

void MassSpringSystemSimulator::initSimpleSystem()
{
	int p1 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	int p2 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
	addSpring(p1, p2, 1);
}

void MassSpringSystemSimulator::initComplexSystem()
{
	int p1 = addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), false);
	int p2 = addMassPoint(Vec3(m_fCubeEdge, 0, 0), Vec3(0, 0, 0), false);
	int p3 = addMassPoint(Vec3(m_fCubeEdge, 0, m_fCubeEdge), Vec3(0, 0, 0), false);
	int p4 = addMassPoint(Vec3(0, 0, m_fCubeEdge), Vec3(0, 0, 0), false);
	int p5 = addMassPoint(Vec3(0, m_fCubeEdge, 0), Vec3(0, 0, 0), false);
	int p6 = addMassPoint(Vec3(m_fCubeEdge, m_fCubeEdge, 0), Vec3(0, 0, 0), false);
	int p7 = addMassPoint(Vec3(m_fCubeEdge, m_fCubeEdge, m_fCubeEdge), Vec3(0, 0, 0), false);
	int p8 = addMassPoint(Vec3(0, m_fCubeEdge, m_fCubeEdge), Vec3(0, 0, 0), false);

	int p9 = addMassPoint(Vec3(m_fCubeEdge / 2.0, 1, m_fCubeEdge / 2.0), Vec3(0, 0, 0), false);
	int p10 = addMassPoint(Vec3(-m_fCubeEdge, 1, -m_fCubeEdge), Vec3(0, 0, 0), true);
	addSpring(p9, p10, m_fCubeEdge * 2);
	addSpring(p9, p5, m_fCubeEdgeInit * 2);
	addSpring(p9, p6, m_fCubeEdgeInit * 2);
	addSpring(p9, p7, m_fCubeEdgeInit * 2);
	addSpring(p9, p8, m_fCubeEdgeInit * 2);

	addSpring(p1, p2, m_fCubeEdgeInit);
	addSpring(p2, p3, m_fCubeEdgeInit);
	addSpring(p3, p4, m_fCubeEdgeInit);
	addSpring(p4, p1, m_fCubeEdgeInit);
	addSpring(p5, p6, m_fCubeEdgeInit);
	addSpring(p6, p7, m_fCubeEdgeInit);
	addSpring(p7, p8, m_fCubeEdgeInit);
	addSpring(p8, p5, m_fCubeEdgeInit);
	addSpring(p1, p5, m_fCubeEdgeInit);
	addSpring(p2, p6, m_fCubeEdgeInit);
	addSpring(p3, p7, m_fCubeEdgeInit);
	addSpring(p4, p8, m_fCubeEdgeInit);

	const float sqrt2 = 1.414213;
	addSpring(p2, p5, m_fCubeEdgeInit * sqrt2);
	addSpring(p3, p6, m_fCubeEdgeInit * sqrt2);
	addSpring(p4, p7, m_fCubeEdgeInit * sqrt2);
	addSpring(p1, p8, m_fCubeEdgeInit * sqrt2);
	addSpring(p6, p8, m_fCubeEdgeInit * sqrt2);
	addSpring(p2, p4, m_fCubeEdgeInit * sqrt2);

	addSpring(p1, p6, m_fCubeEdgeInit * sqrt2);
	addSpring(p2, p7, m_fCubeEdgeInit * sqrt2);
	addSpring(p3, p8, m_fCubeEdgeInit * sqrt2);
	addSpring(p4, p5, m_fCubeEdgeInit * sqrt2);
	addSpring(p5, p7, m_fCubeEdgeInit * sqrt2);
	addSpring(p1, p3, m_fCubeEdgeInit * sqrt2);
}

void MassSpringSystemSimulator::drawMassPoints()
{
	for (const auto& p : m_massPoints)
	{
		//std::cout << "Draw mass point: " << p.position << std::endl;
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
		DUC->drawSphere(p.position, Vec3(m_fMassPointSize, m_fMassPointSize, m_fMassPointSize));
	}
}

void MassSpringSystemSimulator::drawSprings()
{
	for (const auto& s : m_springs)
	{
		DUC->beginLine();
		DUC->drawLine(getPositionOfMassPoint(s.p1), 0.4 * Vec3(0.97, 0.86, 1),
			getPositionOfMassPoint(s.p2), 0.4 * Vec3(0.97, 0.86, 1));
		DUC->endLine();
	}
}

void MassSpringSystemSimulator::clearMassSpringSystem()
{
	m_massPoints.clear();
	m_springs.clear();
}

void MassSpringSystemSimulator::useIntegrator(float timeStep, bool printDebugInfo)
{
	m_accumFrameTime += myTimer.update().time / 1000.0f;
	// std::cout << m_accumFrameTime << " " << timeStep << std::endl;
	if (m_accumFrameTime < timeStep) return;

	int count = (int)(m_accumFrameTime / timeStep);
	for (int i = 0; i < count; i++)
	{
		std::vector<Vec3> forces;
		forces.resize(m_massPoints.size(), Vec3(0, 0, 0));

		if (m_iIntegrator == Euler)
		{
			for (const auto& s : m_springs)
			{
				Vec3 x12 = getPositionOfMassPoint(s.p2) - getPositionOfMassPoint(s.p1);
				float length = norm(x12);
				Vec3 norm_x12 = x12 / length;
				Vec3 fs1 = m_fStiffness * (length - s.initLength) * norm_x12;

				Vec3 v12 = getVelocityOfMassPoint(s.p2) - getVelocityOfMassPoint(s.p1);
				Vec3 fd1 = m_fDamping * dot(v12, norm_x12) * norm_x12;

				forces[s.p1] += fs1 + fd1;
				forces[s.p2] -= fs1 + fd1;

				/*std::cout << "x12: " << x12 << std::endl;
				std::cout << "norm_x12: " <<  norm_x12 << std::endl;
				std::cout << "length: " << length << std::endl;

				std::cout << "fs1: " << fs1 << std::endl;
				std::cout << "fd1: " << fd1 << std::endl;*/
			}

			for (int id = 0; id < m_massPoints.size(); id++)
			{
				auto& p = m_massPoints[id];

				// extern gravity
				forces[id] += Vec3(0, -m_fGravity * m_fMass, 0);

				if (p.isFixed)
				{
					p.velocity = 0;
					continue;
				}

				p.position = p.position + p.velocity * timeStep;
				p.velocity = p.velocity + forces[id] * timeStep / m_fMass;

				if (p.position.y <= m_fGroundY)
				{
					p.position.y = m_fGroundY;
					if (p.velocity.y < 0)
						p.velocity.y = -p.velocity.y * m_fGroundContactLoss;
				}

				if (printDebugInfo)
				{
					std::cout << "Euler Method... id: " << id << " force: " << forces[id] << " velocity: " << p.velocity << " position: " << p.position << std::endl;
					return;
				}
			}
		}
		else if (m_iIntegrator == Midpoint)
		{
			for (const auto& s : m_springs)
			{
				Vec3 x12 = getPositionOfMassPoint(s.p2) - getPositionOfMassPoint(s.p1);
				float length = norm(x12);
				Vec3 norm_x12 = x12 / length;
				Vec3 fs1 = m_fStiffness * (length - s.initLength) * norm_x12;
				Vec3 v12 = getVelocityOfMassPoint(s.p2) - getVelocityOfMassPoint(s.p1);
				Vec3 fd1 = m_fDamping * dot(v12, norm_x12) * norm_x12;

				forces[s.p1] += fs1 + fd1;
				forces[s.p2] -= fs1 + fd1;

				// std::cout << "fs1: " << fs1 << std::endl;
			}

			std::vector<MassPoint> mids;
			mids.resize(m_massPoints.size());

			for (int id = 0; id < m_massPoints.size(); id++)
			{
				const auto& p = m_massPoints[id];
				auto& mid = mids[id];

				mid.position = p.position + p.velocity * timeStep / 2.0f;
				mid.velocity = p.velocity + forces[id] * timeStep / (2.0f * m_fMass);

				forces[id] = Vec3(0, 0, 0);
			}

			for (const auto& s : m_springs)
			{
				int p1 = s.p1, p2 = s.p2;

				Vec3 x12 = mids[p2].position - mids[p1].position;
				float length = norm(x12);
				Vec3 norm_x12 = x12 / length;
				Vec3 fs1 = m_fStiffness * (length - s.initLength) * norm_x12;
				Vec3 v12 = getVelocityOfMassPoint(p2) - getVelocityOfMassPoint(p1);
				Vec3 fd1 = m_fDamping * dot(v12, norm_x12) * norm_x12;

				forces[p1] += fs1 + fd1;
				forces[p2] -= fs1 + fd1;

				// std::cout << "fs1: " << fs1 << std::endl;
			}

			for (int id = 0; id < m_massPoints.size(); id++)
			{
				// extern gravity
				forces[id] += Vec3(0, -m_fGravity * m_fMass, 0);

				auto& p = m_massPoints[id];

				if (p.isFixed)
				{
					p.velocity = 0;
					continue;
				}

				p.position = p.position + p.velocity * timeStep;
				p.velocity = p.velocity + forces[id] * timeStep / m_fMass;

				if (p.position.y <= m_fGroundY)
				{
					p.position.y = m_fGroundY;
					if (p.velocity.y < 0)
						p.velocity.y = -p.velocity.y * m_fGroundContactLoss;
				}

				if (printDebugInfo)
				{
					std::cout << "Midpoint Method... id: " << id << " force: " << forces[id] << " velocity: " << p.velocity << " position: " << p.position << std::endl;
					return;
				}

			}
		}
		else
		{

		}
	}

	m_accumFrameTime -= count * timeStep;
}
