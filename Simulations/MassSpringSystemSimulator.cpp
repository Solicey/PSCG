#include "MassSpringSystemSimulator.h"


MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_iTestCase = 0;
	myTimer.get();
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Demo 1,Demo 2,Demo 3";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:
		break;
	case 1:
		TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=1 step=1");
		TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.1 step=0.1");
		break;
	case 2:
		TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=1 step=1");
		TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.1 step=0.1");
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
		drawMassPoints();
		drawSprings();
		break;
	case 2:
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

		cout << "======================Demo 1======================\n";
		useIntegrator(0.1);
		break;
	case 1:
		clearMassSpringSystem();
		initSimpleSystem();
		setMass(10);
		setStiffness(40);
		setDampingFactor(0.1);
		setIntegrator(Euler);

		cout << "======================Demo 2======================\n";
		break;
	case 2:
		clearMassSpringSystem();
		initSimpleSystem();
		setMass(10);
		setStiffness(40);
		setDampingFactor(0.1);
		setIntegrator(Midpoint);

		cout << "======================Demo 3======================\n";
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
		useIntegrator(timeStep);
		break;
	case 2:
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

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	int id = m_massPoints.size();
	m_massPoints.push_back({ position, Velocity, isFixed });
	std::cout << "current id: " << id << std::endl;
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

void MassSpringSystemSimulator::useIntegrator(float timeStep)
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

				if (p.isFixed)
				{
					p.velocity = 0;
					continue;
				}

				p.position = p.position + p.velocity * timeStep;
				p.velocity = p.velocity + forces[id] * timeStep / m_fMass;

				// std::cout << "id: " << id << " force: " << forces[id] << " velocity: " << p.velocity << " position: " << p.position << std::endl;
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

				forces[s.p1] += fs1;
				forces[s.p2] -= fs1;

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

				forces[p1] += fs1;
				forces[p2] -= fs1;

				// std::cout << "fs1: " << fs1 << std::endl;
			}

			for (int id = 0; id < m_massPoints.size(); id++)
			{
				auto& p = m_massPoints[id];

				p.position = p.position + p.velocity * timeStep;
				p.velocity = p.velocity + forces[id] * timeStep / m_fMass;

				// std::cout << "id: " << id << " force: " << forces[id] << " velocity: " << p.velocity << " position: " << p.position << std::endl;

			}
		}
		else
		{

		}
	}

	m_accumFrameTime -= count * timeStep;
}
