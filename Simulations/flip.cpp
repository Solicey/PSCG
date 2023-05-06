#include "flip.h"

const int followId = 137;

FlipSimulator::FlipSimulator()
{
	myTimer.get();
	m_iTestCase = 0;
}

void FlipSimulator::integrateParticles(float timeStep)
{
	//std::cout << "============integrate==============" << std::endl;
	Vec3 pos = m_particlePos[followId];
	Vec3 vel = m_particleVel[followId];
	//std::cout << "pos[0] before integrate: " << pos.x << " " << pos.y << " " << pos.z << std::endl;
	//std::cout << "vel[0] before integrate: " << vel.x << " " << vel.y << " " << vel.z << std::endl;

	for (int i = 0; i < m_iNumSpheres; i++)
	{
		m_particleVel[i].y += timeStep * -9.8;
		//m_particleVel[i].makeCeil(Vec3(-100, -100, -100));
		//m_particleVel[i].makeFloor(Vec3(100, 100, 100));
		m_particlePos[i] += timeStep * m_particleVel[i];
	}

	pos = m_particlePos[followId];
	vel = m_particleVel[followId];
	//std::cout << "pos[0] after integrate: " << pos.x << " " << pos.y << " " << pos.z << std::endl;
	//std::cout << "vel[0] after integrate: " << vel.x << " " << vel.y << " " << vel.z << std::endl;
}

void FlipSimulator::pushParticlesApart(int numIters)
{
	//std::cout << "============pushapart==============" << std::endl;
	Vec3 pos = m_particlePos[followId];
	Vec3 vel = m_particleVel[followId];
	//std::cout << "pos[0] before pushapart: " << pos.x << " " << pos.y << " " << pos.z << std::endl;
	//std::cout << "vel[0] before pushapart: " << vel.x << " " << vel.y << " " << vel.z << std::endl;

	float colorDiffusionCoeff = 0.001;

	std::vector<int> cellParticles, firstParticle, cellParticlesId;
	cellParticles.clear(); firstParticle.clear(); cellParticlesId.clear();
	cellParticles.resize(m_iNumCells, 0);
	firstParticle.resize(m_iNumCells + 1, 0);
	cellParticlesId.resize(m_iNumCells, 0);

	for (int i = 0; i < m_iNumSpheres; i++)
	{
		cellParticles[GetCellId(m_particlePos[i])]++;
	}

	int first = 0;
	for (int i = 0; i < m_iNumCells; i++)
	{
		first += cellParticles[i];
		firstParticle[i] = first;
	}
	firstParticle[m_iNumCells] = first;

	for (int i = 0; i < m_iNumSpheres; i++)
	{
		firstParticle[GetCellId(m_particlePos[i])]--;
		cellParticlesId[firstParticle[GetCellId(m_particlePos[i])]] = i;
	}

	float minDist = 2.0 * m_particleRadius;
	float minDist2 = minDist * minDist;
	for (int iter = 0; iter < numIters; iter++)
	{
		for (int i = 0; i < m_iNumSpheres; i++)
		{
			Vec3 ppos = m_particlePos[i];

			int pxi = floor(ppos.x * m_fInvSpacing);
			int pyi = floor(ppos.y * m_fInvSpacing);
			int pzi = floor(ppos.z * m_fInvSpacing);

			int x0 = max(pxi - 1, 0);
			int y0 = max(pyi - 1, 0);
			int z0 = max(pzi - 1, 0);

			int x1 = min(pxi + 1, m_iCellX - 1);
			int y1 = min(pyi + 1, m_iCellY - 1);
			int z1 = min(pzi + 1, m_iCellZ - 1);

			for (int xi = x0; xi <= x1; xi++)
			{
				for (int yi = y0; yi <= y1; yi++)
				{
					for (int zi = z0; zi <= z1; zi++)
					{
						int cellNr = GetCellId(xi, yi, zi);
						int first = firstParticle[cellNr];
						int last = firstParticle[cellNr + 1];
						for (int j = first; j < last; j++)
						{
							int id = cellParticlesId[j];
							if (id == i)
								continue;

							Vec3 qpos = m_particlePos[id];

							Vec3 delta = qpos - ppos;
							float d2 = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;

							if (d2 >= minDist2 || d2 <= 0.0f)
								continue;

							float d = sqrt(d2);
							/*if (d == 0)
							{
								delta = (i + id) % 2 == 0 ? Vec3(1, 0, 0) : Vec3(0, 0, 1);
								d = m_particleRadius;
							}
							else if (d < 0.00001) d = 0.00001;*/
							float s = 0.5 * (minDist - d) / d;
							delta *= s;
							m_particlePos[i] -= delta;
							m_particlePos[id] += delta;

							Vec3 color0 = m_particleColor[i];
							Vec3 color1 = m_particleColor[id];
							Vec3 color = (color0 + color1) * 0.5f;
							m_particleColor[i] = color0 + (color - color0) * colorDiffusionCoeff;
							m_particleColor[id] = color1 + (color - color1) * colorDiffusionCoeff;
						}
					}
				}
			}
		}
	}

	pos = m_particlePos[followId];
	vel = m_particleVel[followId];
	//std::cout << "pos[0] after pushapart: " << pos.x << " " << pos.y << " " << pos.z << std::endl;
	//std::cout << "vel[0] after pushapart: " << vel.x << " " << vel.y << " " << vel.z << std::endl;
}

void FlipSimulator::handleParticleCollisions(Vec3 obstaclePos, float obstacleRadius, Vec3 obstacleVel)
{
	//std::cout << "============collision==============" << std::endl;

	float minX = m_h + m_particleRadius;
	float maxX = (m_iCellX - 1) * m_h - m_particleRadius;

	//std::cout << "minX: " << minX << ", maxX: " << maxX << std::endl;
 
	float minY = m_h + m_particleRadius;
	float maxY = (m_iCellY - 1) * m_h - m_particleRadius;

	float minZ = m_h + m_particleRadius;
	float maxZ = (m_iCellZ - 1) * m_h - m_particleRadius;

	{
		Vec3 pos = m_particlePos[followId];
		Vec3 vel = m_particleVel[followId];
		//std::cout << "pos[0] before collision: " << pos.x << " " << pos.y << " " << pos.z << std::endl;
		//std::cout << "vel[0] before collision: " << vel.x << " " << vel.y << " " << vel.z << std::endl;
	}

	float minDist = m_obstacleRadius + m_particleRadius;
	float minDist2 = minDist * minDist;

	for (int i = 0; i < m_iNumSpheres; i++)
	{
		Vec3 pos = m_particlePos[i];

		float dis2 = pos.squaredDistanceTo(m_obstaclePos);
		if (dis2 < minDist2)
		{
			m_particleVel[i] = m_obstacleVel;
		}

		if (pos.x < minX)
		{
			pos.x = minX;
			m_particleVel[i].x = 0;
		}

		if (pos.x > maxX)
		{
			pos.x = maxX;
			m_particleVel[i].x = 0;
		}

		if (pos.y < minY)
		{
			pos.y = minY;
			m_particleVel[i].y = 0;
		}

		if (pos.y > maxY)
		{
			pos.y = maxY;
			m_particleVel[i].y = 0;
		}

		if (pos.z < minZ)
		{
			pos.z = minZ;
			m_particleVel[i].z = 0;
		}

		if (pos.z > maxZ)
		{
			pos.z = maxZ;
			m_particleVel[i].z = 0;
		}

		m_particlePos[i] = pos;
	}

	{
		Vec3 pos = m_particlePos[followId];
		Vec3 vel = m_particleVel[followId];
		//std::cout << "pos[0] after collision: " << pos.x << " " << pos.y << " " << pos.z << std::endl;
		//std::cout << "vel[0] after collision: " << vel.x << " " << vel.y << " " << vel.z << std::endl;
	}
}

void FlipSimulator::updateParticleDensity()
{
	float h2 = m_h / 2.0f; // half of grid size

	for (int i = 0; i < m_iNumCells; i++)
	{
		m_particleDensity[i] = 0;
	}

	for (int i = 0; i < m_iNumSpheres; i++)
	{
		Vec3 pos = m_particlePos[i];

		float x = min(max((float)pos.x, m_h), (m_iCellX - 1) * m_h);
		float y = min(max((float)pos.y, m_h), (m_iCellY - 1) * m_h);
		float z = min(max((float)pos.z, m_h), (m_iCellZ - 1) * m_h);

		int x0 = floor((x - h2) * m_fInvSpacing);
		float tx = ((x - h2) - x0 * m_h) * m_fInvSpacing;
		int x1 = min(x0 + 1, m_iCellX - 2);

		int y0 = floor((y - h2) * m_fInvSpacing);
		float ty = ((y - h2) - y0 * m_h) * m_fInvSpacing;
		int y1 = min(y0 + 1, m_iCellY - 2);

		int z0 = floor((z - h2) * m_fInvSpacing);
		float tz = ((z - h2) - z0 * m_h) * m_fInvSpacing;
		int z1 = min(z0 + 1, m_iCellZ - 2);

		float sx = 1.0 - tx;
		float sy = 1.0 - ty;
		float sz = 1.0 - tz;
		
		// optimize
		if (x0 < m_iCellX && y0 < m_iCellY && z0 < m_iCellZ) m_particleDensity[GetCellId(x0, y0, z0)] += sx * sy * sz;
		if (x0 < m_iCellX && y0 < m_iCellY && z1 < m_iCellZ) m_particleDensity[GetCellId(x0, y0, z1)] += sx * sy * tz;
		if (x0 < m_iCellX && y1 < m_iCellY && z0 < m_iCellZ) m_particleDensity[GetCellId(x0, y1, z0)] += sx * ty * sz;
		if (x0 < m_iCellX && y1 < m_iCellY && z1 < m_iCellZ) m_particleDensity[GetCellId(x0, y1, z1)] += sx * ty * tz;
		if (x1 < m_iCellX && y0 < m_iCellY && z0 < m_iCellZ) m_particleDensity[GetCellId(x1, y0, z0)] += tx * sy * sz;
		if (x1 < m_iCellX && y0 < m_iCellY && z1 < m_iCellZ) m_particleDensity[GetCellId(x1, y0, z1)] += tx * sy * tz;
		if (x1 < m_iCellX && y1 < m_iCellY && z0 < m_iCellZ) m_particleDensity[GetCellId(x1, y1, z0)] += tx * ty * sz;
		if (x1 < m_iCellX && y1 < m_iCellY && z1 < m_iCellZ) m_particleDensity[GetCellId(x1, y1, z1)] += tx * ty * tz;
	}

	if (m_particleRestDensity == 0.0)
	{
		float sum = 0.0;
		int numFluidCells = 0;

		for (int i = 0; i < m_iNumCells; i++)
		{
			if (m_type[i] == FLUID_CELL)
			{
				sum += m_particleDensity[i];
				numFluidCells++;
			}
		}

		if (numFluidCells > 0)
			m_particleRestDensity = sum / numFluidCells;

		//std::cout << "fluid sum: " << sum << std::endl;
		//std::cout << "fluid cells num: " << numFluidCells << std::endl;
	}

	//std::cout << "particle rest density: " << m_particleRestDensity << std::endl;
}

void FlipSimulator::transferVelocities(bool toGrid, float flipRatio)
{
	/* {
		if (toGrid)
			std::cout << "===========to Grid============" << std::endl;
		else
			std::cout << "===========to Particle============" << std::endl;
		Vec3 pos = m_particlePos[followId];
		Vec3 vel = m_particleVel[followId];
		std::cout << "pos[0] before transfer: " << pos.x << " " << pos.y << " " << pos.z << std::endl;
		std::cout << "vel[0] before transfer: " << vel.x << " " << vel.y << " " << vel.z << std::endl;
	}*/

	if (toGrid)
	{
		m_pre_vel.assign(m_vel.begin(), m_vel.end());

		for (int i = 0; i < m_iNumCells; i++)
		{
			m_type[i] = m_s[i] == 0.0 ? SOLID_CELL : EMPTY_CELL;
			m_vel[i] = Vec3(0, 0, 0);
			m_w[i] = Vec3(0, 0, 0);
		}

		for (int i = 0; i < m_iNumSpheres; i++)
		{
			int cellId = GetCellId(m_particlePos[i]);
			if (m_type[cellId] == EMPTY_CELL)
				m_type[cellId] = FLUID_CELL;
		}

	}

	float h2 = m_h / 2.0f; // half of grid size

	for (int axis = 0; axis < 3; axis++)
	{
		float dx = axis != 0 ? h2 : 0;
		float dy = axis != 1 ? h2 : 0;
		float dz = axis != 2 ? h2 : 0;

		for (int i = 0; i < m_iNumSpheres; i++)
		{
			Vec3 pos = m_particlePos[i];

			float x = min(max((float)pos.x, m_h), (m_iCellX - 1) * m_h);
			float y = min(max((float)pos.y, m_h), (m_iCellY - 1) * m_h);
			float z = min(max((float)pos.z, m_h), (m_iCellZ - 1) * m_h);

			int x0 = min((int)floor((x - dx) * m_fInvSpacing), m_iCellX - 2);
			float tx = ((x - dx) - x0 * m_h) * m_fInvSpacing;
			int x1 = min(x0 + 1, m_iCellX - 2);

			int y0 = min((int)floor((y - dy) * m_fInvSpacing), m_iCellY - 2);
			float ty = ((y - dy) - y0 * m_h) * m_fInvSpacing;
			int y1 = min(y0 + 1, m_iCellY - 2);

			int z0 = min((int)floor((z - dz) * m_fInvSpacing), m_iCellZ - 2);
			float tz = ((z - dz) - z0 * m_h) * m_fInvSpacing;
			int z1 = min(z0 + 1, m_iCellZ - 2);

			tx = min(max(tx, 0.0f), 1.0f);
			ty = min(max(ty, 0.0f), 1.0f);
			tz = min(max(tz, 0.0f), 1.0f);

			float sx = 1.0 - tx;
			float sy = 1.0 - ty;
			float sz = 1.0 - tz;

			if (tx < 0.0 || tx > 1.0)
				std::cerr << "tx out of range!" << std::endl;

			float w0 = tx * ty * tz;
			float w1 = tx * ty * sz;
			float w2 = tx * sy * tz;
			float w3 = tx * sy * sz;
			float w4 = sx * ty * tz;
			float w5 = sx * ty * sz;
			float w6 = sx * sy * tz;
			float w7 = sx * sy * sz;

			// optimize
			int nr0 = GetCellId(x1, y1, z1);
			int nr1 = GetCellId(x1, y1, z0);
			int nr2 = GetCellId(x1, y0, z1);
			int nr3 = GetCellId(x1, y0, z0);
			int nr4 = GetCellId(x0, y1, z1);
			int nr5 = GetCellId(x0, y1, z0);
			int nr6 = GetCellId(x0, y0, z1);
			int nr7 = GetCellId(x0, y0, z0);

			if (toGrid)
			{
				float pv = m_particleVel[i][axis];

				//if (w0 > 0.001)
				{
					m_vel[nr0][axis] += pv * w0;
					m_w[nr0][axis] += w0;
				}

				//if (w1 > 0.001)
				{
					m_vel[nr1][axis] += pv * w1;
					m_w[nr1][axis] += w1;
				}

				//if (w2 > 0.001)
				{
					m_vel[nr2][axis] += pv * w2;
					m_w[nr2][axis] += w2;
				}

				//if (w3 > 0.001)
				{
					m_vel[nr3][axis] += pv * w3;
					m_w[nr3][axis] += w3;
				}

				//if (w4 > 0.001)
				{
					m_vel[nr4][axis] += pv * w4;
					m_w[nr4][axis] += w4;
				}

				//if (w5 > 0.001)
				{
					m_vel[nr5][axis] += pv * w5;
					m_w[nr5][axis] += w5;
				}

				//if (w6 > 0.001)
				{
					m_vel[nr6][axis] += pv * w6;
					m_w[nr6][axis] += w6;
				}

				//if (w7 > 0.001)
				{
					m_vel[nr7][axis] += pv * w7;
					m_w[nr7][axis] += w7;
				}

				/* {
					if (nr0 == followId)
					{
						std::cout << "toGrid follow, pv * w1: " << pv * w0 << " , w0: " << w0 << ", pv" << pv << std::endl;
					}
					if (nr1 == followId)
					{
						std::cout << "toGrid follow, pv * w2: " << pv * w1 << " , w1: " << w1 << ", pv" << pv << std::endl;
					}
					if (nr2 == followId)
					{
						std::cout << "toGrid follow, pv * w3: " << pv * w2 << " , w2: " << w2 << ", pv" << pv << std::endl;
					}
					if (nr3 == followId)
					{
						std::cout << "toGrid follow, pv * w4: " << pv * w3 << " , w3: " << w3 << ", pv" << pv << std::endl;
					}
					if (nr4 == followId)
					{
						std::cout << "toGrid follow, pv * w5: " << pv * w4 << " , w4: " << w4 << ", pv" << pv << std::endl;
					}
					if (nr5 == followId)
					{
						std::cout << "toGrid follow, pv * w6: " << pv * w5 << " , w5: " << w5 << ", pv" << pv << std::endl;
					}
					if (nr6 == followId)
					{
						std::cout << "toGrid follow, pv * w7: " << pv * w6 << " , w6: " << w6 << ", pv" << pv << std::endl;
					}
					if (nr7 == followId)
					{
						std::cout << "toGrid follow, pv * w7: " << pv * w7 << " , w7: " << w7 << ", pv" << pv << std::endl;
					}
				}*/

				/*if (pv > 100)
				{
					std::cout << "pv > 100, i: " << i << " axis: " << axis << " pv: " << pv << std::endl;
					std::cout << "pv > 100, w: " << w0 << " " << w1 << " " << w2 << " " << w3 << " " << w4 << " " << w5 << " " << w6 << " " << w7 << std::endl;
				}*/
			}
			else
			{
				int offset = axis == 0 ? (m_iCellY * m_iCellZ) : (axis == 2 ? m_iCellY : 1);
				float valid0 = m_type[nr0] != EMPTY_CELL || m_type[nr0 - offset] != EMPTY_CELL ? 1.0 : 0.0;
				float valid1 = m_type[nr1] != EMPTY_CELL || m_type[nr1 - offset] != EMPTY_CELL ? 1.0 : 0.0;
				float valid2 = m_type[nr2] != EMPTY_CELL || m_type[nr2 - offset] != EMPTY_CELL ? 1.0 : 0.0;
				float valid3 = m_type[nr3] != EMPTY_CELL || m_type[nr3 - offset] != EMPTY_CELL ? 1.0 : 0.0;
				float valid4 = m_type[nr4] != EMPTY_CELL || m_type[nr4 - offset] != EMPTY_CELL ? 1.0 : 0.0;
				float valid5 = m_type[nr5] != EMPTY_CELL || m_type[nr5 - offset] != EMPTY_CELL ? 1.0 : 0.0;
				float valid6 = m_type[nr6] != EMPTY_CELL || m_type[nr6 - offset] != EMPTY_CELL ? 1.0 : 0.0;
				float valid7 = m_type[nr7] != EMPTY_CELL || m_type[nr7 - offset] != EMPTY_CELL ? 1.0 : 0.0;

				float v = m_particleVel[i][axis];
				float d = valid0 * w0 + valid1 * w1 + valid2 * w2 + valid3 * w3 + valid4 * w4 + valid5 * w5 + valid6 * w6 + valid7 * w7;

				//if (d != d) d = 0.5f;

				if (d > 0.0)
				{
					float picV =
						(valid0 * w0 * m_vel[nr0][axis] +
							valid1 * w1 * m_vel[nr1][axis] +
							valid2 * w2 * m_vel[nr2][axis] +
							valid3 * w3 * m_vel[nr3][axis] +
							valid4 * w4 * m_vel[nr4][axis] +
							valid5 * w5 * m_vel[nr5][axis] +
							valid6 * w6 * m_vel[nr6][axis] +
							valid7 * w7 * m_vel[nr7][axis]) / d;
					float corr =
						(valid0 * w0 * (m_vel[nr0][axis] - m_pre_vel[nr0][axis]) +
							valid1 * w1 * (m_vel[nr1][axis] - m_pre_vel[nr1][axis]) +
							valid2 * w2 * (m_vel[nr2][axis] - m_pre_vel[nr2][axis]) +
							valid3 * w3 * (m_vel[nr3][axis] - m_pre_vel[nr3][axis]) +
							valid4 * w4 * (m_vel[nr4][axis] - m_pre_vel[nr4][axis]) +
							valid5 * w5 * (m_vel[nr5][axis] - m_pre_vel[nr5][axis]) +
							valid6 * w6 * (m_vel[nr6][axis] - m_pre_vel[nr6][axis]) +
							valid7 * w7 * (m_vel[nr7][axis] - m_pre_vel[nr7][axis])) / d;
					float flipV = v + corr;

					/*if (i == 0)
					{
						printf("Q: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", m_vel[nr0][axis], m_vel[nr1][axis], m_vel[nr2][axis], m_vel[nr3][axis], m_vel[nr4][axis], m_vel[nr5][axis], m_vel[nr6][axis], m_vel[nr7][axis]);
						printf("preQ: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", m_pre_vel[nr0][axis], m_pre_vel[nr1][axis], m_pre_vel[nr2][axis], m_pre_vel[nr3][axis], m_pre_vel[nr4][axis], m_pre_vel[nr5][axis], m_pre_vel[nr6][axis], m_pre_vel[nr7][axis]);
						std::cout << "i: " << i << " axis: " << axis << " pic : " << picV << " flip : " << flipV << " ratio : " << m_fRatio << " d : " << d << std::endl;
					}*/


					if (picV == picV && flipV == flipV)
						m_particleVel[i][axis] =  (1.0 - m_fRatio) * picV + m_fRatio * flipV;
				}
			}
		}

		if (toGrid)
		{
			for (int i = 0; i < m_iNumCells; i++)
			{
				/*if (i == followId)
				{
					std::cout << "====toGrid followId====" << std::endl;
					std::cout << "m_vel(before div): " << m_vel[i][axis] << " axis: " << axis << "m_w: " << m_w[i][axis] << std::endl;
				}*/
				if (m_w[i][axis] > 0 && m_w[i][axis] == m_w[i][axis])
					m_vel[i][axis] /= m_w[i][axis];

				/*if (m_vel[i][axis] > 100)
				{
					std::cout << "toGrid Cell " << i << " m_vel: " << m_vel[i][axis] << " axis: " << axis << " m_w: " << m_w[i][axis] << std::endl;
				}*/
				/*if (i == followId)
				{
					std::cout << "m_vel(after div): " << m_vel[i][axis] << " axis: " << axis << "m_w: " << m_w[i][axis] << std::endl;
					std::cout << "====toGrid followId====" << std::endl;
				}*/
			}

			
		}
		
	}
	/*{
		Vec3 pos = m_particlePos[followId];
		Vec3 vel = m_particleVel[followId];
		std::cout << "pos[0] after transfer: " << pos.x << " " << pos.y << " " << pos.z << std::endl;
		std::cout << "vel[0] after transfer: " << vel.x << " " << vel.y << " " << vel.z << std::endl;
	}*/

	if (toGrid)
	{
		for (int i = 0; i < m_iCellX; i++)
		{
			for (int j = 0; j < m_iCellY; j++)
			{
				for (int k = 0; k < m_iCellZ; k++)
				{
					int cellId = GetCellId(i, j, k);
					bool solid = m_type[cellId] == SOLID_CELL;

					if (solid || (i > 0 && m_type[GetCellId(i - 1, j, k)] == SOLID_CELL))
						m_vel[cellId][0] = m_pre_vel[cellId][0];

					if (solid || (j > 0 && m_type[GetCellId(i, j - 1, k)] == SOLID_CELL))
						m_vel[cellId][1] = m_pre_vel[cellId][1];

					if (solid || (k > 0 && m_type[GetCellId(i, j, k - 1)] == SOLID_CELL))
						m_vel[cellId][2] = m_pre_vel[cellId][2];
				}
			}
		}
	}
}

void FlipSimulator::solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift)
{
	//std::cout << "================solveIncompress================" << std::endl;

	m_pre_vel.assign(m_vel.begin(), m_vel.end());
	/*std::vector<float> divs;
	divs.resize(m_iNumCells, 0);
	m_p.resize(m_iNumCells, 0);

	for (int i = 1; i < m_iCellX - 1; i++)
	{
		for (int j = 1; j < m_iCellY - 1; j++)
		{
			for (int k = 1; k < m_iCellZ - 1; k++)
			{
				int center = GetCellId(i, j, k);

				int left = GetCellId(i - 1, j, k);
				int right = GetCellId(i + 1, j, k);
				int bottom = GetCellId(i, j - 1, k);
				int top = GetCellId(i, j + 1, k);
				int front = GetCellId(i, j, k - 1);
				int back = GetCellId(i, j, k + 1);

				divs[center] = 0.5f * (m_vel[right][0] - m_vel[center][0] +
					m_vel[top][1] - m_vel[center][1] +
					m_vel[back][2] - m_vel[center][2]);
			}
		}

	}*/

	for (int iter = 0; iter < numIters; iter++)
	{
		for (int i = 1; i < m_iCellX - 1; i++)
		{
			for (int j = 1; j < m_iCellY - 1; j++)
			{
				for (int k = 1; k < m_iCellZ - 1; k++)
				{
					int center = GetCellId(i, j, k);
					if (m_type[center] != FLUID_CELL)
						continue;

					int left = GetCellId(i - 1, j, k);
					int right = GetCellId(i + 1, j, k);
					int bottom = GetCellId(i, j - 1, k);
					int top = GetCellId(i, j + 1, k);
					int front = GetCellId(i, j, k - 1);
					int back = GetCellId(i, j, k + 1);

					float sx0 = m_s[left];
					float sx1 = m_s[right];
					float sy0 = m_s[bottom];
					float sy1 = m_s[top];
					float sz0 = m_s[front];
					float sz1 = m_s[back];
					float s = sx0 + sx1 + sy0 + sy1 + sz0 + sz1;
					if (s <= 0.0f)
						continue;

					//m_p[center] = (divs[center] - (m_p[right] + m_p[left] + m_p[bottom] + m_p[top] + m_p[front] + m_p[back])) / -s;

					float div = (m_vel[right][0] - m_vel[center][0] +
						m_vel[top][1] - m_vel[center][1] +
						m_vel[back][2] - m_vel[center][2]);

					//if (div > 100)
						//std::cout << "Div > 100, center: " << center << " div: " << div << std::endl;

				
					if (m_particleRestDensity > 0.0 && compensateDrift)
					{
						float k = 1.0f;
						float compression = m_particleDensity[center] - m_particleRestDensity;
						if (compression > 0.0)
							div -= k * compression;
					}

					float p = -div / s;
					p *= overRelaxation;

					//if (p > 100)
						//std::cout << "p > 100, center:" << center << " p: " << p << std::endl;

					m_vel[center][0] -= sx0 * p;
					m_vel[right][0] += sx1 * p;
					m_vel[center][1] -= sy0 * p;
					m_vel[top][1] += sy1 * p;
					m_vel[center][2] -= sz0 * p;
					m_vel[back][2] += sz1 * p;

				}
			}

		}
	}

	/*for (int i = 1; i < m_iCellX - 1; i++)
	{
		for (int j = 1; j < m_iCellY - 1; j++)
		{
			for (int k = 1; k < m_iCellZ - 1; k++)
			{
				int center = GetCellId(i, j, k);
				if (m_type[center] != FLUID_CELL)
					continue;

				int left = GetCellId(i - 1, j, k);
				int right = GetCellId(i + 1, j, k);
				int bottom = GetCellId(i, j - 1, k);
				int top = GetCellId(i, j + 1, k);
				int front = GetCellId(i, j, k - 1);
				int back = GetCellId(i, j, k + 1);

				m_vel[center][0] -= (m_p[right] - m_p[left]) * 0.5f;
				m_vel[center][1] -= (m_p[top] - m_p[bottom]) * 0.5f;
				m_vel[center][2] -= (m_p[back] - m_p[front]) * 0.5f;
			}
		}

	}*/

	/*for (int i = 0; i < m_iNumCells; i++)
	{
		if (m_vel[i][2] > 100)
		{
			std::cout << "compress Cell " << i << " m_vel: " << m_vel[i][2] << " m_w: " << m_w[i][2] << std::endl;
		}
	}*/
}

void FlipSimulator::updateParticleColors()
{
	for (int i = 0; i < m_iNumSpheres; i++)
	{
		float s = 0.01;

		m_particleColor[i].x = min(max(m_particleColor[i].x - s, 0.0), 1.0);
		m_particleColor[i].y = min(max(m_particleColor[i].y - s, 0.0), 1.0);
		m_particleColor[i].z = min(max(m_particleColor[i].z + s, 0.0), 1.0);

		int cellNr = GetCellId(m_particlePos[i]);
		float d0 = m_particleRestDensity;
		if (d0 > 0.0)
		{
			float relDensity = m_particleDensity[cellNr] / d0;
			if (relDensity < 0.6)
			{
				float s = 0.8;
				m_particleColor[i].x = m_particleColor[i].y = s;
				m_particleColor[i].z = 1.0;
			}
		}
	}
}

const char* FlipSimulator::getTestCasesStr()
{
	return "Demo 1";
}

void FlipSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:break;
	default:break;
	}
}

void FlipSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void FlipSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	switch (m_iTestCase)
	{
	case 0: 
		DrawParticles();
		break;
	}
}

void FlipSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "========================================Demo 1======================================\n";
		setupScene(12);
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void FlipSimulator::externalForcesCalculations(float timeElapsed)
{
	if (m_bIsClicked)
	{
		// Apply the mouse deltas to g_vfMovableObjectPos(move along cameras view plane)
			Point2D mouseDiff;
		mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
		mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;

		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 1.0f;
		
		inputWorld = inputWorld * inputScale;
		m_obstacleVel = inputWorld;
		m_obstaclePos = m_obstaclePos + inputWorld * timeElapsed;
		
	}
	else
	{
		m_obstacleVel = Vec3(0, 0, 0);
	}
}

void FlipSimulator::onClick(int x, int y)
{
	m_oldtrackmouse.x = m_trackmouse.x;
	m_oldtrackmouse.y = m_trackmouse.y;
	
	m_bIsClicked = true;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void FlipSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
	m_bIsClicked = false;
}


void FlipSimulator::DrawParticles()
{

	int counter = 0;
	for (int i = 0; i < m_iNumSpheres; i++)
	{
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, m_particleColor[i]);
		DUC->drawSphere(m_particlePos[i] - Vec3(0.5), m_particleRadius);

		if (!(m_particlePos[i].x == m_particlePos[i].x) || !(m_particlePos[i].y == m_particlePos[i].y) || !(m_particlePos[i].z == m_particlePos[i].z))
			counter++;
		
	}
	//DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, Vec3(0, 1, 0));
	//DUC->drawSphere(m_particlePos[followId] - Vec3(0.5), m_particleRadius);

	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, Vec3(0, 1, 0));
	DUC->drawSphere(Vec3(-0.5) + m_obstaclePos, m_obstacleRadius);
	//std::cout << "NAN: " << counter << std::endl;
	/*for (int i = 0; i < m_iCellX; i++)
	{
		for (int j = 0; j < m_iCellY; j++)
		{
			for (int k = 0; k < m_iCellZ; k++)
			{
				int cellId = i * m_iCellY * m_iCellZ + k * m_iCellY + j;
				if (m_vel[cellId] > 100)
				{
					DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, Vec3(1.0, 0.0, 0.0));
					DUC->drawSphere(Vec3(i, j, k) * m_h - Vec3(0.5), m_particleRadius);
					//std::cout << "----Draw Particle, cellId: " << cellId << std::endl;
				}
			}
		}
	}*/
}
