using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
	public interface IParticleForceGenerator
	{
		void UpdateForce(Particle particle, float duration);
	}

}
