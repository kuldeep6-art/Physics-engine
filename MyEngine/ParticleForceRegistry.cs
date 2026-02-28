using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
	public class ParticleForceRegistry
	{
		// Struct to keep track of registered forces
		private struct ParticleForceRegistration
		{
			public Particle Particle;
			public IParticleForceGenerator ForceGenerator;

			public ParticleForceRegistration(Particle particle, IParticleForceGenerator fg)
			{
				Particle = particle;
				ForceGenerator = fg;
			}
		}

		private List<ParticleForceRegistration> registrations = new List<ParticleForceRegistration>();

		/// <summary>
		/// Registers a force generator for a specific particle.
		/// </summary>
		public void Add(Particle particle, IParticleForceGenerator fg)
		{
			registrations.Add(new ParticleForceRegistration(particle, fg));
		}

		/// <summary>
		/// Removes a specific force generator from a particle.
		/// </summary>
		public void Remove(Particle particle, IParticleForceGenerator fg)
		{
			registrations.RemoveAll(r => r.Particle == particle && r.ForceGenerator == fg);
		}

		/// <summary>
		/// Clears all force registrations.
		/// </summary>
		public void Clear()
		{
			registrations.Clear();
		}

		/// <summary>
		/// Calls all force generators to apply forces to their corresponding particles.
		/// </summary>
		public void UpdateForces(float duration)
		{
			foreach (var reg in registrations)
			{
				reg.ForceGenerator.UpdateForce(reg.Particle, duration);
			}
		}
	}
}
