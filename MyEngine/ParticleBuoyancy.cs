using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
	using System;
	using System.Numerics; // For Vector3

	public class ParticleBuoyancy : IParticleForceGenerator
	{
		private float maxDepth;       // Max submersion depth before full buoyancy
		private float volume;         // Volume of the particle
		private float waterHeight;    // Y-level of the water surface
		private float liquidDensity;
		private Vector3 centerOfBuoyancy;// Density of the liquid (1000 kg/m³ for water)

		/// <summary>
		/// Creates a new buoyancy force generator.
		/// </summary>
		public ParticleBuoyancy(Vector3 centerOfBuoyancy, float maxDepth, float volume,
		float waterHeight, float liquidDensity = 1000.0f)
		{
			this.centerOfBuoyancy = centerOfBuoyancy;
			this.maxDepth = maxDepth;
			this.volume = volume;
			this.waterHeight = waterHeight;
			this.liquidDensity = liquidDensity;
		}
		/// <summary>
		/// Applies the buoyancy force to the given particle.
		/// </summary>
		public void UpdateForce(Particle particle, float duration)
		{
			// Get the current depth of the particle
			float depth = particle.Position.Y;

			// Check if completely above water (no buoyancy)
			if (depth >= waterHeight + maxDepth) return;

			Vector3 force = new Vector3();

			// Fully submerged
			if (depth <= waterHeight - maxDepth)
			{
				force.Y = liquidDensity * volume;
			}
			else // Partially submerged
			{
				force.Y = liquidDensity * volume * (depth - maxDepth - waterHeight) / (2 * maxDepth);
			}

			particle.addForce(force);
		}
	}

}
