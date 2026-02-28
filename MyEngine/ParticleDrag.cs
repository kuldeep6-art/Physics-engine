using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
	using System;
	using System.Numerics; // For Vector3

	public class ParticleDrag : IParticleForceGenerator
	{
		private float k1; // Linear drag coefficient
		private float k2; // Quadratic drag coefficient

		/// <summary>
		/// Initializes the drag force generator with given coefficients.
		/// </summary>
		public ParticleDrag(float k1, float k2)
		{
			this.k1 = k1;
			this.k2 = k2;
		}
		

		/// <summary>
		/// Applies drag force to the given particle.
		/// </summary>
		public void UpdateForce(Particle particle, float duration)
		{
			Vector3 force = particle.Velocity;

			float dragCoeff = force.Magnitude();
			dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

			if (force != new Vector3()) // Avoid division by zero
			{
				 force.normalize();
				force = force * -dragCoeff;
				particle.addForce(force);
			}
		}
	}

}
