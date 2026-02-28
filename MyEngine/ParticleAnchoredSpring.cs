using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
	using System;
	using System.Numerics; // For Vector3

	public class ParticleAnchoredSpring : IParticleForceGenerator
	{
		private Vector3 anchor; // Fixed point in space
		private float springConstant; // Hooke's Law stiffness
		private float restLength; // Natural length of the spring

		/// <summary>
		/// Creates a new anchored spring force generator.
		/// </summary>
		public ParticleAnchoredSpring(Vector3 anchor, float springConstant, float restLength)
		{
			this.anchor = anchor;
			this.springConstant = springConstant;
			this.restLength = restLength;
		}

		/// <summary>
		/// Applies the spring force to the given particle.
		/// </summary>
		public void UpdateForce(Particle particle, float duration)
		{
			// Calculate the force vector from the particle to the anchor
			Vector3 force = particle.Position - anchor;

			// Calculate the magnitude of the force
			float magnitude = force.Magnitude();
			magnitude = Math.Abs(magnitude - restLength);
			magnitude *= springConstant;

			// Normalize and apply force
			if (force != new Vector3()) // Avoid division by zero
			{
				force.normalize();
				force = force* -magnitude;
				particle.addForce(force);
			}
		}
	}

}
