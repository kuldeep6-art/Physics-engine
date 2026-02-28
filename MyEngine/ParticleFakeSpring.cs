using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
	using MyEngine.warppers;
	using System;
	using System.Numerics; // For Vector3

	public class ParticleFakeSpring : IParticleForceGenerator
	{
		private Vector3 anchor;       // Location of the fixed end of the spring
		private float springConstant; // Spring stiffness coefficient
		private float damping;        // Damping factor (reduces oscillations)

		public ParticleFakeSpring(Vector3 anchor, float springConstant, float damping)
		{
			this.anchor = anchor;
			this.springConstant = springConstant;
			this.damping = damping;
		}

		public void UpdateForce(Particle particle, float duration)
		{
			// Ensure the particle has a finite mass
			if (particle.hasMass()) return;

			// Calculate the relative position of the particle to the anchor
			Vector3 position = particle.Position - anchor;

			// Compute gamma and check bounds
			float gamma = 0.5f * MathF.Sqrt(4 * springConstant - damping * damping);
			if (gamma == 0.0f) return;

			// Calculate the c term
			Vector3 c = (position * (damping / (2.0f * gamma))) + (particle.Velocity *(1.0f/ gamma));

			// Compute the target position
			Vector3 target = (position * MathF.Cos(gamma * duration)) +
							 (c * MathF.Sin(gamma * duration));
			target *= MathF.Exp(-0.5f * duration * damping);

			// Compute acceleration and resulting force
			Vector3 acceleration = ((target - position) * (1.0f/duration * duration)) -
								   (particle.Velocity * duration);

			particle.addForce(acceleration * particle.getMass());
		}
	}

}
