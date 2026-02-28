using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
	public class ParticleGravity : IParticleForceGenerator,IBodyForceGenerator
	{
		private Vector3 gravity;

		/// <summary>
		/// Initializes the gravity force generator with a given acceleration.
		/// </summary>
		public ParticleGravity(Vector3 gravity)
		{
			this.gravity = gravity;
		}

		/// <summary>
		/// Applies gravitational force to the given particle.
		/// </summary>
		public void UpdateForce(Particle particle, float duration)
		{
			if (particle.hasMass()) return;

			particle.addForce(gravity * particle.getMass());
		}
		public void UpdateForce(RigidBody body, float duration)
		{
			if (body.hasMass()) return;

			body.AddForce(gravity * body.getMass());
		}
	}
}
