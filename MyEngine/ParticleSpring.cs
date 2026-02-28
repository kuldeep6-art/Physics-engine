using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
	using MyEngine.warppers;
	using System;
	using System.Numerics; // For Vector3

	public class ParticleSpring : IParticleForceGenerator
	{
		private Vector3 connectionPoint;
		private Vector3 otherConnectionPoint;
		private RigidBody otherB;
		private Particle otherP; // The second particle attached to the spring
		private float springConstant; // Spring stiffness (Hooke’s Law)
		private float restLength; // Natural length of the spring

		/// <summary>
		/// Creates a spring force generator.
		/// </summary>
		public ParticleSpring(Particle other, float springConstant, float restLength)
		{
			this.otherP = other;
			this.springConstant = springConstant;
			this.restLength = restLength;
		}
		public ParticleSpring(Vector3 localConnectionPt, RigidBody other, Vector3 otherConnectionPt, float springConstant, float restLength)
		{
			this.connectionPoint = localConnectionPt;
			this.otherB = other;
			this.otherConnectionPoint = otherConnectionPt;
			this.springConstant = springConstant;
			this.restLength = restLength;
		}

		/// <summary>
		/// Applies the spring force to the given particle.
		/// </summary>
		public void UpdateForce(Particle particle, float duration)
		{
			// Calculate the vector from this particle to the other
			Vector3 force = particle.Position - otherP.Position;

			// Calculate the magnitude of the force
			float magnitude = force.Magnitude();
			magnitude = Math.Abs(magnitude - restLength);
			magnitude *= springConstant;

			// Normalize the force vector and apply the force
			if (force != new Vector3()) // Avoid division by zero
			{
				force.normalize();
				force = force* -magnitude;
				particle.addForce(force);
			}
		}
		public void UpdateForce(RigidBody body, float duration)
		{
			Vector3 lws = body.GetPointInWorldSpace(connectionPoint);
			Vector3 ows = otherB.GetPointInWorldSpace(otherConnectionPoint);
			Vector3 force = lws - ows;
			float magnitude = force.Magnitude();
			magnitude = MathF.Abs(magnitude - restLength);
			magnitude *= springConstant;
			force.normalize();
				force=force* -magnitude;
			body.AddForceAtPoint(force, lws);
		}
	}

}
