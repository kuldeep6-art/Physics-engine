using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MyEngine
{
   public class Particle
    {
        public Vector3 Position{ get; set; }
        public Vector3 Velocity { get; set; }
		public Vector3 Acceleration { get; set; }
        public float Damping { get; set; }
        public float InverseMass { get; set; }
		public Vector3 ForceAccum { get; set; }
		public void ClearAccumulator()
		{
			ForceAccum = new Vector3(); // Reset accumulated force
		}

		public Particle(Vector3 position, Vector3 velocity, Vector3 acceleration, float inverseMass, float damping)
		{
			Position = position;
			Velocity = velocity;
			Acceleration = acceleration;
			ForceAccum = new Vector3(0, 0, 0);
			InverseMass = inverseMass;
			Damping = damping;
		}

		public void addForce(Vector3 force)
		{
			ForceAccum += force;
		}
		public bool hasMass()
		{
			if (1/InverseMass > 0)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		public float getMass()
		{
			return 1 / InverseMass;
		}
		public void Integrate(float duration)
		{
			Debug.Assert(duration > 0.0f);

			// Update position using velocity
			Position += Velocity * duration;

			// Compute acceleration from force
			Vector3 resultingAcc = Acceleration;
			resultingAcc += ForceAccum * InverseMass;

			// Update velocity
			Velocity += resultingAcc * duration;

			// Apply damping
			Velocity *= (float)Math.Pow(Damping, duration);
		}

	}
}
