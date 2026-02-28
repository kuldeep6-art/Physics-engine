using System;
namespace MyEngine
{
    public class ParticleBungee : IParticleForceGenerator
    {
        private readonly Particle other;
        private readonly float springConstant;
        private readonly float restLength;

        public ParticleBungee(Particle other, float springConstant, float restLength)
        {
            this.other = other ?? throw new ArgumentNullException(nameof(other));
            this.springConstant = springConstant;
            this.restLength = restLength;
        }

        public void UpdateForce(Particle particle, float duration)
        {
            if (particle == null) return;

            Vector3 force = particle.Position - other.Position;
            float magnitudeSquared = force.X * force.X + force.Y * force.Y + force.Z * force.Z;
            float restLengthSquared = restLength * restLength;

            if (magnitudeSquared <= restLengthSquared) return;

            float magnitude = (float)Math.Sqrt(magnitudeSquared);
            float forceMagnitude = springConstant * (restLength - magnitude);

            if (forceMagnitude != 0)
            {
                force.X = force.X / magnitude * -forceMagnitude;
                force.Y = force.Y / magnitude * -forceMagnitude;
                force.Z = force.Z / magnitude * -forceMagnitude;
                particle.addForce(force);
            }
        }
    }
}
