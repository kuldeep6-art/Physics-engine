using MyEngine;
using System;
using System.Collections.Generic;
using System.Diagnostics.Contracts;
using System.Text;

namespace MyEngine
{
	public class ParticleContact
	{
		/// <summary>
		/// Holds the particles that are involved in the contact.
		/// The second of these can be null, for contacts with the scenery.
		/// </summary>
		private Matrix3[] inverseInertiaTensor = new Matrix3[2];
		public float friction { get; set; }
		public ParticleContact NextInOrder { get; set; }
		public ParticleContact PreviousInOrder { get; set; }
		public ParticleContact[] NextObject { get; } = new ParticleContact[2];
		public Particle[] Particles { get; set; } = new Particle[2];
		public Vector3 contactVelocity { get; set; }
		/// <summary>
		/// Holds the normal restitution coefficient at the contact.
		/// </summary>
     private float VelocityLimit { get; set; } = 0.1f; // Default velocity limit for restitution adjustment
		public float Restitution { get; set; } = 0.5f;

		public Vector3 contactPoint { get; set; }
		/// <summary>
		/// Holds the direction of the contact in world coordinates.
		/// </summary>
		public Vector3 ContactNormal { get; set; }
		public float penetration { get; set; }


		public RigidBody[] Body { get; set; } = new RigidBody[2]; // Array for two bodies
		public float Friction;
		public Matrix3 ContactToWorld { get; set; } // Added to store the basis matrix
		public Vector3[] RelativeContactPosition { get; set; } // Added for relative positions
		public Matrix3[] InverseInertiaTensor { get; set; }   // Added for inertia tensors
		public float DeltaVelocity { get; set; }
		public ParticleContact()
		{
			ContactToWorld = new Matrix3();
			RelativeContactPosition = new Vector3[2] { new Vector3(0, 0, 0), new Vector3(0, 0, 0) };
			InverseInertiaTensor = new Matrix3[2] { new Matrix3(), new Matrix3() };
		}
		public ParticleContact(Vector3 point, Vector3 normal, float penetration)
		{
			contactPoint = point;
			ContactNormal = normal;
			penetration = penetration;
			Body = new RigidBody[2];
			Restitution = 0f;
			Friction = 0f;
			ContactToWorld = new Matrix3();
			RelativeContactPosition = new Vector3[2] { new Vector3(0, 0, 0), new Vector3(0, 0, 0) };
			InverseInertiaTensor = new Matrix3[2] { new Matrix3(), new Matrix3() };
			DeltaVelocity = 0f;
		}
		/// <summary>
		/// Resolves this contact, for both velocity and interpenetration.
		/// </summary>
		/// /// <summary>
		/// Holds the rigid bodies involved in the contact. The second can be null for scenery contacts.
		/// </summary>
		


		
		

		// Linked list properties for contact management
		

		// Constructors
	


		public void CalculateInternals(float duration)
		{
			if (Body[0] == null) SwapBodies();
			if (Body[0] == null) throw new InvalidOperationException("First body cannot be null after swap");

			CalculateContactBasis();

			RelativeContactPosition[0] = contactPoint - Body[0].Position;
			if (Body[1] != null)
			{
				RelativeContactPosition[1] = contactPoint - Body[1].Position;
			}

			contactVelocity = CalculateLocalVelocity(0, duration);
			if (Body[1] != null)
			{
				contactVelocity -= CalculateLocalVelocity(1, duration);
			}

			CalculateDesiredDeltaVelocity(duration);
		}

		public void Resolve(float duration)
		{
			ResolveVelocity(duration);
			ResolveInterpenetration(duration);
		}

		private void ResolveVelocity(float duration)
		{
			float separatingVelocity = CalculateSeparatingVelocity();

			if (separatingVelocity > 0) return;

			float newSepVelocity = -separatingVelocity * Restitution;

			Vector3 accCausedVelocity = Body[0]?.lastFrameAcceleration ?? new Vector3();
			if (Body[1] != null)
				accCausedVelocity -= Body[1].lastFrameAcceleration;
			float accCausedSepVelocity = accCausedVelocity.ScalarProduct(accCausedVelocity, ContactNormal) * duration;

			if (accCausedSepVelocity < 0)
			{
				newSepVelocity += Restitution * accCausedSepVelocity;
				if (newSepVelocity < 0) newSepVelocity = 0;
			}

			float deltaVelocity = newSepVelocity - separatingVelocity;
			float totalInverseMass = Body[0]?.InverseMass ?? 0;
			if (Body[1] != null)
				totalInverseMass += Body[1].InverseMass;

			if (totalInverseMass <= 0) return;

			float impulse = deltaVelocity / totalInverseMass;
			Vector3 impulsePerIMass = ContactNormal * impulse;

			if (Body[0] != null)
				Body[0].Velocity += impulsePerIMass * Body[0].InverseMass;
			if (Body[1] != null)
				Body[1].Velocity -= impulsePerIMass * Body[1].InverseMass;
		}

		private void ResolveInterpenetration(float duration)
		{
			if (penetration <= 0) return;

			float totalInverseMass = Body[0]?.InverseMass ?? 0;
			if (Body[1] != null)
				totalInverseMass += Body[1].InverseMass;

			if (totalInverseMass <= 0) return;

			Vector3 movePerMass = ContactNormal * (-penetration / totalInverseMass);

			if (Body[0] != null)
				Body[0].Position += movePerMass * Body[0].InverseMass;
			if (Body[1] != null)
				Body[1].Position -= movePerMass * Body[1].InverseMass;
		}

		public float CalculateSeparatingVelocity()
		{
			Vector3 relativeVelocity = Body[0]?.Velocity ?? new Vector3();
			if (Body[1] != null)
				relativeVelocity -= Body[1].Velocity;
			return relativeVelocity.ScalarProduct(relativeVelocity, ContactNormal);
		}

		private Vector3 CalculateLocalVelocity(uint bodyIndex, float duration)
		{
			RigidBody thisBody = Body[bodyIndex];
			if (thisBody == null) return new Vector3();

			Vector3 velocity = thisBody.rotation.vectorProduct(RelativeContactPosition[bodyIndex]);
			velocity += thisBody.Velocity;

			Vector3 contactVelocity = ContactToWorld.TransformTranspose(velocity);
			Vector3 accVelocity = thisBody.lastFrameAcceleration * duration;
			accVelocity = ContactToWorld.TransformTranspose(accVelocity);
			accVelocity.X = 0; // Ignore normal direction acceleration
			contactVelocity += accVelocity;

			return contactVelocity;
		}

		public void CalculateDesiredDeltaVelocity(float duration)
		{
			float velocityFromAcc = Body[0]?.lastFrameAcceleration.ScalarProduct(Body[0]?.lastFrameAcceleration, ContactNormal) * duration ?? 0;
			if (Body[1] != null)
			{
				velocityFromAcc -= Body[1].lastFrameAcceleration.ScalarProduct(Body[1].lastFrameAcceleration, ContactNormal) * duration;
			}

			float thisRestitution = Math.Abs(contactVelocity.X) < 0.01f ? 0.0f : Restitution; // velocityLimit assumed as 0.01
			DeltaVelocity = -contactVelocity.X - thisRestitution * (contactVelocity.X - velocityFromAcc);
		}
		

		private void CalculateContactBasis()
		{
			Vector3[] contactTangent = new Vector3[2];
			float s;

			if (Math.Abs(ContactNormal.X) > Math.Abs(ContactNormal.Y))
			{
				s = 1.0f / (float)Math.Sqrt(ContactNormal.Z * ContactNormal.Z + ContactNormal.X * ContactNormal.X);
				contactTangent[0] = new Vector3(ContactNormal.Z * s, 0, -ContactNormal.X * s);
				contactTangent[1] = new Vector3(
					ContactNormal.Y * contactTangent[0].X,
					ContactNormal.Z * contactTangent[0].X - ContactNormal.X * contactTangent[0].Z,
					-ContactNormal.Y * contactTangent[0].X);
			}
			else
			{
				s = 1.0f / (float)Math.Sqrt(ContactNormal.Z * ContactNormal.Z + ContactNormal.Y * ContactNormal.Y);
				contactTangent[0] = new Vector3(0, -ContactNormal.Z * s, ContactNormal.Y * s);
				contactTangent[1] = new Vector3(
					ContactNormal.Y * contactTangent[0].Z - ContactNormal.Z * contactTangent[0].Y,
					-ContactNormal.X * contactTangent[0].Z,
					ContactNormal.X * contactTangent[0].Y);
			}

			ContactToWorld.SetComponents(ContactNormal, contactTangent[0], contactTangent[1]);
		}

		public void SwapBodies()
		{
			ContactNormal *= -1;
			RigidBody temp = Body[0];
			Body[0] = Body[1];
			Body[1] = temp;
		}
		public void ApplyVelocityChange()
		{
			float inverseMass = Body[0].InverseMass;
			Matrix3 impulseToTorque = new Matrix3();
			impulseToTorque.SetSkewSymmetric(RelativeContactPosition[0]);

			Matrix3 deltaVelWorld = impulseToTorque;
			deltaVelWorld *= inverseInertiaTensor[0];
			deltaVelWorld *= impulseToTorque;
			deltaVelWorld *= -1;

			if (Body[1] != null)
			{
				Body[1].GetInverseInertiaTensorWorld(out inverseInertiaTensor[1]);
				impulseToTorque.SetSkewSymmetric(RelativeContactPosition[1]);

				Matrix3 deltaVelWorld2 = impulseToTorque;
				deltaVelWorld2 *= inverseInertiaTensor[1];
				deltaVelWorld2 *= impulseToTorque;
				deltaVelWorld2 *= -1;

				deltaVelWorld += deltaVelWorld2;
				inverseMass += Body[1].InverseMass;
			}

			Matrix3 deltaVelocity = ContactToWorld.Transpose();
			deltaVelocity *= deltaVelWorld;
			deltaVelocity *= ContactToWorld;

			deltaVelocity.data[0] += inverseMass;
			deltaVelocity.data[4] += inverseMass;
			deltaVelocity.data[8] += inverseMass;

			Matrix3 impulseMatrix = deltaVelocity.Inverse();
			Vector3 velKill = new Vector3(DeltaVelocity,
										-contactVelocity.Y,
										-contactVelocity.Z);
			Vector3 impulseContact = impulseMatrix.Transform(velKill);

			float planarImpulse = (float)Math.Sqrt(impulseContact.Y * impulseContact.Y +
												 impulseContact.Z * impulseContact.Z);

			if (planarImpulse > impulseContact.X * friction)
			{
				impulseContact.Y /= planarImpulse;
				impulseContact.Z /= planarImpulse;
				impulseContact.X = deltaVelocity.data[0] +
								 deltaVelocity.data[1] * friction * impulseContact.Y +
								 deltaVelocity.data[2] * friction * impulseContact.Z;
				impulseContact.X = DeltaVelocity / impulseContact.X;
				impulseContact.Y *= friction * impulseContact.X;
				impulseContact.Z *= friction * impulseContact.X;
			}
		}
		public void MatchAwakeState()
		{
			// Collisions with the world (null second body) never cause a body to wake up
			if (Body[1] == null) return;

			bool body0awake = Body[0].GetAwake();
			bool body1awake = Body[1].GetAwake();

			// Wake up only the sleeping one using XOR
			if (body0awake ^ body1awake)
			{
				if (body0awake)Body[1].SetAwake();
				else Body[0].SetAwake();
			}
		}
		public void ApplyPositionChange(out Vector3[] velocityChange, out Vector3[] rotationChange, out float[] rotationAmount, float maxPenetration)
		{
			velocityChange = new Vector3[2];
			rotationChange = new Vector3[2];
			rotationAmount = new float[2];

			// Simplified implementation - actual logic would depend on physics requirements
			if (Body[0] != null)
			{
				velocityChange[0] = ContactNormal * (penetration > 0 ? penetration : 0);
				rotationChange[0] = new Vector3(0, 0, 0); // Placeholder
				rotationAmount[0] = 0; // Placeholder
			}
			if (Body[1] != null)
			{
				velocityChange[1] = ContactNormal * (penetration > 0 ? -penetration : 0);
				rotationChange[1] = new Vector3(0, 0, 0); // Placeholder
				rotationAmount[1] = 0; // Placeholder
			}
			penetration -= maxPenetration; // Reduce penetration
		}
		
		public void UpdatePenetration(Vector3[] velocityChange, Vector3[] rotationChange, float[] rotationAmount)
		{
			Vector3 cp;
			if (Body[0] != null)
			{
				cp = rotationChange[0].vectorProduct(RelativeContactPosition[0]) + velocityChange[0];
				penetration -= rotationAmount[0] * cp.ScalarProduct(cp,ContactNormal);
			}
			if (Body[1] != null)
			{
				cp = rotationChange[1].vectorProduct(RelativeContactPosition[1]) + velocityChange[1];
				penetration += rotationAmount[1] * cp.ScalarProduct(cp,ContactNormal);
			}
		}
		private Vector3 CalculateClosingVelocity()
		{
			Vector3 velocity = Body[0].rotation.vectorProduct(RelativeContactPosition[0]);
			velocity += Body[0].Velocity;

			if (Body[1] != null)
			{
				Vector3 velocity2 = Body[1].rotation.vectorProduct(RelativeContactPosition[1]);
				velocity2 += Body[1].Velocity;
				velocity += velocity2;
			}

			return velocity;
		}

		private void ApplyImpulse(RigidBody body, Vector3 impulse, Vector3 relativeContactPosition)
		{
			Vector3 velocityChange = impulse * body.InverseMass;
			body.Velocity += velocityChange;

			Vector3 impulsiveTorque = impulse.vectorProduct(relativeContactPosition);
			Matrix3 inverseInertiaTensor;
			inverseInertiaTensor=body.inverseInertiaTensorWorld;
			Vector3 rotationChange = inverseInertiaTensor.Transform(impulsiveTorque);
			body.rotation += rotationChange;
		}

		public void ResolveInterpenetration()
		{
			if (penetration <= 0) return;

			float[] linearInertia = new float[2];
			float[] angularInertia = new float[2];
			float totalInertia = 0;

			for (int i = 0; i < 2; i++)
			{
				if (Body[i] != null)
				{
					Matrix3 inverseInertiaTensor;
					inverseInertiaTensor=Body[i].inverseInertiaTensorWorld;
					Vector3 angularInertiaWorld = RelativeContactPosition[i].vectorProduct(ContactNormal);
					angularInertiaWorld = inverseInertiaTensor.Transform(angularInertiaWorld);
					angularInertiaWorld = angularInertiaWorld.vectorProduct(RelativeContactPosition[i]);
					angularInertia[i] = angularInertiaWorld.ScalarProduct(angularInertiaWorld, ContactNormal);
					linearInertia[i] = Body[i].InverseMass;
					totalInertia += linearInertia[i] + angularInertia[i];
				}
			}

			float inverseInertia = 1.0f / totalInertia;
			float[] linearMove = new float[2] { penetration * linearInertia[0] * inverseInertia, -penetration * linearInertia[1] * inverseInertia };
			float[] angularMove = new float[2] { penetration * angularInertia[0] * inverseInertia, -penetration * angularInertia[1] * inverseInertia };

			const float angularLimitConstant = 0.2f;
			for (int i = 0; i < 2; i++)
			{
				if (Body[i] != null)
				{
					float limit = angularLimitConstant * RelativeContactPosition[i].Magnitude();
					if (Math.Abs(angularMove[i]) > limit)
					{
						float totalMove = linearMove[i] + angularMove[i];
						angularMove[i] = angularMove[i] >= 0 ? limit : -limit;
						linearMove[i] = totalMove - angularMove[i];
					}

					Body[i].Position += ContactNormal * linearMove[i];

					Matrix3 inverseInertiaTensor;
					inverseInertiaTensor=Body[i].inverseInertiaTensorWorld;
					Vector3 impulsiveTorque = RelativeContactPosition[i].vectorProduct(ContactNormal);
					Vector3 impulsePerMove = inverseInertiaTensor.Transform(impulsiveTorque);
					Vector3 rotationPerMove = impulsePerMove * (1.0f / angularInertia[i]);
					Vector3 rotation = rotationPerMove * angularMove[i];
					// Apply rotation to orientation (assuming quaternion update method exists)
					// Body[i].UpdateOrientation(rotation); // Placeholder
				}
			}
		}
		public void CalculateDeltaVelocity()
		{
			// Build a vector showing the change in velocity in world space
			// for a unit impulse in the direction of the contact normal
			Vector3 deltaVelWorld = RelativeContactPosition[0].vectorProduct(ContactNormal);
			deltaVelWorld = InverseInertiaTensor[0].Transform(deltaVelWorld);
			deltaVelWorld = deltaVelWorld.vectorProduct(RelativeContactPosition[0]);

			// Work out the change in velocity in contact coordinates
			DeltaVelocity = deltaVelWorld.ScalarProduct(deltaVelWorld, ContactNormal);

			// Add the linear component of velocity change
			DeltaVelocity += Body[0].InverseMass;

			// Check whether we need to consider the second body’s data
			if (Body[1] != null)
			{
				// Get the inverse inertia tensor for the second body
				InverseInertiaTensor[1] = Body[1].inverseInertiaTensorWorld;

				// Go through the same transformation sequence for the second body
				deltaVelWorld = RelativeContactPosition[1].vectorProduct(ContactNormal);
				deltaVelWorld = InverseInertiaTensor[1].Transform(deltaVelWorld);
				deltaVelWorld = deltaVelWorld.vectorProduct(RelativeContactPosition[1]);

				// Add the change in velocity due to rotation
				DeltaVelocity += deltaVelWorld.ScalarProduct(deltaVelWorld, ContactNormal);

				// Add the change in velocity due to linear motion
				DeltaVelocity += Body[1].InverseMass;
			}
		}
	


		public void resolveInterpenetration(float duration)
		{
			if (penetration <= 0) return;
			float totalInverseMass = Particles[0].InverseMass;
			if (Particles[1] != null)
			{
				totalInverseMass += Particles[1].InverseMass;
			}
			if (totalInverseMass <= 0) return;
			Vector3 movePerMass = ContactNormal * (-penetration / totalInverseMass);

			Particles[1].Position = Particles[1].Position + movePerMass * Particles[1].InverseMass;
		}
		/// <summary>
		/// Calculates the separating velocity at this contact.
		/// </summary>
	
	}
}
