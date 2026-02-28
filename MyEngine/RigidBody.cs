using MyEngine.warppers;
using System;
using System.Collections.Generic;
using System.Diagnostics.Contracts;
using System.Text;

namespace MyEngine
{
	public class RigidBody
	{
		/**
		* Holds the inverse of the mass of the rigid body. It is more
		* useful to hold the inverse mass because integration is simpler,
		* and because in real-time simulation, it is more useful to have
		* bodies with infinite mass (immovable) than zero mass (completely
		* unstable in numerical simulation).
		*/
		public float motion { get; set; }

		// A body can be put to sleep to avoid it being updated by the integration 
		// functions or affected by collisions with the world.
		public bool isAwake { get; set; }

		// Some bodies may never be allowed to fall asleep.
		// User-controlled bodies, for example, should be always awake.
		public bool canSleep { get; set; }
		public Vector3 forceAccum { get; set; }
		public Vector3 torqueAccum { get; set; }
		public Vector3 acceleration { get; set; }
		public Vector3 lastFrameAcceleration { get; set; }
		public float InverseMass { get; set; }
		public Vector3 rotation { get; set; }
		public ParticleContact Contacts { get; set; }
		public float sleepEpsilon = 0.1f; // Default value, can be set via method
		private const float biasBase = 0.5f;
		/**
		* Holds the linear position of the rigid body in world space.
		*/
		public Vector3 Position { get; set; }

		/**
		* Holds the angular orientation of the rigid body in world space.
		*/
		public Quaternion Orientation { get; set; }
		public Matrix3 inverseInertiaTensorWorld { get; set; }
		public float linearDamping { get; set; }
		public float angularDamping { get; set; }
		/**
		* Holds the linear velocity of the rigid body in world space.
		*/
		public Vector3 Velocity { get; set; }
		public void GetInverseInertiaTensorWorld(out Matrix3 tensor)
		{
			tensor = new Matrix3(); // Placeholder
		}

		/**
		* Holds the angular velocity, or rotation, of the rigid body
		* in world space.
		*/
		public void UpdateAcceleration()
		{
			lastFrameAcceleration = new Vector3(acceleration.X, acceleration.Y, acceleration.Z);
			lastFrameAcceleration.addScaledVector(lastFrameAcceleration,forceAccum, InverseMass);
			// Reset ForceAccum after use if needed
		}
		public void SetAwake(bool awake = true)
		{
			if (awake)
			{
				isAwake = true;
				// Add a bit of motion to avoid it falling asleep immediately
				motion = sleepEpsilon * 2.0f;
			}
			else
			{
				isAwake = false;
				Velocity = new Vector3();
				rotation = new Vector3();
			}
		}

		public bool GetAwake()
		{
			return isAwake;
		}

		public void SetCanSleep(bool canSleep)
		{
			this.canSleep = canSleep;
		}

		public bool GetCanSleep()
		{
			return canSleep;
		}

		public void SetSleepEpsilon(float epsilon)
		{
			sleepEpsilon = epsilon;
		}

		public float GetSleepEpsilon()
		{
			return sleepEpsilon;
		}

		// Method to update motion and check if body should sleep
		public void CalculateMotion(float duration)
		{
			if (!canSleep || !isAwake) return;

			// Calculate current motion (squared magnitude of velocity and rotation)
			float currentMotion = Velocity.SquareMagnitude() + rotation.SquareMagnitude();

			// Calculate bias based on frame duration
			float bias = (float)Math.Pow(biasBase, duration);

			// Update recency-weighted average
			motion = bias * motion + (1 - bias) * currentMotion;

			// Limit maximum motion value
			if (motion > 10 * sleepEpsilon) motion = 10 * sleepEpsilon;

			// Check if body should sleep
			if (motion < sleepEpsilon)
			{
				SetAwake(false);
			}
		}
		public void Integrate(float duration)
		{
			if (!isAwake) return;

			// Ensure all required fields are initialized
			forceAccum ??= new Vector3();
			torqueAccum ??= new Vector3();
			inverseInertiaTensorWorld ??= new Matrix3(); // Default to identity if not set

			// Calculate linear acceleration
			lastFrameAcceleration = acceleration;
			lastFrameAcceleration += forceAccum * InverseMass;

			// Calculate angular acceleration
			Vector3 angularAcceleration = inverseInertiaTensorWorld.Transform(torqueAccum);

			// Update velocity and rotation
			Velocity += lastFrameAcceleration * duration;
			rotation += angularAcceleration * duration;

			// Apply damping
			Velocity *= MathF.Pow(linearDamping, duration);
			rotation *= MathF.Pow(angularDamping, duration);

			// Update position and orientation
			Position += Velocity * duration;
			Orientation.AddScaledVector(rotation, duration); // Assuming Quaternion has this method
			Orientation.Normalize(); // Keep orientation normalized

			// Clear accumulators
			ClearAccumulators();

			// Update motion for sleep check
			CalculateMotion(duration);
		}
		public RigidBody()
		{
			forceAccum = new Vector3();
			torqueAccum = new Vector3();
			Velocity = new Vector3();
			rotation = new Vector3();
			acceleration = new Vector3();
			lastFrameAcceleration = new Vector3();
			Position = new Vector3();
			Orientation = new Quaternion(1, 0, 0, 0); // Identity quaternion
			TransformMatrix = new Matrix4(new float[16]); // Identity matrix
			inverseInertiaTensorWorld = new Matrix3(); // Identity matrix
			InverseInertiaTensor = new Matrix3(); // Identity matrix
			linearDamping = 0.99f;
			angularDamping = 0.99f;
			isAwake = true;
			canSleep = true;
			motion = 0f;
		}
		public bool hasMass()
		{
			if (1 / InverseMass > 0)
			{
				return true;
			}
			else
			{
				return false;
			}
		}

		/// <summary>
		/// Clears the force accumulator.
		/// </summary>
		public void ClearAccumulators()
		{
			forceAccum = new Vector3();
			torqueAccum = new Vector3();
		}
	

		/**
		* Holds a transform matrix for converting body space into world
		* space and vice versa. This can be achieved by calling the
		* getPointIn*Space functions.
		*/
		public Matrix4 TransformMatrix { get; set; }
		public void CalculateDerivedData()
		{
			// Calculate the transform matrix for the body.
			_CalculateTransformMatrix(TransformMatrix, Position, Orientation);
		}
		public void AddForce(Vector3 force)
		{
			forceAccum += force;
		}
		public float getMass()
		{
			return 1 / InverseMass;
		}
		public void AddForceAtBodyPoint(Vector3 force, Vector3 point)
		{
			// Convert to coordinates relative to the center of mass.
			Vector3 pt = GetPointInWorldSpace(point);
			AddForceAtPoint(force, pt);
		}
		public Vector3 GetPointInWorldSpace(Vector3 point)
		{
			// Placeholder implementation, assuming identity transformation.
			return point;
		}
		public void AddForceAtPoint(Vector3 force, Vector3 point)
		{
			// Placeholder implementation.
			AddForce(force);
		}
		/**
		* Inline function that creates a transform matrix from a position
		* and orientation.
		*/
		public static void _CalculateTransformMatrix(Matrix4 transformMatrix, Vector3 position, Quaternion orientation)
		{
			transformMatrix.data[0] = 1 - 2 * orientation.J * orientation.J - 2 * orientation.K * orientation.K;
			transformMatrix.data[1] = 2 * orientation.I * orientation.J - 2 * orientation.R * orientation.K;
			transformMatrix.data[2] = 2 * orientation.I * orientation.K + 2 * orientation.R * orientation.J;
			transformMatrix.data[3] = position.X;

			transformMatrix.data[4] = 2 * orientation.I * orientation.J + 2 * orientation.R * orientation.K;
			transformMatrix.data[5] = 1 - 2 * orientation.I * orientation.I - 2 * orientation.K * orientation.K;
			transformMatrix.data[6] = 2 * orientation.J * orientation.K - 2 * orientation.R * orientation.I;
			transformMatrix.data[7] = position.Y;

			transformMatrix.data[8] = 2 * orientation.I * orientation.K - 2 * orientation.R * orientation.J;
			transformMatrix.data[9] = 2 * orientation.J * orientation.K + 2 * orientation.R * orientation.I;
			transformMatrix.data[10] = 1 - 2 * orientation.I * orientation.I - 2 * orientation.J * orientation.J;
			transformMatrix.data[11] = position.Z;
		}
		public Matrix3 InverseInertiaTensor { get; set; }

		// Method to set the inertia tensor and calculate its inverse.
		public void SetInertiaTensor(Matrix3 inertiaTensor)
		{
			// Calculate the inverse inertia tensor and set it.
			InverseInertiaTensor = inertiaTensor.Inverse();

			// Check if the inverse inertia tensor is valid.
			CheckInverseInertiaTensor(InverseInertiaTensor);
		}

		// Method to check if the inverse inertia tensor is valid (non-degenerate).
		public void CheckInverseInertiaTensor(Matrix3 inverseTensor)
		{
			// Implement the check here, for example:
			if (inverseTensor.IsDegenerate())
			{
				throw new InvalidOperationException("Inertia tensor is degenerate (zero inertia for spinning along one axis).");
			}
		}
		// Assuming the Matrix3 and Matrix4 classes are defined similarly to your previous example.
		// Assuming Matrix3 and Matrix4 are defined properly elsewhere, with the necessary methods and properties.

		public static void TransformInertiaTensor(ref Matrix3 iitWorld,
	   Quaternion q,
	   Matrix3 iitBody,
	   Matrix4 rotmat)
		{
			float t4 = rotmat.data[0] * iitBody.data[0] +
					  rotmat.data[1] * iitBody.data[3] +
					  rotmat.data[2] * iitBody.data[6];

			float t9 = rotmat.data[0] * iitBody.data[1] +
					  rotmat.data[1] * iitBody.data[4] +
					  rotmat.data[2] * iitBody.data[7];

			float t14 = rotmat.data[0] * iitBody.data[2] +
					   rotmat.data[1] * iitBody.data[5] +
					   rotmat.data[2] * iitBody.data[8];

			float t28 = rotmat.data[4] * iitBody.data[0] +
					   rotmat.data[5] * iitBody.data[3] +
					   rotmat.data[6] * iitBody.data[6];

			float t33 = rotmat.data[4] * iitBody.data[1] +
					   rotmat.data[5] * iitBody.data[4] +
					   rotmat.data[6] * iitBody.data[7];

			float t38 = rotmat.data[4] * iitBody.data[2] +
					   rotmat.data[5] * iitBody.data[5] +
					   rotmat.data[6] * iitBody.data[8];

			float t52 = rotmat.data[8] * iitBody.data[0] +
					   rotmat.data[9] * iitBody.data[3] +
					   rotmat.data[10] * iitBody.data[6];

			float t57 = rotmat.data[8] * iitBody.data[1] +
					   rotmat.data[9] * iitBody.data[4] +
					   rotmat.data[10] * iitBody.data[7];

			float t62 = rotmat.data[8] * iitBody.data[2] +
					   rotmat.data[9] * iitBody.data[5] +
					   rotmat.data[10] * iitBody.data[8];

			iitWorld.data[0] = t4 * rotmat.data[0] +
							  t9 * rotmat.data[1] +
							  t14 * rotmat.data[2];

			iitWorld.data[1] = t4 * rotmat.data[4] +
							  t9 * rotmat.data[5] +
							  t14 * rotmat.data[6];

			iitWorld.data[2] = t4 * rotmat.data[8] +
							  t9 * rotmat.data[9] +
							  t14 * rotmat.data[10];

			iitWorld.data[3] = t28 * rotmat.data[0] +
							  t33 * rotmat.data[1] +
							  t38 * rotmat.data[2];

			iitWorld.data[4] = t28 * rotmat.data[4] +
							  t33 * rotmat.data[5] +
							  t38 * rotmat.data[6];

			iitWorld.data[5] = t28 * rotmat.data[8] +
							  t33 * rotmat.data[9] +
							  t38 * rotmat.data[10];

			iitWorld.data[6] = t52 * rotmat.data[0] +
							  t57 * rotmat.data[1] +
							  t62 * rotmat.data[2];

			iitWorld.data[7] = t52 * rotmat.data[4] +
							  t57 * rotmat.data[5] +
							  t62 * rotmat.data[6];

			iitWorld.data[8] = t52 * rotmat.data[8] +
							  t57 * rotmat.data[9] +
							  t62 * rotmat.data[10];
		}
      
	 

		// In the RigidBody class, this function can be called to transform the inertia tensor.
		
	}

}
