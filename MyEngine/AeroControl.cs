using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{

	public class Aero : IBodyForceGenerator
	{
		/// <summary>
		/// Holds the aerodynamic tensor for the surface in body space.
		/// </summary>
		protected Matrix3 tensor;

		/// <summary>
		/// Holds the relative position of the aerodynamic surface in body coordinates.
		/// </summary>
		protected Vector3 position;

		/// <summary>
		/// Holds a reference to a vector containing the wind speed of the environment.
		/// </summary>
		protected Vector3 windspeed;

		/// <summary>
		/// Creates a new aerodynamic force generator with the given properties.
		/// </summary>
		public Aero(Matrix3 tensor, Vector3 position, ref Vector3 windspeed)
		{
			this.tensor = tensor;
			this.position = position;
			this.windspeed = windspeed;
		}

		/// <summary>
		/// Applies the force to the given rigid body.
		/// </summary>
		public virtual void UpdateForce(RigidBody body, float duration)
		{
			UpdateForceFromTensor(body, duration, tensor);
		}

		public void UpdateForceFromTensor(RigidBody body, float duration, Matrix3 tensor)
		{
			// Calculate total velocity (windspeed and body's velocity)
			Vector3 velocity = body.Velocity;
			velocity += windspeed;

			// Calculate the velocity in body coordinates
			Vector3 bodyVel = body.TransformMatrix.TransformInverseDirection(velocity);

			// Calculate the force in body coordinates
			Vector3 bodyForce = tensor.Transform(bodyVel);
			Vector3 force = body.TransformMatrix.TransformDirection(bodyForce);

			// Apply the force
			body.AddForceAtBodyPoint(force, position);
		}
	}

	public class AeroControl : Aero
	{
		/// <summary>
		/// The aerodynamic tensor for the surface when the control is at its maximum value.
		/// </summary>
		private Matrix3 maxTensor;

		/// <summary>
		/// The aerodynamic tensor for the surface when the control is at its minimum value.
		/// </summary>
		private Matrix3 minTensor;

		/// <summary>
		/// The current position of the control for this surface. 
		/// Ranges between -1 (minTensor) through 0 (base tensor) to +1 (maxTensor).
		/// </summary>
		private float controlSetting;

		/// <summary>
		/// Creates a new aerodynamic control surface with the given properties.
		/// </summary>
		public AeroControl(Matrix3 baseTensor, Matrix3 minTensor, Matrix3 maxTensor,
			Vector3 position, ref Vector3 windspeed)
			: base(baseTensor, position, ref windspeed)
		{
			this.maxTensor = maxTensor;
			this.minTensor = minTensor;
			this.controlSetting = 0f;
		}

		/// <summary>
		/// Calculates the final aerodynamic tensor for the current control setting.
		/// </summary>
		private Matrix3 GetTensor()
		{
			if (controlSetting <= -1.0f)
			{
				return minTensor;
			}
			else if (controlSetting >= 1.0f)
			{
				return maxTensor;
			}
			else if (controlSetting == 0.0f)
			{
				return tensor;
			}
			else if (controlSetting > 0.0f)
			{
				// Linear interpolation between base and max
				float factor = controlSetting;
				return Matrix3.Lerp(tensor, maxTensor, factor);
			}
			else
			{
				// Linear interpolation between min and base
				float factor = controlSetting + 1.0f;
				return Matrix3.Lerp(minTensor, tensor, factor);
			}
		}

		/// <summary>
		/// Sets the control position of this control. Should range between -1 and +1.
		/// Values outside this range give undefined results.
		/// </summary>
		public void SetControl(float value)
		{
			this.controlSetting = value;
		}

		/// <summary>
		/// Applies the force to the given rigid body.
		/// </summary>
		public override void UpdateForce(RigidBody body, float duration)
		{
			Matrix3 currentTensor = GetTensor();
			UpdateForceFromTensor(body, duration, currentTensor);
		}
	}
}