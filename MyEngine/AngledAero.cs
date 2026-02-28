using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine
{
	public class AngledAero : Aero
	{
		/// <summary>
		/// Holds the orientation of the aerodynamic surface relative to the rigid body to which it is attached.
		/// </summary>
		private Quaternion orientation;

		/// <summary>
		/// Creates a new aerodynamic surface with the given properties.
		/// </summary>
		public AngledAero(Matrix3 tensor, Vector3 position, ref Vector3 windspeed)
			: base(tensor, position, ref windspeed)
		{
			this.orientation = new Quaternion(); // Default orientation
		}

		/// <summary>
		/// Sets the relative orientation of the aerodynamic surface relative to the rigid body it is attached to.
		/// Note that this doesn't affect the point of connection of the surface to the body.
		/// </summary>
		public void SetOrientation(Quaternion quat)
		{
			this.orientation = quat;
		}

		/// <summary>
		/// Applies the force to the given rigid body.
		/// </summary>
		public override void UpdateForce(RigidBody body, float duration)
		{
			// Implementation would go here
		}
	}
}
