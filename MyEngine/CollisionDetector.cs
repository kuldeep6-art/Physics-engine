using System;
using System.Collections.Generic;
using System.Diagnostics.Contracts;
using System.Text;

namespace MyEngine
{
	public static class CollisionDetector
	{
		public static uint SphereAndSphere(Sphere one, Sphere two, CollisionData data)
		{
			// Make sure we have contacts available
			if (data.ContactsLeft <= 0) return 0;
			// Extract translation from the matrix (row-major, translation in data[3], [7], [11])
			Vector3 positionOne = new Vector3(one.Offset.data[3], one.Offset.data[7], one.Offset.data[11]);
			Vector3 positionTwo = new Vector3(two.Offset.data[3], two.Offset.data[7], two.Offset.data[11]);

			// Find the vector between the objects
			Vector3 midline = positionOne - positionTwo;
			float size = midline.Magnitude();

			// See if the spheres are overlapping
			float totalRadius = one.Radius + two.Radius;
			if (size <= 0.0f || size >= totalRadius)
			{
				return 0;
			}

			// Manually create the normal using the size
			Vector3 normal = midline * (1.0f / size);

			// Set up the contact data
			ParticleContact contact = data.Contacts[0]; // Assuming first available contact
			contact.ContactNormal = normal;
			contact.contactPoint = positionOne + midline * 0.5f;
			contact.penetration = totalRadius - size;

			// Write the appropriate data
			contact.Body[0] = one.Body;
			contact.Body[1] = two.Body;
			contact.Restitution = data.Restitution; // Assuming CollisionData has these
			contact.Friction = data.Friction;

			// Update the contact array and count
			data.Contacts[0] = contact; // Update the array with the modified contact
			data.ContactsLeft--;        // Reduce available contacts
			return 1;
		}
		public static uint SphereAndHalfSpace(Sphere sphere, Plane plane, ref CollisionData data)
		{
			// Make sure we have contacts available
			if (data.ContactsLeft <= 0) return 0;

			// Cache the sphere position (translation from Matrix4)
			Vector3 position = new Vector3(
				sphere.Offset.data[3],  // x translation
				sphere.Offset.data[7],  // y translation
				sphere.Offset.data[11]  // z translation
			);

			// Find the distance from the plane
			float ballDistance = plane.Normal.ScalarProduct(plane.Normal, position) - sphere.Radius - plane.Offset;

			// If the sphere is not intersecting the plane (distance >= 0), no contact
			if (ballDistance >= 0) return 0;

			// Create the contact
			ParticleContact contact = data.Contacts[0];
			contact.ContactNormal = plane.Normal;
			contact.penetration = -ballDistance; // Negative distance means penetration
			contact.contactPoint = position - plane.Normal * (ballDistance + sphere.Radius);

			// Write the appropriate data
			contact.Body[0] = sphere.Body;
			contact.Body[1] = null; // No second body for half-space
			contact.Restitution = data.Restitution;
			contact.Friction = data.Friction;

			// Update the contact array and count
			data.Contacts[0] = contact;
			data.ContactsLeft--;
			return 1;
		}
		public static uint SphereAndTruePlane(Sphere sphere, Plane plane, ref CollisionData data)
		{
			// Make sure we have contacts available
			if (data.ContactsLeft <= 0) return 0;

			// Cache the sphere position (translation from Matrix4)
			Vector3 position = new Vector3(
				sphere.Offset.data[3],  // x translation
				sphere.Offset.data[7],  // y translation
				sphere.Offset.data[11]  // z translation
			);

			// Find the distance from the plane
			float centerDistance = plane.Normal.ScalarProduct(plane.Normal, position) - plane.Offset;

			// Check if we’re within radius (compare squared distances)
			if (centerDistance * centerDistance > sphere.Radius * sphere.Radius)
			{
				return 0;
			}

			// Determine which side of the plane we’re on
			Vector3 normal = plane.Normal;
			float penetration = -centerDistance;
			if (centerDistance < 0)
			{
				normal = normal * -1f; // Reverse the normal
				penetration = -penetration; // Flip penetration
			}
			penetration += sphere.Radius;

			// Create the contact
			ParticleContact contact = data.Contacts[0];
			contact.ContactNormal = normal;
			contact.penetration = penetration;
			contact.contactPoint = position - plane.Normal * centerDistance;

			// Write the appropriate data
			contact.Body[0] = sphere.Body;
			contact.Body[1] = null; // No second body for the plane
			contact.Restitution = data.Restitution;
			contact.Friction = data.Friction;

			// Update the contact array and count
			data.Contacts[0] = contact;
			data.ContactsLeft--;
			return 1;
		}
		public static uint BoxAndSphere(Box box, Sphere sphere, ref CollisionData data)
		{
			// Make sure we have contacts available
			if (data.ContactsLeft <= 0) return 0;

			// Transform the center of the sphere into box coordinates
			Vector3 center = new Vector3(
				sphere.Offset.data[3],
				sphere.Offset.data[7],
				sphere.Offset.data[11]
			);
			Vector3 relCenter = box.Offset.TransformInverse(center);

			// Early-out check using separating axes
			if (Math.Abs(relCenter.X) - sphere.Radius > box.HalfSize.X ||
				Math.Abs(relCenter.Y) - sphere.Radius > box.HalfSize.Y ||
				Math.Abs(relCenter.Z) - sphere.Radius > box.HalfSize.Z)
			{
				return 0;
			}

			// Find the closest point on the box to the sphere center
			Vector3 closestPt = new Vector3(0, 0, 0);
			float dist;

			// Clamp X coordinate
			dist = relCenter.X;
			if (dist > box.HalfSize.X) dist = box.HalfSize.X;
			if (dist < -box.HalfSize.X) dist = -box.HalfSize.X;
			closestPt.X = dist;

			// Clamp Y coordinate
			dist = relCenter.Y;
			if (dist > box.HalfSize.Y) dist = box.HalfSize.Y;
			if (dist < -box.HalfSize.Y) dist = -box.HalfSize.Y;
			closestPt.Y = dist;

			// Clamp Z coordinate
			dist = relCenter.Z;
			if (dist > box.HalfSize.Z) dist = box.HalfSize.Z;
			if (dist < -box.HalfSize.Z) dist = -box.HalfSize.Z;
			closestPt.Z = dist;

			// Check if we’re in contact
			Vector3 diff = closestPt - relCenter;
			dist = diff.X * diff.X + diff.Y * diff.Y + diff.Z * diff.Z; // Square magnitude
			if (dist > sphere.Radius * sphere.Radius) return 0;

			// Transform the closest point back to world coordinates
			Vector3 closestPtWorld = box.Offset.Multiply(closestPt);

			// Compile the contact
			ParticleContact contact = data.Contacts[0];
			Vector3 contactNormal = center - closestPtWorld;
			float distance = contactNormal.Magnitude();
			if (distance > 0) contactNormal = contactNormal * (1f / distance); // Normalize
			contact.ContactNormal = contactNormal;
			contact.contactPoint = closestPtWorld;
			contact.penetration = sphere.Radius - (float)Math.Sqrt(dist);

			// Write the appropriate data
			contact.Body[0] = box.Body;
			contact.Body[1] = sphere.Body;
			contact.Restitution = data.Restitution;
			contact.Friction = data.Friction;

			// Update the contact array and count
			data.Contacts[0] = contact;
			data.ContactsLeft--;
			return 1;
		}
		public static float TransformToAxis(Box box, Vector3 axis)
		{
			// Get the box's axes from the Offset matrix (assuming row-major)
			Vector3 axis0 = new Vector3(box.Offset.data[0], box.Offset.data[1], box.Offset.data[2]);   // Row 0: x-axis
			Vector3 axis1 = new Vector3(box.Offset.data[4], box.Offset.data[5], box.Offset.data[6]);   // Row 1: y-axis
			Vector3 axis2 = new Vector3(box.Offset.data[8], box.Offset.data[9], box.Offset.data[10]);  // Row 2: z-axis

			return
				box.HalfSize.X * Math.Abs(axis.ScalarProduct(axis, axis0)) +
				box.HalfSize.Y * Math.Abs(axis.ScalarProduct(axis, axis1)) +
				box.HalfSize.Z * Math.Abs(axis.ScalarProduct(axis, axis2));
		}
		public static uint BoxAndPoint(Box box, Vector3 point, ref CollisionData data)
		{
			// Make sure we have contacts available
			if (data.ContactsLeft <= 0) return 0;

			// Transform the point into box coordinates
			Vector3 relPt = box.Offset.TransformInverse(point);

			// Check each axis, finding the least penetration depth
			Vector3 normal;
			float minDepth = box.HalfSize.X - Math.Abs(relPt.X);
			if (minDepth < 0) return 0;
			normal = new Vector3(box.Offset.data[0], box.Offset.data[1], box.Offset.data[2]) * (relPt.X < 0 ? -1f : 1f); // x-axis

			float depth = box.HalfSize.Y - Math.Abs(relPt.Y);
			if (depth < 0) return 0;
			else if (depth < minDepth)
			{
				minDepth = depth;
				normal = new Vector3(box.Offset.data[4], box.Offset.data[5], box.Offset.data[6]) * (relPt.Y < 0 ? -1f : 1f); // y-axis
			}

			depth = box.HalfSize.Z - Math.Abs(relPt.Z);
			if (depth < 0) return 0;
			else if (depth < minDepth)
			{
				minDepth = depth;
				normal = new Vector3(box.Offset.data[8], box.Offset.data[9], box.Offset.data[10]) * (relPt.Z < 0 ? -1f : 1f); // z-axis
			}

			// Compile the contact
			ParticleContact contact = data.Contacts[0];
			contact.ContactNormal = normal;
			contact.contactPoint = point;
			contact.penetration = minDepth;

			// Write the appropriate data
			contact.Body[0] = box.Body;
			contact.Body[1] = null; // No rigid body associated with the point
			contact.Restitution = data.Restitution;
			contact.Friction = data.Friction;

			// Update the contact array and count
			data.Contacts[0] = contact;
			data.ContactsLeft--;
			return 1;
		}
		// Checks if two boxes overlap along a given axis (separating axis test)
		public static bool OverlapOnAxis(Box one, Box two, Vector3 axis)
		{
			// Project the half-size of each box onto the axis
			float oneProject = TransformToAxis(one, axis);
			float twoProject = TransformToAxis(two, axis);

			// Find the vector between the two centers
			Vector3 centerOne = new Vector3(one.Offset.data[3], one.Offset.data[7], one.Offset.data[11]);
			Vector3 centerTwo = new Vector3(two.Offset.data[3], two.Offset.data[7], two.Offset.data[11]);
			Vector3 toCenter = centerTwo - centerOne;

			// Project this onto the axis
			float distance = Math.Abs(toCenter.ScalarProduct(toCenter, axis));

			// Check for overlap (if distance is less than the sum of projections, they overlap)
			return distance < oneProject + twoProject;
		}
		public static bool GenerateVertexPlaneContact(Vector3 vertexPos, Plane plane, ref CollisionData data)
		{
			// Calculate the distance from the plane
			float vertexDistance = vertexPos.ScalarProduct(vertexPos, plane.Normal);

			// Compare this to the plane’s distance plus tolerance
			if (vertexDistance <= plane.Offset + data.Tolerance)
			{
				// Create the contact data
				ParticleContact contact = data.Contacts[0];
				contact.contactPoint = plane.Normal * (vertexDistance - plane.Offset) + vertexPos;
				contact.ContactNormal = plane.Normal;
				contact.penetration = plane.Offset - vertexDistance;

				// Update the contact array
				data.Contacts[0] = contact;
				return true;
			}
			return false;
		}
	}
}

