using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine.warppers
{
	using System;

	public static class MathF
	{
		// Constants
		public const float E = (float)Math.E;
		public const float PI = (float)Math.PI;

		// Trigonometric Functions
		public static float Acos(float value) => (float)Math.Acos(value);
		public static float Asin(float value) => (float)Math.Asin(value);
		public static float Atan(float value) => (float)Math.Atan(value);
		public static float Atan2(float y, float x) => (float)Math.Atan2(y, x);
		public static float Cos(float value) => (float)Math.Cos(value);
		public static float Sin(float value) => (float)Math.Sin(value);
		public static float Tan(float value) => (float)Math.Tan(value);

		// Exponential and Logarithmic Functions
		public static float Exp(float value) => (float)Math.Exp(value);
		public static float Log(float value) => (float)Math.Log(value);
		public static float Log10(float value) => (float)Math.Log10(value);

		// Power Functions
		public static float Pow(float value, float exponent) => (float)Math.Pow(value, exponent);
		public static float Sqrt(float value) => (float)Math.Sqrt(value);

		// Rounding Functions
		public static float Ceiling(float value) => (float)Math.Ceiling(value);
		public static float Floor(float value) => (float)Math.Floor(value);
		public static float Round(float value) => (float)Math.Round(value);

		// Other Functions
		public static float Abs(float value) => (float)Math.Abs(value);
		public static float Max(float value1, float value2) => (float)Math.Max(value1, value2);
		public static float Min(float value1, float value2) => (float)Math.Min(value1, value2);
		public static float IEEERemainder(float dividend, float divisor) => (float)Math.IEEERemainder(dividend, divisor);
		public static float Sign(float value) => (float)Math.Sign(value);
	}


}
