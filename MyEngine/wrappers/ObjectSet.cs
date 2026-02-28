using System;
using System.Collections.Generic;
using System.Text;

namespace MyEngine.warppers
{
	public class ObjectSet<T>
	{
		public List<T> Items { get; set; }

		public ObjectSet()
		{
			Items = new List<T>();
		}
	}
}
