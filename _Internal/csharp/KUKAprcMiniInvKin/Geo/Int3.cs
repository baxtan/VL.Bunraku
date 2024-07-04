using System;
using System.Collections.Generic;
using System.Text;
using System.Xml.Linq;
using g = KUKAprcCoreMini.PRC_MathSharpDX;

namespace KUKAprcCoreMini.Geometry
{
    public class Int3
    {
        /// <summary>
        /// The X component of the vector.
        /// </summary>
        public int X;

        /// <summary>
        /// The Y component of the vector.
        /// </summary>
        public int Y;

        /// <summary>
        /// The Z component of the vector.
        /// </summary>
        public int Z;

        /// <summary>
        /// Initializes a new instance of the <see cref = "Int3" /> struct.
        /// </summary>
        /// <param name = "value">The value that will be assigned to all components.</param>
        public Int3(int value)
        {
            X = value;
            Y = value;
            Z = value;
        }

        /// <summary>
        /// Initializes a new instance of the <see cref = "Int3" /> struct.
        /// </summary>
        /// <param name = "x">Initial value for the X component of the vector.</param>
        /// <param name = "y">Initial value for the Y component of the vector.</param>
        /// <param name = "z">Initial value for the Z component of the vector.</param>
        public Int3(int x, int y, int z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        //public XElement ToXML()
        //{
        //    XElement int3 =
        //       new XElement("int3",
        //            new XAttribute("X", X),
        //            new XAttribute("Y", Y),
        //            new XAttribute("Z", Z)
        //   );
        //    return int3;
        //}
    }
}
