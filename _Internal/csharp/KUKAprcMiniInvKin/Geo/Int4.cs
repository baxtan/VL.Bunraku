using System;
using System.Collections.Generic;
using System.Text;
using System.Xml.Linq;
using g = KUKAprcCoreMini.PRC_MathSharpDX;

namespace KUKAprcCoreMini.Geometry
{
    public class Int4
    {
        /// <summary>
        /// The A component of the vector.
        /// </summary>
        public int A;

        /// <summary>
        /// The B component of the vector.
        /// </summary>
        public int B;

        /// <summary>
        /// The C component of the vector.
        /// </summary>
        public int C;

        /// <summary>
        /// The D component of the vector.
        /// </summary>
        public int D;

        /// <summary>
        /// Initializes a new instance of the <see cref = "Int4" /> struct.
        /// </summary>
        /// <param name = "value">The value that will be assigned to all components.</param>
        public Int4(int value)
        {
            A = value;
            B = value;
            C = value;
            D = value;
        }

        /// <summary>
        /// Initializes a new instance of the <see cref = "Int4" /> struct.
        /// </summary>
        /// <param name = "a">Initial value for the A component of the vector.</param>
        /// <param name = "b">Initial value for the B component of the vector.</param>
        /// <param name = "c">Initial value for the C component of the vector.</param>
        /// <param name = "d">Initial value for the D component of the vector.</param>
        public Int4(int a, int b, int c, int d)
        {
            A = a;
            B = b;
            C = c;
            D = d;
        }

        /// <summary>
        /// Initializes a new instance of the <see cref = "Int4" /> struct.
        /// </summary>
        /// <param name = "a">Initial value for the A component of the vector.</param>
        /// <param name = "b">Initial value for the B component of the vector.</param>
        /// <param name = "c">Initial value for the C component of the vector.</param>
        /// <param name = "d">Initial value for the D component of the vector.</param>
        public Int4()
        {
            A = 0;
            B = 0;
            C = 0;
            D = 0;
        }
    }
}
