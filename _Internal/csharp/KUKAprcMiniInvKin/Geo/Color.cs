using System;
using System.Collections.Generic;
using System.Text;
using g = KUKAprcCoreMini.PRC_MathSharpDX;

namespace KUKAprcCoreMini.Geometry
{
    public class Color
    {
        private g.Color4 clr;


        public Color()
        {
        }

        public Color(Color clrin)
        {
            if (clrin != null)
            {
                this.A = clrin.A;
                this.R = clrin.R;
                this.G = clrin.G;
                this.B = clrin.B;
            }
            else
            {
                this.A = -1;
                this.R = 0;
                this.G = 0;
                this.B = 0;
            }
            
        }


        public static Color FromArgb(int red, int green, int blue)
        {
            Color cl = new Color();

            cl.A = 255;
            cl.R = red;
            cl.G = green;
            cl.B = blue;

            return cl;
        }

        public static Color FromArgb(int alpha, int red, int green, int blue)
        {
            Color cl = new Color();

            cl.A = alpha;
            cl.R = red;
            cl.G = green;
            cl.B = blue;

            return cl;
        }

        public int A
        {
            get
            {
                return Convert.ToInt16(clr.Alpha * 255.0f);
            }
            set
            {
                clr.Alpha = Convert.ToSingle(value) / 255.0f;
            }
        }

        public int R
        {
            get
            {
                return Convert.ToInt16(clr.Red * 255.0f);
            }
            set
            {
                clr.Red = Convert.ToSingle(value) / 255.0f;
            }
        }

        public int G
        {
            get
            {
                return Convert.ToInt16(clr.Green * 255.0f);
            }
            set
            {
                clr.Green = Convert.ToSingle(value) / 255.0f;
            }
        }

        public int B
        {
            get
            {
                return Convert.ToInt16(clr.Blue * 255.0f);
            }
            set
            {
                clr.Blue = Convert.ToSingle(value) / 255.0f;
            }
        }

        public bool IsEmpty
        {
            get
            {
                if (this.A < 0)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }

        public override string ToString()
        {
            return this.A.ToString() + "," + this.R.ToString() + "," + this.G.ToString() + "," + this.G.ToString();
        }

        public static Color Blend(Color color, Color backColor, double amount)
        {
            byte r = (byte)((color.R * amount) + backColor.R * (1 - amount));
            byte g = (byte)((color.G * amount) + backColor.G * (1 - amount));
            byte b = (byte)((color.B * amount) + backColor.B * (1 - amount));
            return Color.FromArgb(r, g, b);
        }



        public static Color Blue
        {
            get
            {
                return Color.FromArgb(255, 0, 0, 255);
            }
        }


        public static Color Gray
        {
            get
            {
                return Color.FromArgb(255, 128, 128, 128);
            }
        }

        public static Color DarkGray
        {
            get
            {
                return Color.FromArgb(255, 169, 169, 169);
            }
        }

        public static Color Orange
        {
            get
            {
                return Color.FromArgb(255, 255, 165, 0);
            }
        }

        public static Color LightYellow
        {
            get
            {
                return Color.FromArgb(255, 255, 255, 224);
            }
        }

        public static Color LightSalmon
        {
            get
            {
                return Color.FromArgb(255, 255, 160, 122);
            }
        }

        public static Color Red
        {
            get
            {
                return Color.FromArgb(255, 255, 0, 0);
            }
        }

        public static Color Green
        {
            get
            {
                return Color.FromArgb(255, 0, 255, 0);
            }
        }

        public static Color Salmon
        {
            get
            {
                return Color.FromArgb(255, 250, 128, 114);
            }
        }


    }
}
