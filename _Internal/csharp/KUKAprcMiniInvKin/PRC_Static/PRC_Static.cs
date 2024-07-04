using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Microsoft.Win32;
using System.Security.Cryptography;
using System.Globalization;
using System.Collections.Concurrent;
using System.Runtime.InteropServices;

using System.Net.Http;
using System.Threading.Tasks;


namespace KUKAprcCoreMini.PRC_Static
{
    //public static class GenerateHash
    //{
    //    public static Random rnd = new Random();
    //}

    public static class GenerateHash
    {
        private static Random _global = new Random();
        [ThreadStatic]
        private static Random _local;

        public static int Next(int max)
        {
            Random inst = _local;
            if (inst == null)
            {
                int seed;
                lock (_global) seed = _global.Next(max);
                _local = inst = new Random(seed);
            }
            return inst.Next(max);
        }
    }

    public static class MassCustomCounter
    {
        private static int int_counter = -1;

        public static int GlobalCounter
        {
            get { return int_counter; }
            set { int_counter = value; }
        }
    }


}
