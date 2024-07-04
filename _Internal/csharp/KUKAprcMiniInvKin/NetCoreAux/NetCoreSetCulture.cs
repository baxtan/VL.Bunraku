using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace KUKAprcCoreMini.NetCoreAux.Threading
{
    public static class Thread
    {
        public static class CurrentThread
        {
            public static CultureInfo CurrentCulture
            {
                set
                {
#if NETSTANDARD
                    CultureInfo.CurrentCulture = value;
#elif net451
                    System.Threading.Thread.CurrentThread.CurrentCulture = value;
#elif net46
                    System.Threading.Thread.CurrentThread.CurrentCulture = value;
#endif
                }
            }
            public static CultureInfo CurrentUICulture
            {
                set
                {
#if NETSTANDARD
                    CultureInfo.CurrentCulture = value;
#elif net451
                    System.Threading.Thread.CurrentThread.CurrentUICulture = value;
#elif net46
                    System.Threading.Thread.CurrentThread.CurrentUICulture = value;
#endif
                }
            }
        }
    }
}
