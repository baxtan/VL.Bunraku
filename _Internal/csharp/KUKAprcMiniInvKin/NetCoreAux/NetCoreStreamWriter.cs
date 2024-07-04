using System;
using System.Collections.Generic;
using System.Text;

namespace KUKAprcCoreMini.NetCoreAux.IO
{
    public class StreamWriter : System.IO.StreamWriter
    {
        public StreamWriter(string path, bool boolval) :
         base(CreateStream(path, boolval))
        {

        }

        private static System.IO.Stream CreateStream(string path, bool boolval)
        {
            System.IO.FileStream filestream = new System.IO.FileStream(path, System.IO.FileMode.Create);
            return filestream;
        }

#if net451
        public override void Close()
        {
            this.Dispose();
        }
#elif net46
        public override void Close()
        {
            this.Dispose();
        }
#endif
    }
}
