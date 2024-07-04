using KUKAprcCoreMini.Geometry;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Xml.Linq;
using g = KUKAprcCoreMini.PRC_MathSharpDX;
using KUKAprcCoreMini.PRC_Static;

namespace KUKAprcCoreMini.Geometry
{
    public class Mesh
    {


        public List<Int4> faces;
        public List<Point3d> vertices;
        public List<Vector3d> normals;
        public Plane position;
        public Color color;
        public Color ovcolor;
        public int hash;


        public Mesh()
        {
            position = Plane.WorldXY;
            vertices = new List<Point3d>();
            faces = new List<Int4>();
            normals = new List<Vector3d>();
            color = Color.Gray;
            hash = GenerateHash.Next(32767);
        }

        public Mesh(List<Point3d> verticesin, List<Int4> facesin, List<Vector3d> normalsin)
        {
            position = Plane.WorldXY;
            vertices = verticesin;
            faces = facesin;
            normals = normalsin;
            color = Color.Gray;
            hash = GenerateHash.Next(32767);
        }

        public Mesh(List<Point3d> verticesin, List<Int4> facesin, List<Vector3d> normalsin, Color colorin)
        {
            position = Plane.WorldXY;
            vertices = verticesin;
            faces = facesin;
            normals = normalsin;
            color = colorin;
            hash = GenerateHash.Next(32767);
        }

        public void Append(Mesh mesh)
        {
            int currentcount = vertices.Count;
            this.hash += mesh.hash;

            foreach(Int4 face in mesh.faces)
            {
                faces.Add(new Int4(face.A + currentcount, face.B + currentcount, face.C + currentcount, face.D + currentcount));
            }

            if (!mesh.position.Equals(position))
            {
                KUKAprcCoreMini.Geometry.Transform torigin = KUKAprcCoreMini.Geometry.Transform.ChangeBasis(mesh.position, position);

                for (int i = 0; i < mesh.vertices.Count; i++)
                {
                    Point3d pt = new Point3d(mesh.vertices[i]);
                    pt = pt.Transform(torigin);
                    vertices.Add(pt);
                }
            }
            else
            {
                for (int i = 0; i < mesh.vertices.Count; i++)
                {
                    vertices.Add(new Point3d(mesh.vertices[i]));
                }
            }

            //foreach(Point3d vertex in mesh.vertices)
            //{
            //    vertex.Transform(torigin);
            //    vertices.Add(vertex);
            //}

            //foreach (Vector3d normal in mesh.normals)
            //{
            //    normal.Transform(torigin);
            //    normals.Add(normal);
            //}

            for (int i = 0; i < mesh.normals.Count; i++)
            {
                //mesh.normals[i].Transform(torigin);
                normals.Add(new Vector3d(mesh.normals[i].X, mesh.normals[i].Y, mesh.normals[i].Z));
            }

        }

        public Mesh DuplicateMesh()
        {
            List<Int4> facelist = new List<Int4>();
            foreach(Int4 face in faces)
            {
                facelist.Add(new Int4(face.A, face.B, face.C, face.D));
            }

            List<Point3d> vertexlist = new List<Point3d>();
            foreach (Point3d vertex in vertices)
            {
                vertexlist.Add(new Point3d(vertex.X, vertex.Y, vertex.Z));
            }

            List<Vector3d> normallist = new List<Vector3d>();
            foreach (Vector3d normal in normals)
            {
                normallist.Add(new Vector3d(normal.X, normal.Y, normal.Z));
            }

            Mesh tmp = new Mesh();
            tmp.position = new Plane(position);
            tmp.vertices = vertexlist;
            tmp.normals = normallist;
            tmp.faces = facelist;
            tmp.color = Color.FromArgb(color.A, color.R, color.G, color.B);
            tmp.hash = hash;

            return tmp;
        }

        public Mesh SoftDuplicateMesh()
        {

            Mesh tmp = new Mesh();
            tmp.position = new Plane(position);
            tmp.vertices = vertices;
            tmp.normals = normals;
            tmp.faces = faces;
            tmp.color = Color.FromArgb(color.A, color.R, color.G, color.B);
            tmp.hash = hash;

            return tmp;
        }


        public static Mesh SingleFace(double size)
        {
            List<Int4> facelist = new List<Int4>();
            facelist.Add(new Int4(0, 1, 2, 3));


            List<Point3d> vertexlist = new List<Point3d>();
            vertexlist.Add(new Point3d(-size, -size, 0));
            vertexlist.Add(new Point3d(-size, size, 0));
            vertexlist.Add(new Point3d(size, size, 0));
            vertexlist.Add(new Point3d(size, -size, 0));

            List<Vector3d> normallist = new List<Vector3d>();
            foreach (Point3d pt in vertexlist)
            {
                normallist.Add(new Vector3d(0,0,1));
            }

            Mesh tmp = new Mesh();
            tmp.position = Plane.WorldXY;
            tmp.vertices = vertexlist;
            tmp.normals = normallist;
            tmp.faces = facelist;
            tmp.color = Color.FromArgb(0,0,0,0);
            tmp.hash = GenerateHash.Next(32767);

            return tmp;
        }


        public void Bake()
        {
            if (!position.Equals(Plane.WorldXY))
            {
                KUKAprcCoreMini.Geometry.Transform tonewpos = KUKAprcCoreMini.Geometry.Transform.PlaneToPlane(Plane.WorldXY, position);

                for (int i = 0; i < vertices.Count; i++)
                {
                    vertices[i] = vertices[i].Transform(tonewpos);
                }

                //for (int i = 0; i < normals.Count; i++)
                //{
                //    normals[i].Unitize();
                //    normals[i].Reverse();
                //    normals[i].Transform(tonewpos);
                //}
            }

            position = Plane.WorldXY;
        }

        public int DisjointMeshCount
        {
            get
            {
                return 1;
            }
        }

        public static Mesh SetOVColor (Mesh mesh, Color clr)
        {
            mesh.ovcolor = clr;
            return mesh;
        }

        public static List<Mesh> SetOVColor (List<Mesh> meshes, Color clr)
        {
            foreach (Mesh mesh in meshes)
            {
                mesh.ovcolor = clr;
            }
            return meshes;
        }

        public void Transform(g.Matrix matrix)
        {
            Plane tmp = new Plane(position);
            tmp.Transform(matrix);
            hash = GenerateHash.Next(32767);

            position = tmp;
        }

        public void Scale(Vector3d factor)
        {
            for (int i = 0; i < this.vertices.Count; i++)
            {
                vertices[i] = new Point3d(vertices[i].X * factor.X, vertices[i].Y * factor.Y, vertices[i].Z * factor.Z);
            }
        }

        /// <inheritdoc/>
        public override bool Equals(object obj)
        {
            if (ReferenceEquals(null, obj)) return false;
            if (obj.GetType() != typeof(Mesh)) return false;
            return Equals((Mesh)obj);
        }

        /// <inheritdoc/>
        public override int GetHashCode()
        {
            unchecked
            {
                return 0;
            }
        }


        public override string ToString()
        {
            return "mesh";
        }

        //public XElement ToXML(string name = "mesh")
        //{
        //    XElement mesh =
        //       new XElement(name,
        //           new XElement("position",
        //                new XAttribute("Ox", position.OriginX),
        //                new XAttribute("Oy", position.OriginX),
        //                new XAttribute("Oz", position.OriginX),
        //                new XAttribute("Xx", position.XAxis.X),
        //                new XAttribute("Xy", position.XAxis.Y),
        //                new XAttribute("Xz", position.XAxis.Z),
        //                new XAttribute("Yx", position.YAxis.X),
        //                new XAttribute("Yy", position.YAxis.Y),
        //                new XAttribute("Yz", position.YAxis.Z)),
        //           new XElement("color",
        //                new XAttribute("A", color.A),
        //                new XAttribute("R", color.R),
        //                new XAttribute("G", color.G),
        //                new XAttribute("B", color.B))
        //   );

        //    XElement xmlvertices = new XElement("vertices");
        //    foreach (Point3d pt in vertices)
        //    {
        //        xmlvertices.Add(pt.ToXML());
        //    }
        //    mesh.Add(xmlvertices);

        //    XElement xmlnormals = new XElement("normals");
        //    foreach (Vector3d pt in normals)
        //    {
        //        xmlnormals.Add(pt.ToXML());
        //    }
        //    mesh.Add(xmlnormals);

        //    XElement xmlfaces = new XElement("faces");
        //    foreach (Int4 int3 in faces)
        //    {
        //        xmlfaces.Add(int3.ToXML());
        //    }
        //    mesh.Add(xmlfaces);

        //    return mesh;
        //}



    }

    

    public class PRC_MeshCollection
    {
        public List<Mesh> highresmesh;
        public List<Mesh> lowresmesh;
        public Plane position;
        public string name;
        public int hash;

        public PRC_MeshCollection()
        {
            highresmesh = new List<Mesh>();
            lowresmesh = new List<Mesh>();
            position = Plane.WorldXY;
            hash = GenerateHash.Next(32767);
            name = hash.ToString();
        }

        public PRC_MeshCollection(Mesh singlemesh)
        {
            highresmesh = new List<Mesh> { singlemesh };
            lowresmesh = new List<Mesh> { singlemesh };
            position = Plane.WorldXY;
            hash = GenerateHash.Next(32767);
            
        }

        public PRC_MeshCollection(List<Mesh> highresmeshin, List<Mesh> lowresmeshin)
        {
            highresmesh = new List<Mesh>();
            lowresmesh = new List<Mesh>();
            hash = 0;
            foreach (Mesh highmesh in highresmeshin)
            {
                highresmesh.Add(highmesh);
                hash += highmesh.hash;
            }

            foreach (Mesh lowmesh in lowresmeshin)
            {
                lowresmesh.Add(lowmesh);
                hash += lowmesh.hash;
            }

            position = Plane.WorldXY;
            name = hash.ToString();
        }

        public PRC_MeshCollection(List<Mesh> highresmeshin, List<Mesh> lowresmeshin, string meshname)
        {
            hash = GenerateHash.Next(32767);
            highresmesh = new List<Mesh>();
            lowresmesh = new List<Mesh>();
            foreach (Mesh highmesh in highresmeshin)
            {
                highresmesh.Add(highmesh);
            }

            foreach (Mesh lowmesh in lowresmeshin)
            {
                lowresmesh.Add(lowmesh);
            }

            position = Plane.WorldXY;
            name = meshname;
        }

        public void Transform(PRC_MathSharpDX.Matrix matrix)
        {
            Plane tmp = new Plane(position);
            tmp.Transform(matrix);
            hash = GenerateHash.Next(32767);

            position = tmp;
        }

        public void Scale (Vector3d factor)
        {
            foreach (Mesh mesh in this.lowresmesh)
            {
                mesh.Scale(factor);
            }

            foreach (Mesh mesh in this.highresmesh)
            {
                mesh.Scale(factor);
            }
        }

        public List<Mesh> GetHighResMesh()
        {
            List<Mesh> outmesh = new List<Mesh>();

            List<Mesh> processmesh = new List<Mesh>();

            if (lowresmesh.Count > highresmesh.Count)
            {
                processmesh = lowresmesh;
            }
            else
            {
                processmesh = highresmesh;
            }

            foreach (Mesh mesh in processmesh)
            {
                mesh.position = position;
                outmesh.Add(mesh);
            }
            return outmesh;
        }

        public List<Mesh> GetLowResMesh()
        {
            List<Mesh> outmesh = new List<Mesh>();
            List<Mesh> processmesh = new List<Mesh>();

            if (highresmesh.Count > lowresmesh.Count)
            {
                processmesh = highresmesh;
            }
            else
            {
                processmesh = lowresmesh;
            }

            foreach (Mesh mesh in processmesh)
            {
                mesh.position = position;
                outmesh.Add(mesh);
            }
            return outmesh;
        }

        public void Add(Mesh mesh, bool ishighres)
        {
            if (!mesh.position.Equals(position))
            {
                KUKAprcCoreMini.Geometry.Transform torigin = KUKAprcCoreMini.Geometry.Transform.ChangeBasis(mesh.position, position);
                for (int i = 0; i < mesh.vertices.Count; i++)
                {
                    mesh.vertices[i] = mesh.vertices[i].Transform(torigin);
                }

                mesh.position = position;
            }

            if (ishighres)
            {
                highresmesh.Add(mesh);
            }
            else
            {
                lowresmesh.Add(mesh);
            }
        }

        public void Bake()
        {
            KUKAprcCoreMini.Geometry.Transform torigin = KUKAprcCoreMini.Geometry.Transform.ChangeBasis(position, Plane.WorldXY);

            for (int i = 0; i < highresmesh.Count; i++)
            {
                highresmesh[i].Transform(torigin);
                highresmesh[i].Bake();
            }


            for (int i = 0; i < lowresmesh.Count; i++)
            {
                lowresmesh[i].Transform(torigin);
                lowresmesh[i].Bake();
            }

            position = Plane.WorldXY;
        }

        public void Append(PRC_MeshCollection meshcoll)
        {
            if (!meshcoll.position.Equals(position))
            {
                KUKAprcCoreMini.Geometry.Transform torigin = KUKAprcCoreMini.Geometry.Transform.ChangeBasis(meshcoll.position, position);

                for (int i = 0; i < meshcoll.highresmesh.Count; i++)
                {
                    meshcoll.highresmesh[i].Transform(torigin);
                    highresmesh.Add(meshcoll.highresmesh[i]);
                }

                for (int i = 0; i < meshcoll.lowresmesh.Count; i++)
                {
                    meshcoll.lowresmesh[i].Transform(torigin);
                    lowresmesh.Add(meshcoll.lowresmesh[i]);
                }

            }
            else
            {
                for (int i = 0; i < meshcoll.highresmesh.Count; i++)
                {
                    highresmesh.Add(meshcoll.highresmesh[i]);
                }

                for (int i = 0; i < meshcoll.lowresmesh.Count; i++)
                {
                    lowresmesh.Add(meshcoll.lowresmesh[i]);
                }
            }

        }

        public void Append (Mesh mesh)
        {

            if (!mesh.position.Equals(position))
            {
                KUKAprcCoreMini.Geometry.Transform torigin = KUKAprcCoreMini.Geometry.Transform.ChangeBasis(mesh.position, position);

                for (int i = 0; i < mesh.vertices.Count; i++)
                {
                    mesh.vertices[i] = mesh.vertices[i].Transform(torigin);
                }
            }

            this.Add(mesh, true);
            this.Add(mesh, false);
        }

        public void SetOVColor (Color clr)
        {
            Mesh.SetOVColor(this.highresmesh, clr);
            Mesh.SetOVColor(this.lowresmesh, clr);
        }


        public PRC_MeshCollection DuplicateMesh()
        {
            PRC_MeshCollection meshcoll = new PRC_MeshCollection();

            foreach (Mesh highmesh in this.highresmesh)
            {
                meshcoll.highresmesh.Add(highmesh.DuplicateMesh());
            }

            foreach (Mesh lowmesh in this.lowresmesh)
            {
                meshcoll.lowresmesh.Add(lowmesh.DuplicateMesh());
            }


            meshcoll.name = this.name;

            meshcoll.position = new Plane(this.position);

            meshcoll.hash = this.hash;

            return meshcoll;
        }

        public PRC_MeshCollection SoftDuplicateMesh()
        {
            PRC_MeshCollection meshcoll = new PRC_MeshCollection();

            foreach (Mesh highmesh in this.highresmesh)
            {
                meshcoll.highresmesh.Add(highmesh.SoftDuplicateMesh());
            }

            foreach (Mesh lowmesh in this.lowresmesh)
            {
                meshcoll.lowresmesh.Add(lowmesh.SoftDuplicateMesh());
            }

            

            meshcoll.name = this.name;

            meshcoll.position = new Plane(this.position);

            meshcoll.hash = this.hash;

            return meshcoll;
        }


    }

        //public XElement ToXML(string name = "MeshCollection")
        //{
        //    XElement meshcollection =
        //       new XElement(name,
        //           new XElement("position",
        //                new XAttribute("Ox", position.OriginX),
        //                new XAttribute("Oy", position.OriginX),
        //                new XAttribute("Oz", position.OriginX),
        //                new XAttribute("Xx", position.XAxis.X),
        //                new XAttribute("Xy", position.XAxis.Y),
        //                new XAttribute("Xz", position.XAxis.Z),
        //                new XAttribute("Yx", position.YAxis.X),
        //                new XAttribute("Yy", position.YAxis.Y),
        //                new XAttribute("Yz", position.YAxis.Z))
        //    );

        //    foreach (Mesh mesh in meshes)
        //    {
        //        meshcollection.Add(mesh.ToXML());
        //    }

        //    return meshcollection;
        //}

        //public static implicit operator PRC_MeshCollection(List<Mesh> value)
        //{
        //    if (value == null | value.Count < 1)
        //    {
        //        return new PRC_MeshCollection();
        //    }
        //    else
        //    {

        //        PRC_MeshCollection meshcollection = new PRC_MeshCollection();
        //        meshcollection.position = new Plane(value[0].position);

        //        foreach (Mesh mesh in value)
        //        {
        //            meshcollection.Add(mesh);
        //        }

        //        return meshcollection;

        //    }
        //}


    [Serializable]
    public class PRC_SerializableMeshCollection
    {
        public List<string> highresmeshes;
        public List<string> lowresmeshes;
        public Plane position;
        public string name;

        public PRC_SerializableMeshCollection()
        {

        }






    }

    [Serializable]
    public class SerializableMesh
    {
        public List<int> facesA = new List<int>();
        public List<int> facesB = new List<int>();
        public List<int> facesC = new List<int>();
        public List<int> facesD = new List<int>();
        public List<float> verticesX = new List<float>();
        public List<float> verticesY = new List<float>();
        public List<float> verticesZ = new List<float>();
        public List<float> normalsX = new List<float>();
        public List<float> normalsY = new List<float>();
        public List<float> normalsZ = new List<float>();
        public int colorA = 255;
        public int colorR = 127;
        public int colorG = 127;
        public int colorB = 127;

        public SerializableMesh()
        {

        }

        public SerializableMesh(Mesh fromMesh)
        {
            fromMesh.Bake();

            facesA = new List<int>();
            facesB = new List<int>();
            facesC = new List<int>();
            facesD = new List<int>();
            foreach (Int4 face in fromMesh.faces)
            {
                facesA.Add(face.A);
                facesB.Add(face.B);
                facesC.Add(face.C);
                facesD.Add(face.D);
            }

            verticesX = new List<float>();
            verticesY = new List<float>();
            verticesZ = new List<float>();
            foreach (Point3d vertex in fromMesh.vertices)
            {
                verticesX.Add(vertex.X);
                verticesY.Add(vertex.Y);
                verticesZ.Add(vertex.Z);
            }

            normalsX = new List<float>();
            normalsY = new List<float>();
            normalsZ = new List<float>();
            foreach (Vector3d normal in fromMesh.normals)
            {
                normalsX.Add(normal.X);
                normalsY.Add(normal.Y);
                normalsZ.Add(normal.Z);
            }

            colorA = fromMesh.color.A;
            colorR = fromMesh.color.R;
            colorG = fromMesh.color.G;
            colorB = fromMesh.color.B;
        }


        //public static byte[] BrotliCompress(byte[] bytes)
        //{
        //    return Brotli.CompressBuffer(bytes, 0, bytes.Length);
        //}

        //public static byte[] BrotliDeCompress(byte[] bytes)
        //{
        //    return Brotli.DecompressBuffer(bytes, 0, bytes.Length);
        //}

        public Mesh ToMesh()
        {
            List<Int4> faces = new List<Int4>();
            List<Point3d> vertices = new List<Point3d>();
            List<Vector3d> normals = new List<Vector3d>();
            Color color = Color.Gray;

            for (int i = 0; i < facesA.Count; i++)
            {
                faces.Add(new Int4(facesA[i], facesB[i], facesC[i], facesD[i]));
            }

            for (int i = 0; i < verticesX.Count; i++)
            {
                vertices.Add(new Point3d(verticesX[i], verticesY[i], verticesZ[i]));
            }

            for (int i = 0; i < normalsX.Count; i++)
            {
                normals.Add(new Vector3d(normalsX[i], normalsY[i], normalsZ[i]));
            }

            color.A = colorA;
            color.R = colorR;
            color.G = colorG;
            color.B = colorB;

            Mesh mesh = new Mesh(vertices, faces, normals, color);

            return mesh;
        }

    }
}
