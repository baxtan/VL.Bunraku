using KUKAprcCoreMini.PRC_MathSharpDX;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Xml.Linq;
using g = KUKAprcCoreMini.PRC_MathSharpDX;

namespace KUKAprcCoreMini.Geometry
{
   /* public class MeshCollection
    {
        public List<Mesh> meshes { get; }
        public Plane position;

        public MeshCollection()
        {
            meshes = new List<Mesh>();
            position = Plane.WorldXY;
        }

        public MeshCollection(List<Mesh> mesh)
        {
            meshes = mesh;
            position = Plane.WorldXY;
        }

        public void Transform(g.Matrix matrix)
        {
            Plane tmp = new Plane(position);
            tmp.Transform(matrix);

            position = tmp;
        }

        public void Add(Mesh mesh)
        {
            KUKAprcCore.Geometry.Transform torigin = KUKAprcCore.Geometry.Transform.ChangeBasis(mesh.position, position);
            for (int i = 0; i < mesh.vertices.Count; i++)
            {
                mesh.vertices[i] = mesh.vertices[i].Transform(torigin);
            }
            mesh.position = position;

            meshes.Add(mesh);
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

        public static implicit operator MeshCollection(List<Mesh> value)
        {
            if (value == null | value.Count < 1)
            {
                return new MeshCollection();
            }
            else
            {
                
                MeshCollection meshcollection = new MeshCollection();
                meshcollection.position = new Plane(value[0].position);

                foreach (Mesh mesh in value)
                {
                    meshcollection.Add(mesh);
                }

                return meshcollection;

            }
        }
    }

    [Serializable]
    public class SerializableMeshCollection
    {
        List<string> meshes;
        Plane position;

        public SerializableMeshCollection()
        {

        }

        public SerializableMeshCollection(MeshCollection fromMesh)
        {
            position = new Plane(fromMesh.position);

            meshes = new List<string>();
            foreach (Mesh mesh in fromMesh.meshes)
            {
                SerializableMesh sermesh = new SerializableMesh(mesh);
                meshes.Add(sermesh.Serialize());
            }
        }

        public MeshCollection ExtractMeshCollection()
        {
            List <Mesh> meshlist = new List<Mesh>();

            foreach (string item in meshes)
            {
                meshlist.Add(SerializableMesh.DeSerialize(item));
            }

            MeshCollection meshcoll = new MeshCollection(meshlist);
            meshcoll.position = new Plane(position);

            return meshcoll;
        }

        public string Serialize()
        {
            string output = JsonConvert.SerializeObject(this);

            return output;
        }

        public static MeshCollection DeSerialize(string data)
        {
            SerializableMeshCollection deserializedProduct = JsonConvert.DeserializeObject<SerializableMeshCollection>(data);
            return deserializedProduct.ExtractMeshCollection();
        }

    }
    */

    }
