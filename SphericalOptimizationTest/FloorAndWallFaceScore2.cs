using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using OptimizationToolbox;
using TVGL;

namespace SphericalOptimizationTest
{
    internal class FloorAndWallFaceScore2 : IObjectiveFunction
    {
        private double thickness;
        private const double maxAngle = Math.PI * 3 / 4; 
        //max overhanging angle Pi*3/4 , 135 degree from printing bed normal direction,
        private const double ToolheadAngle = Math.PI / 2; 
        // toolhead is assumed to be as a cone-shape with angle Pi/2, 90 degree,

        private readonly TessellatedSolid tessellatedSolid;
        public TessellatedSolid bestTessellatedSolid { get; private set; }
        private double bestObjFunc = double.PositiveInfinity;
        private const double b = 50; //practical values between 5 to 100
        private static double denomProx = 1 + Math.Exp(-b);


        public FloorAndWallFaceScore2(TessellatedSolid tessellatedSolid, double thickness)
        {
            this.tessellatedSolid = tessellatedSolid;
            this.thickness = thickness;
        }

        public double calculate(double[] x)
        {
            Vector3 d = new Vector3(x).Normalize();
            //Console.WriteLine("Current Direction: " + ((decimal)d.X) + " , " + ((decimal)d.Y) + " , " + ((decimal)d.Z));
            //Vector3 d = MakeUnitVectorFromSpherical(x[0], x[1]);

            var w1 = 0.5; var w2 = 0.5; var w3 = 0.2;
            var x1 = SupportStructureScore(d);
            var x2 = 0.0;
            var test = GetFirstLayer(d);
            //Presenter.ShowAndHang(test);
            /*foreach (var face in tessellatedSolid.Faces)
            {
                var dot = d.Dot(face.Normal);
                x2 += face.Area * StaircaseEffectScore(dot);
            }
            */
            var FirstLayer = FirstLayerScore(test);
            double x3 = 0.0;
            double total = w1 * x1 + w2 * FirstLayer + w3 * x3;
            //Console.WriteLine("Total Cost: " + ((decimal)total));
            
            return total;
        }
        private List<Polygon> GetFirstLayer(Vector3 direction)
        {
            double offset = 1E-6;
            var transform = direction.TransformToXYPlane(out _);
            var plane = new Plane(0.0, direction);
            //First, sort the vertices along the given axis. Duplicate distances are not important.
            var sortedVertices = tessellatedSolid.Vertices.OrderBy(v => v.Dot(direction)).ToArray();
            var firstDistance = sortedVertices[0].Dot(direction);
            var currentEdges = new HashSet<Edge>();
            var vIndex = 0;
            var d = firstDistance + offset * thickness;
            var thisVertex = sortedVertices[vIndex];
            var needToOffset = false;
            while (thisVertex.Dot(direction) <= d)
            {
                if (d.IsPracticallySame(thisVertex.Dot(direction))) needToOffset = true;
                foreach (var edge in thisVertex.Edges)
                {
                    if (currentEdges.Contains(edge)) currentEdges.Remove(edge);
                    else currentEdges.Add(edge);
                }
                vIndex++;
                if (vIndex == sortedVertices.Length)
                    break;
                thisVertex = sortedVertices[vIndex];
            }
            if (needToOffset)
                d += Math.Min(thickness, sortedVertices[vIndex].Dot(direction) - d) / 10.0;
            plane.DistanceToOrigin = d;
            if (currentEdges.Any())
                return Loops.GetLoops(currentEdges.ToDictionary(ce => ce, ce =>
                   MiscFunctions.PointOnPlaneFromIntersectingLine(plane, ce.From.Coordinates, ce.To.Coordinates, out _)
                       .ConvertTo2DCoordinates(transform)), plane.Normal, plane.DistanceToOrigin, out _);
            else
            {
                Loops.output("Error First Layer: Consider increasing value of offset ");
                return null;
            }
                #region

                /*
                var sortedVertices = tessellatedSolid.Vertices.OrderBy(v => v.Dot(d)).ToArray();
                var ListOfVertices = new List<Vertex>();
                foreach (var v in sortedVertices)
                    if (sortedVertices[0].Dot(d) == v.Dot(d))
                        ListOfVertices.Add(v);
                    else
                        break;
                // The first layer stands on the printing bed on:
                // 1. vertices,
                // 2. edges,
                // 3. faces,
                // 4. combination of the above cases

                // in the case of standing on a vertex, and edge,
                // on an edge, the first layer is a line, but the thickness and surface curvature can influnce the decision
                // Computationally, we can adjust the first layer thickness to have a small area.
                // We can get small area by pushing the layer along the printing direction 1% of the printing thickness.
                // Possible issues: the first layer is too small to be printed,
                //                  example a single line with the printing thickness is to fit in the first layer
                // check the surface curvature at a vertex, or edge 

                // in the case of standing on faces, return the border edges  

                if (ListOfVertices.Count == 1) // a single vertex
                {

                }
                else if (ListOfVertices.Count == 2)
                {
                    if (ListOfVertices[0].Edges.Intersect(ListOfVertices[1].Edges).Count() >= 0) // single edge
                    {
                    }
                    else // two vertices
                    {

                    }
                }

                else
                {
                    Dictionary<Vertex, List<Vertex>> vertices = new Dictionary<Vertex, List<Vertex>>();
                    foreach (var v in ListOfVertices)
                        vertices.Add(v, new List<Vertex>());
                    foreach (var v_i in ListOfVertices)
                    {
                        foreach (var v_j in ListOfVertices)
                        {
                            if (v_i != v_j)
                            {
                                if (v_i.Edges.Intersect(v_j.Edges).Count() > 0)
                                {
                                    vertices[v_i].Add(v_j);
                                }
                            }
                        }
                    }
                    var Case1 = new List<Vertex>();
                    var Case2 = new List<(Vertex, Vertex)>();
                    foreach(var v in vertices)
                    {
                        if (v.Value.Count() == 1)
                            Case1.Add(v.Key);
                        else if (v.Value.Count() >= 2)
                        {
                            for (int i = 1; i < v.Value.Count(); i++)
                                if (!Case2.Contains((v.Value[0], v.Value[i])) || !Case2.Contains((v.Value[i], v.Value[0]))) 
                                    Case2.Add((v.Value[0], v.Value[i]));
                        }
                    }
                }*/
                #endregion
        }
        
        private double FirstLayerScore(List<Polygon> FirstLayer)
        {
            return 1E-3*tessellatedSolid.SurfaceArea / FirstLayer.Sum(x => x.Area);

        }
        private double StaircaseEffectScore(double dot)
        {
            return 1 - (Math.Exp(b * (Math.Abs(dot) - 1)) + Math.Exp(-b * Math.Abs(dot))) / denomProx;
        }

        private double SupportStructureScore(Vector3 d, double currentBest = double.PositiveInfinity)
        {
            var SolidOriginal = TVGL.Slice.GetUniformlySpacedCrossSections(
                tessellatedSolid, d, double.NaN, -1, 0.5 * thickness).ToList();
            SolidOriginal.Insert(0, GetFirstLayer(d));
            var solid = CrossSectionSolid.CreateFromTessellatedSolid(tessellatedSolid, d, SolidOriginal.Count());
            var css_i = new List<Polygon>[(int)(SolidOriginal.Count() / 2)];
            css_i.ToList();
            for (int i = 0; i < css_i.Count(); i++)
            {
                css_i[i] = SolidOriginal[i * 2];
            }
            /*var verts = new List<List<Vector3>>();
            for (int i = 0; i < SolidOriginal.Count(); i++) 
            {
                foreach(var j in SolidOriginal[i])
                {
                    var current_verts = new List<Vector3>();

                    foreach (var v in j.Vertices)
                    {
                        current_verts.Add(new Vector3(v.X, v.Y, i * 0.5 * thickness));
                    }
                    verts.Add(current_verts);

                }
            }
            Presenter.ShowVertexPaths(verts, null, 0.5);
            Presenter.ShowAndHang(solid);*/
            var Area = 0.0;
            var Area2 = 0.0;
            for (int i = 1; i < css_i.Count(); i++)
            {
                var temp_css = new List<Polygon>();
                if (css_i[i - 1] is null)
                    css_i[i]=null;
                else
                {
                    for (int j = 0; j < css_i[i].Count(); j++) 
                    {
                        for (int k = 0; k < css_i[i - 1].Count(); k++) 
                        {
                            var interaction = css_i[i][j].GetPolygonInteraction(css_i[i - 1][k]);
                            if (!interaction.IntersectionWillBeEmpty())
                            {
                                temp_css.Add(css_i[i][j]);
                                break;
                            }
                        }
                    }
                }
                Area2+= temp_css.Select(x => x.Area).Sum();
                //if (Area2 > 0 & Area2 < currentBest)
                 //   return double.PositiveInfinity;
                css_i[i] = temp_css;
                if (i != css_i.Count() - 1)
                {
                    //Presenter.ShowAndHang(css_i[i]);
                    //Presenter.ShowAndHang(SolidOriginal[(i*2)+1]);

                    var subrtractPoly = PolygonOperations.Subtract(css_i[i], SolidOriginal[(i * 2) + 1]);
                    if(subrtractPoly.Count()>0)
                    {
                       // Presenter.ShowAndHang(subrtractPoly);
                    }
                    Area += subrtractPoly.Sum(x => x.Area);
                }
                else if ((double)SolidOriginal.Count() / (double)css_i.Count() != 2)
                    Area += SolidOriginal.Last().Sum(x => x.Area);
                
            }

            /*
            var newcss = new List<CrossSectionSolid>();
            var newcss2 = new List<Solid>();
            for (int i = 0; i < css_i.Count(); i++)
            {
                newcss.Add(TVGL.CrossSectionSolid.CreateConstantCrossSectionSolid(d, i*thickness, thickness, css_i[i], 1e-10, UnitType.unspecified));
                newcss2.Add(newcss[i].ConvertToTessellatedExtrusions(false, false));
            }*/
            //Presenter.ShowAndHang(newcss2);
            return Area * thickness;
        }


            private Vector3 MakeUnitVectorFromSpherical(double inclinationAngle, double azimuthalAngle)
        {
            var sinInclination = Math.Sin(inclinationAngle);
            var x = Math.Cos(azimuthalAngle) * sinInclination;
            var y = Math.Sin(azimuthalAngle) * sinInclination;
            var z = Math.Cos(inclinationAngle);
            return (new Vector3(x, y, z)).Normalize();
        }
    }

    public class ShowBestShape
    {
        public static List<Solid> ShowShape(Vector3 d, TessellatedSolid ts, double thickness)
        {
            // printing bed is assumed to change in dimension 
            var css_i = TVGL.Slice.GetUniformlySpacedCrossSections(
                ts, d, double.NaN, -1, thickness).ToList();
            var TS = new List<Solid>();
            for (int i = 1; i < css_i.Count(); i++)
            {
                var temp_css = new List<Polygon>();
                if (css_i[i - 1] is null)
                    css_i[i] = null;
                else
                {
                    foreach(var next_layer in  css_i[i])
                    {
                        foreach (var current_layer in css_i[i - 1])
                        {
                            var interaction = current_layer.GetPolygonInteraction(next_layer);
                            if (!interaction.IntersectionWillBeEmpty())
                            {
                                temp_css.Add(next_layer);
                                break;
                            }
                        }
                    }
                }
                css_i[i] = temp_css;
                var newcss = TVGL.CrossSectionSolid.CreateConstantCrossSectionSolid(d, i * thickness, thickness, css_i[i], 1e-10, UnitType.unspecified);
                TS.Add(newcss.ConvertToTessellatedExtrusions(false, false));
            }
            //Presenter.ShowAndHang(TS);
            return TS;
        }

    }

    public class Loops
    {
        public static List<Polygon> GetLoops(Dictionary<Edge, Vector2> edgeDictionary, Vector3 normal, double distanceToOrigin,
            out Dictionary<Vertex2D, Edge> e2VDictionary)
        {
            var polygons = new List<Polygon>();
            e2VDictionary = new Dictionary<Vertex2D, Edge>();
            while (edgeDictionary.Any())
            {
                var path = new List<Vector2>();
                var edgesInLoop = new List<Edge>();
                var firstEdgeInLoop = edgeDictionary.First().Key;
                var currentEdge = firstEdgeInLoop;
                var finishedLoop = false;
                PolygonalFace nextFace = null;
                do
                {
                    var intersectVertex2D = edgeDictionary[currentEdge];
                    edgeDictionary.Remove(currentEdge);
                    edgesInLoop.Add(currentEdge);
                    path.Add(intersectVertex2D);
                    var prevFace = nextFace;
                    if (prevFace == null)
                        nextFace = (currentEdge.From.Dot(normal) < distanceToOrigin) ? currentEdge.OtherFace : currentEdge.OwnedFace;
                    else nextFace = (nextFace == currentEdge.OwnedFace) ? currentEdge.OtherFace : currentEdge.OwnedFace;
                    Edge nextEdge = null;
                    foreach (var whichEdge in nextFace.Edges)
                    {
                        if (currentEdge == whichEdge) continue;
                        if (whichEdge == firstEdgeInLoop)
                        {
                            finishedLoop = true;
                            if (path.Count > 2)
                                AddToPolygons(path, edgesInLoop, polygons, e2VDictionary);
                            break;
                        }
                        else if (edgeDictionary.ContainsKey(whichEdge))
                        {
                            nextEdge = whichEdge;
                            break;
                        }
                    }
                    if (!finishedLoop && nextEdge == null)
                    {
                        output("Incomplete loop.", 3);
                        if (path.Count > 2)
                            AddToPolygons(path, edgesInLoop, polygons, e2VDictionary);
                        finishedLoop = true;
                    }
                    else currentEdge = nextEdge;
                } while (!finishedLoop);
            }
            return polygons.CreateShallowPolygonTrees(false);
        }
        public static TVGL.VerbosityLevels Verbosity = TVGL.VerbosityLevels.OnlyCritical;

        public static bool output(object message, int verbosityLimit = 0)
        {
            if ((verbosityLimit > (int)Verbosity)
                || string.IsNullOrEmpty(message.ToString()))
                return false;
            Debug.WriteLine(message);
            return true;
        }
        public static void AddToPolygons(List<Vector2> path, List<Edge> edgesInLoop, List<Polygon> polygons, Dictionary<Vertex2D, Edge> e2VDictionary)
        {
            var polygon = new Polygon(path);
            polygons.Add(polygon);
            for (int i = 0; i < polygon.Vertices.Count; i++)
                e2VDictionary.Add(polygon.Vertices[i], edgesInLoop[i]);
        }

    }
}
