using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using OptimizationToolbox;
using TVGL;

namespace SphericalOptimizationTest
{
    internal class FirstLayer : IObjectiveFunction
    {
        private int Case;
        private double thickness;
        private double LargestArea;
        private const double maxAngle = Math.PI * 3 / 4;
        //max overhanging angle Pi*3/4 , 135 degree from printing bed normal direction,
        private const double ToolheadAngle = Math.PI / 2;
        // toolhead is assumed to be as a cone-shape with angle Pi/2, 90 degree,

        private readonly TessellatedSolid tessellatedSolid;
        public TessellatedSolid bestTessellatedSolid { get; private set; }
        private double bestObjFunc = double.PositiveInfinity;
        private const double b = 50; //practical values between 5 to 100
        private static double denomProx = 1 + Math.Exp(-b);


        public FirstLayer(TessellatedSolid tessellatedSolid, double thickness, double LargestArea, int Case = 0)
        {
            this.tessellatedSolid = tessellatedSolid;
            this.thickness = thickness;
            this.LargestArea = LargestArea;
            this.Case = Case;
        }

        public double calculate(double[] x)
        {
            Vector3 d = new Vector3(x).Normalize();

            if (Case == 1)
            {
                return SupportStructureScore135DegOverhang(d, Loops.GetFirstLayer(tessellatedSolid,thickness,d));
                //return SupportStructureScore(d,GetFirstLayer(d));
                //return SupportStructureScore(d, QuadricSlicer.GetUniformlySpacedCrossSections(tessellatedSolid, d, 1E-6, -1, thickness)[0]);
                //return SupportStructureScore135DegOverhang(d, QuadricSlicer.GetUniformlySpacedCrossSections(tessellatedSolid, d, 1E-6, -1, thickness)[0]);
            }
            else if (Case == 2)
            {
                var X2 = 0.0;
                
                foreach (var face in tessellatedSolid.Faces)
                {
                    var dot = d.Dot(face.Normal.Normalize());
                    X2 += face.Area * StaircaseEffectScore(dot);
                }
                return X2 / tessellatedSolid.Faces.Select(x => x.Area).Sum();
            }
            else if(Case == 3)
            {
                return FirstLayerScore(Loops.GetFirstLayer(tessellatedSolid, thickness, d), LargestArea,d);
                //return FirstLayerScore(QuadricSlicer.GetUniformlySpacedCrossSections(tessellatedSolid, d, 1E-6, -1, thickness)[0], LargestArea,d);
            }
            else
            {
                //Console.WriteLine("Current Direction: " + ((decimal)d.X) + " , " + ((decimal)d.Y) + " , " + ((decimal)d.Z));
                //Vector3 d = MakeUnitVectorFromSpherical(x[0], x[1]);

                var w1 = 0.5; var w2 = 0.35; var w3 = 0.15;
                var x2 = 0.0;
                //var firstLayer = QuadricSlicer.GetUniformlySpacedCrossSections(tessellatedSolid, d, 1E-6, -1, thickness)[0];
                var firstLayer = Loops.GetFirstLayer(tessellatedSolid, thickness, d);
                //Presenter.ShowAndHang(test);
                foreach (var face in tessellatedSolid.Faces)
                {
                    var dot = d.Dot(face.Normal);
                    x2 += face.Area * StaircaseEffectScore(dot);
                }
                x2 /= tessellatedSolid.Faces.Select(x => x.Area).Sum();
                var x1 = SupportStructureScore135DegOverhang(d, firstLayer);

                var x3 = FirstLayerScore(firstLayer, LargestArea,d);

                double total = w1 * x1 + w2 * x2 + w3 * x3;
                //Console.WriteLine("Total Cost: " + ((decimal)total));

                return total;
            }
        }

        private double FirstLayerScore(List<Polygon> FirstLayer, double maxArea, Vector3 d)
        {
            // Area ratio of the total surface and first layer that touch the printing bed,
            // The ration Eq.= 0.001*Total/FirstLayer
            // The best achievable value is approximately 0.002, an object consists of a single thin layer 
            // Printing on faces or edges will return a value range from 1E1 to less than 1E6, 
            // but in the case of printing on a single vertex, this function would return a value > 1E6,

            // return 1E-3 *tessellatedSolid.SurfaceArea / FirstLayer.Sum(x => x.Area);
            // update max score 0 and min score is almost 1
            //if(d.X != 1 && d.Y != 1 && d.Z != 1)
                //Presenter.ShowAndHang(FirstLayer);
            return 1 - (FirstLayer.Select(x => x.Area).Sum() / maxArea);
            /// Possible future improvments:
            /// 1. measure the first layer score based on printing cases:
            ///     A. Desirable case: 
            ///     Printing on faces,
            ///     How to detect:  The first layer direction is one of the face directions of the convexhull
            ///     B. Acceptable case (if noticed a significant improvement on other factors, e.g. less support):
            ///     Printing on edges,
            ///     How to detect: 
            ///     Problematic case: 
            ///     Prinitng on vertices,
            ///     How to detect:
        }
        private double StaircaseEffectScore(double dot)
        {
            //return dot * (1 - dot);
            //double score = (Math.Exp(b * (Math.Abs(dot) - 1)) + Math.Exp(-b * Math.Abs(dot))) / denomProx;
            double score = (Math.Exp(b * ((dot*dot) - 1)) + Math.Exp(-b * (dot* dot))) / denomProx;

            return 1 - score;
        }

        private double SupportStructureScore(Vector3 d, List<Polygon> firstLayer, double currentBest = double.PositiveInfinity)
        {
            var SolidOriginal = TVGL.Slice.GetUniformlySpacedCrossSections(
                tessellatedSolid, d, double.NaN, -1, 0.5 * thickness).ToList();
            SolidOriginal.Insert(0, firstLayer);
            var solid = CrossSectionSolid.CreateFromTessellatedSolid(tessellatedSolid, d, SolidOriginal.Count());
            var css_i = new List<Polygon>[(int)(SolidOriginal.Count() / 2)];
            css_i.ToList();
            for (int i = 0; i < css_i.Count(); i++)
            {
                css_i[i] = SolidOriginal[i * 2];
            }
            
            var VolumetricError = 0.0; // volumetric error  
            var PrintableVolume = 0.0; // printable region of layers
            for (int i = 1; i < css_i.Count(); i++)
            {
                if (i <= 10)
                {
                    var temp_css = new List<Polygon>();
                    if (css_i[i - 1] is null)
                        css_i[i] = null;
                    else
                    {
                        for (int j = 0; j < css_i[i].Count(); j++)
                        {
                            for (int k = 0; k < css_i[i - 1].Count(); k++)
                            {
                                var interaction = css_i[i][j].GetPolygonInteraction(css_i[i - 1][k]);
                                if (!interaction.IntersectionWillBeEmpty())
                                {
                                    var intersections = css_i[i][j].Intersect(css_i[i - 1][k]);
                                    temp_css.AddRange(intersections);
                                }
                            }
                        }
                    }
                    PrintableVolume += temp_css.Select(x => x.Area).Sum();
                    css_i[i] = temp_css;

                    if (i != css_i.Count() - 1)
                    {
                        var subrtractPoly = PolygonOperations.Subtract(css_i[i], SolidOriginal[(i * 2) + 1]);
                        VolumetricError += subrtractPoly.Sum(x => x.Area);
                    }
                    else if ((double)SolidOriginal.Count() / (double)css_i.Count() != 2)
                        VolumetricError += SolidOriginal.Last().Sum(x => x.Area);
                }
                else
                {
                    var temp_css = new List<Polygon>();
                    if (css_i[i - 1] is null)
                        css_i[i] = null;
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
                    PrintableVolume += temp_css.Select(x => x.Area).Sum();
                    css_i[i] = temp_css;

                    if (i != css_i.Count() - 1)
                    {
                        var subrtractPoly = PolygonOperations.Subtract(css_i[i], SolidOriginal[(i * 2) + 1]);
                        VolumetricError += subrtractPoly.Sum(x => x.Area);
                    }
                    else if ((double)SolidOriginal.Count() / (double)css_i.Count() != 2)
                        VolumetricError += SolidOriginal.Last().Sum(x => x.Area);
                }
                
            }

           
            return 1 - ((PrintableVolume -VolumetricError)* thickness / tessellatedSolid.Volume);
        }

        private double SupportStructureScore135DegOverhang(Vector3 d, List<Polygon> firstLayer, double currentBest = double.PositiveInfinity)
        {
            var SolidOriginal = TVGL.Slice.GetUniformlySpacedCrossSections(
                tessellatedSolid, d, double.NaN, -1, thickness).ToList();
            SolidOriginal.Insert(0, firstLayer);
            //SolidOriginal.Reverse();
            var AccessibleRegion = new List<Polygon>[SolidOriginal.Count];
            AccessibleRegion[0] = SolidOriginal[0];
            var ListOverhangRegion = new List<Polygon>[SolidOriginal.Count].ToList();

            for (int i = 1; i < SolidOriginal.Count(); i++)
            {
                if (SolidOriginal[i].Count == 0 || SolidOriginal[i - 1].Count == 0)
                    continue;
                else if (i <= 10)
                {
                    var angle = (i - 1) * Math.PI / 20;
                    var t = thickness * Math.Sin(angle) / Math.Sin(Math.PI / 2 - angle);
                    AccessibleRegion[i] = SolidOriginal[i].IntersectPolygons(PolygonOperations.OffsetRound(AccessibleRegion[i - 1], t));

                }
                else if (i > 10)
                {
                    var TempPoly = new List<Polygon>();
                    var TempOverhang = new List<Polygon>();
                    for (int j = 0; j < SolidOriginal[i].Count; j++)
                    {
                        var Empty = true;
                        for (int k = 0; k < SolidOriginal[i - 1].Count; k++)
                        {
                            if (!SolidOriginal[i][j].GetPolygonInteraction(SolidOriginal[i - 1][k]).IntersectionWillBeEmpty())
                            {
                                TempPoly.Add(SolidOriginal[i][j]);
                                Empty = false;
                                break;
                            }
                        }
                        if(Empty)
                        {
                            TempOverhang.Add(SolidOriginal[i][j]);
                        }
                    }
                    ListOverhangRegion[i] = TempOverhang;
                    AccessibleRegion[i] = TempPoly;
                }
            }
            var startid = -1; var endid = -1;
            for (int i = 1; i < SolidOriginal.Count; i++)
            {
                if (ListOverhangRegion[i - 1] != null && ListOverhangRegion[i - 1].Count > 0) 
                {
                    if (ListOverhangRegion[i] != null && ListOverhangRegion[i].Count > 0) 
                    {
                        if (startid == -1)
                            startid = i;

                        var angle = (i - 1) * Math.PI / 20;
                        var t = thickness * Math.Sin(angle) / Math.Sin(Math.PI / 2 - angle);
                        ListOverhangRegion[i] = ListOverhangRegion[i].IntersectPolygons(PolygonOperations.OffsetRound(ListOverhangRegion[i - 1], t));

                    }
                    else
                    {
                        endid = i;
                        var TempPoly = new List<Polygon>();
                        for (int j = 0; j < AccessibleRegion[i].Count; j++)
                        {
                            for (int k = 0; k < ListOverhangRegion[i - 1].Count; k++)
                            {
                                if (!AccessibleRegion[i][j].GetPolygonInteraction(ListOverhangRegion[i - 1][k]).IntersectionWillBeEmpty())
                                {
                                    TempPoly.Add(ListOverhangRegion[i - 1][k]);
                                    break;
                                }
                            }
                            
                        }
                        if(TempPoly.Count > 0)
                        {
                            if (startid != -1 && endid != -1) 
                                for (int j = startid; j < i; j++)
                                {
                                    AccessibleRegion[i].AddRange(ListOverhangRegion[j]);
                                }
                        }
                    }
                    
                }
                
            }


            /*
            var showRegion = new List<List<Vector3>>();

            for (int i = 0; i < AccessibleRegion.Count(); i++)
            {
                if (AccessibleRegion[i].Count > 0)
                    foreach (var p in AccessibleRegion[i])
                    {
                        var V = new List<Vector3>();
                        foreach (var v in p.Path)
                            V.Add(new Vector3(v, i * thickness));
                        showRegion.Add(V);
                    }

            }
            
            var Showorigianl = new List<List<Vector3>>();
            for (int i = 0; i < SolidOriginal.Count; i++)
            {
                foreach (var p in SolidOriginal[i])
                {
                    var V = new List<Vector3>();
                    foreach (var v in p.Path)
                        V.Add(new Vector3(v, i * thickness));
                    Showorigianl.Add(V);
                }
            }

            {
                Presenter.ShowVertexPaths(showRegion);
                Presenter.ShowVertexPaths(Showorigianl);

            }*/
            var Volume = AccessibleRegion.Select(x => x.Select(p => p.Area).Sum()).Sum() * thickness;

            return Math.Abs(1 - (Volume / tessellatedSolid.Volume));
        }


        private double SupportStructureScoreSphere(Vector3 d, List<Polygon> firstLayer, double currentBest = double.PositiveInfinity)
        {
            double t = double.PositiveInfinity;
            foreach(var v in tessellatedSolid.Vertices)
                if (d.Dot(v.Coordinates) < t)
                    t = d.Dot(v.Coordinates);
            Vector3 SphereCenter = d * (t - thickness * 100);
            Plane plane = new Plane(SphereCenter, d);

            Sphere sph = new Sphere(SphereCenter, 10, true);
            var (SphSlices,SphFaces,SphRadius) = QuadricSlicer.GetSphericalSections(tessellatedSolid,sph,thickness);

            List<List<(List<Vector3>, List<List<Vector3>>)>> SphSlicesordered = QuadricSlicer.ConverTo3DPolygonWithInnerPolygon(SphSlices.ToList(), plane);
            
            

            var VolumetricError = 0.0; // volumetric error  
            var PrintableVolume = 0.0; // printable region of layers


            return 1 - (PrintableVolume - VolumetricError) * thickness / tessellatedSolid.Volume;
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
            var css_i = Slice.GetUniformlySpacedCrossSections(
                ts, d, double.NaN, -1, thickness).ToList();
            var TS = new List<Solid>();
            //css_i.Insert(0, Slice.GetUniformlySpacedCrossSections(ts, d, 1E-6, -1, thickness, true)[0]);
            for (int i = 1; i < css_i.Count; i++)
            {
                if (css_i[i].Count > 0)
                {
                    var temp_css = new List<Polygon>();
                    foreach (var next_layer in css_i[i])
                    {
                        foreach (var current_layer in css_i[i - 1])
                        {
                            var interaction = current_layer.GetPolygonInteraction(next_layer);
                            if (!interaction.IntersectionWillBeEmpty())
                            // Add exception for A is a hole inside B
                            {
                                temp_css.Add(next_layer);
                                break;
                            }
                        }
                    }

                    css_i[i] = PolygonOperations.UnionPolygons(temp_css);
                    
                    
                }
                else
                    css_i[i].Clear();
                
            }

            for (int i = 0; i < css_i.Count; i++)
            {
                if (css_i[i].Count > 0)
                {
                    var newcss = TVGL.CrossSectionSolid.CreateConstantCrossSectionSolid(d, i * thickness, thickness, new List<Polygon> { css_i[i][0], css_i[i][0] }, 1e-10, UnitType.unspecified);
                    TS.Add(newcss.ConvertToTessellatedExtrusions(false, false));
                }
            }
            //Presenter.ShowAndHang(TS);
            return TS;
        }

        public static List<Solid> ShowShape((List<List<Vector3>>[], List<List<PolygonalFace>>[], Sphere[]) Data, Vector3 d )
        {
            // printing bed is assumed to change in dimension 
            var Layers = Data.Item1.Where(x => x != null).Select(x => x).ToList();
            var Spheres = Data.Item3.ToList();
            var thickness = Spheres[0].Radius - Spheres[1].Radius;
            var TS = new List<Solid>();


            var css_i = new List<List<Polygon>>();
            for (int i = 0; i < Layers.Count; i++)
            {
                if (Layers[i].Count > 0)
                {
                    var temp_css = new List<Polygon>();
                    for (int j = 0; j < Layers[i].Count; j++)
                    {
                        temp_css.Add(new Polygon(Spheres[i].TransformFrom3DTo2D(Layers[i][j], true)));
                    }
                    css_i.Add(PolygonOperations.UnionPolygons(temp_css));

                }

            }

            for (int i = 1; i < css_i.Count; i++)
            {
                if (css_i[i].Count > 0)
                {
                    var temp_css = new List<Polygon>();
                    foreach (var next_layer in css_i[i])
                    {
                        foreach (var current_layer in css_i[i - 1])
                        {
                            var interaction = current_layer.GetPolygonInteraction(next_layer);
                            if (!interaction.IntersectionWillBeEmpty())
                            // Add exception for A is inside B
                            {
                                temp_css.Add(next_layer);
                                break;
                            }
                        }
                    }

                    css_i[i] = PolygonOperations.UnionPolygons(temp_css);


                }
                else
                    css_i[i].Clear();

            }

            for (int i = 0; i < css_i.Count; i++)
            {
                if (css_i[i].Count > 0)
                {
                    var newcss = TVGL.CrossSectionSolid.CreateConstantCrossSectionSolid(d, i * thickness, thickness, new List<Polygon> { css_i[i][0], css_i[i][0] }, 1e-10, UnitType.unspecified);
                    TS.Add(newcss.ConvertToTessellatedExtrusions(false, false));
                }
            }
            //Presenter.ShowAndHang(TS);
            return TS;
        }


    }

    public class Loops
    {
        public static List<Polygon> GetFirstLayer(TessellatedSolid ts, double thickness, Vector3 direction, double offset = 1E-6)
        {
            // The first layer stands on the printing bed on:
            // 1. vertices,
            // 2. edges,
            // 3. faces,
            // 4. combination of the above cases

            // in the case of standing on a vertex, and edge,
            // on an edge, the first layer is a line, but the thickness and surface curvature can influnce the decision
            // Computationally, we can adjust the first layer thickness to have a small area.
            // In most cases, we can get small area by offsetting the layer along the printing direction 1E-6 of the printing thickness.
            // Possible issues: the first layer is too small to be printed,
            //                  example: the size of the printing thickness is larger than the produced first layer
            // possible deteching condition:
            //                  check the surface curvature at a vertex, or edge 


            var transform = direction.TransformToXYPlane(out _);
            var plane = new Plane(0.0, direction);
            //First, sort the vertices along the given axis. Duplicate distances are not important.
            var sortedVertices = ts.Vertices.OrderBy(v => v.Dot(direction)).ToArray();
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
        }

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
