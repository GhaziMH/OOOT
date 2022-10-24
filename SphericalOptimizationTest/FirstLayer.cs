using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using OptimizationToolbox;
using TVGL;
using static System.Net.Mime.MediaTypeNames;

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
                return SupportStructureScore(d, QuadricSlicer.GetUniformlySpacedCrossSections(tessellatedSolid, d, 1E-6, -1, thickness)[0]);
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
                return FirstLayerScore(QuadricSlicer.GetUniformlySpacedCrossSections(tessellatedSolid, d, 1E-6, -1, thickness)[0], LargestArea);
            }
            else
            {
                //Console.WriteLine("Current Direction: " + ((decimal)d.X) + " , " + ((decimal)d.Y) + " , " + ((decimal)d.Z));
                //Vector3 d = MakeUnitVectorFromSpherical(x[0], x[1]);

                var w1 = 0.5; var w2 = 0.3; var w3 = 0.2;
                var x2 = 0.0;
                var firstLayer = QuadricSlicer.GetUniformlySpacedCrossSections(tessellatedSolid, d, 1E-6, -1, thickness)[0];

                //Presenter.ShowAndHang(test);
                foreach (var face in tessellatedSolid.Faces)
                {
                    var dot = d.Dot(face.Normal);
                    x2 += face.Area * StaircaseEffectScore(dot);
                }
                x2 /= tessellatedSolid.Faces.Select(x => x.Area).Sum();
                var x1 = SupportStructureScore(d, firstLayer);

                var x3 = FirstLayerScore(firstLayer, LargestArea);

                double total = w1 * x1 + w2 * x2 + w3 * x3;
                //Console.WriteLine("Total Cost: " + ((decimal)total));

                return total;
            }
        }
        private List<Polygon> GetFirstLayer(Vector3 direction, double offset = 1E-6)
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
        }

        private double FirstLayerScore(List<Polygon> FirstLayer, double maxArea)
        {
            // Area ratio of the total surface and first layer that touch the printing bed,
            // The ration Eq.= 0.001*Total/FirstLayer
            // The best achievable value is approximately 0.002, an object consists of a single thin layer 
            // Printing on faces or edges will return a value range from 1E1 to less than 1E6, 
            // but in the case of printing on a single vertex, this function would return a value > 1E6,

            // return 1E-3 *tessellatedSolid.SurfaceArea / FirstLayer.Sum(x => x.Area);
            // update max score 0 and min score is almost 1
            return Math.Abs(1 - FirstLayer.Select(x => x.Area).Sum() / maxArea);
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
            double score = (Math.Exp(b * (Math.Abs(dot) - 1)) + Math.Exp(-b * Math.Abs(dot))) / denomProx;
            if (score > 1)
                score = 1;
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
