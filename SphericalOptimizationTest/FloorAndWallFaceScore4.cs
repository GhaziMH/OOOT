using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using OptimizationToolbox;
using TVGL;

namespace SphericalOptimizationTest
{
    internal class FloorAndWallFaceScore4Inequality : FloorAndWallFaceScore4Base, IInequality
    {
        public FloorAndWallFaceScore4Inequality(TessellatedSolid tessellatedSolid, double thickness, double LargestArea,
            double maxAllowable) 
            : base(tessellatedSolid, thickness, LargestArea)
        {
            this.MaxAllowable = maxAllowable;
        }
        double IOptFunction.calculate(double[] x)
        {
            return base.calculate(x) - MaxAllowable;
        }

        public double MaxAllowable
        { get; }
    }
    internal class FloorAndWallFaceScore4ObjFun : FloorAndWallFaceScore4Base, IObjectiveFunction
    {
        public FloorAndWallFaceScore4ObjFun(TessellatedSolid tessellatedSolid, double thickness, double LargestArea)
            : base(tessellatedSolid, thickness, LargestArea)
        {
        }
        double IOptFunction.calculate(double[] x)
        {
            return base.calculate(x);
        }
    }
    internal class FloorAndWallFaceScore4Base
    {
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


        protected FloorAndWallFaceScore4Base(TessellatedSolid tessellatedSolid, double thickness, double LargestArea)
        {
            this.tessellatedSolid = tessellatedSolid;
            this.thickness = thickness;
            this.LargestArea = LargestArea;
        }

        protected virtual double calculate(double[] x)
        {
            Vector3 d = new Vector3(x).Normalize();
            //Console.WriteLine("Current Direction: " + ((decimal)d.X) + " , " + ((decimal)d.Y) + " , " + ((decimal)d.Z));
            //Vector3 d = MakeUnitVectorFromSpherical(x[0], x[1]);

            var w1 = 0.5; var w2 = 0.5; var w3 = 0.2;
            var x2 = 0.0;
            var test = GetFirstLayer(d);

            //Presenter.ShowAndHang(test);
            /*foreach (var face in tessellatedSolid.Faces)
            {
                var dot = d.Dot(face.Normal);
                x2 += face.Area * StaircaseEffectScore(dot);
            }
            */
            var FirstLayer = FirstLayerScore(test, LargestArea);
            //var x1 = SupportStructureScore(d, test);
            var x1 = 0;

            if (x1 < 0)
                Console.WriteLine("negative support score");
            if (FirstLayer < 0)
                Console.WriteLine("negative first layer scroe");
            double x3 = 0.0;
            //double total = w1 * x1 + 1 * FirstLayer + w3 * x3;
            //Console.WriteLine("Total Cost: " + ((decimal)total));

            return FirstLayer;
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
            // Area ration of the total surface and first layer that touch the printing bed,
            // The ration Eq.= 0.001*Total/FirstLayer
            // The best achievable value is approximately 0.002, an object consists of a single thin layer 
            // Printing on faces or edges will return a value range from 1E1 to less than 1E6, 
            // but in the case of printing on a single vertex, this function would return a value > 1E6,

            // return 1E-3 *tessellatedSolid.SurfaceArea / FirstLayer.Sum(x => x.Area);
            // update max score 0 and min score is almost 1
            return maxArea - FirstLayer.Select(x => x.Area).Sum();
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
            return 1 - (Math.Exp(b * (Math.Abs(dot) - 1)) + Math.Exp(-b * Math.Abs(dot))) / denomProx;
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
            var Area = 0.0; // volumetric error  
            var Area2 = 0.0; // printable region of layers
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
                Area2 += temp_css.Select(x => x.Area).Sum();
                //if (Area2 > 0 & Area2 < currentBest)
                //   return double.PositiveInfinity;
                css_i[i] = temp_css;

                if (i != css_i.Count() - 1)
                {
                    //Presenter.ShowAndHang(css_i[i]);
                    //Presenter.ShowAndHang(SolidOriginal[(i*2)+1]);

                    var subrtractPoly = PolygonOperations.Subtract(css_i[i], SolidOriginal[(i * 2) + 1]);
                    //if(subrtractPoly.Count()>0)
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
            return 1 - (Area2 * thickness / tessellatedSolid.Volume) + (Area * thickness);
            //return Math.Pow(Area2 * thickness - tessellatedSolid.Volume, 2);
            //return Math.Abs(Area2 * thickness - tessellatedSolid.Volume) / (Area2 * thickness + tessellatedSolid.Volume);
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

}
