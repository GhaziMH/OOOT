using System;
using System.Collections.Generic;
using System.Linq;
using OptimizationToolbox;
using TVGL;
using TVGL.Numerics;
using TVGL.TwoDimensional;

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
            //Vector3 d = MakeUnitVectorFromSpherical(x[0], x[1]);

            //
          
            var w1 = 0.5; var w2 = 0.3; var w3 = 0.2;
            var x1 = SupportStructureScore(d);
            var x2 = 0.0;
            foreach (var face in tessellatedSolid.Faces)
            {
                var dot = d.Dot(face.Normal);
                x2 += face.Area * StaircaseEffectScore(dot);
            }
            double x3 = 0.0;
            double total = w1 * x1 + w2 * x2 + w3 * x3;
            Console.WriteLine("Current Direction: " + ((decimal)d.X) + " , " + ((decimal)d.Y) + " , " + ((decimal)d.Z));
            if (total == double.PositiveInfinity)
                Console.WriteLine("Total cost is larger than current best cost");
            else
                Console.WriteLine("Total Cost: " + ((decimal)total));
            //if (total<bestObjFunc)
            //{
            //    //bestTessellatedSolid=
            //}
            return total;
        }

        
        private double StaircaseEffectScore(double dot)
        {
            return 1 - (Math.Exp(b * (Math.Abs(dot) - 1)) + Math.Exp(-b * Math.Abs(dot))) / denomProx;
        }

        private double SupportStructureScore(Vector3 d, double currentBest = double.PositiveInfinity)
        {
            var css_i = TVGL.Boolean_Operations.Slice.GetUniformlySpacedCrossSections(
                tessellatedSolid, d, double.NaN, -1, thickness).ToList();
            var SolidOriginal = TVGL.Boolean_Operations.Slice.GetUniformlySpacedCrossSections(
                tessellatedSolid, d, double.NaN, -1, 0.5 * thickness).ToList();
            var verts = new List<List<Vector3>>();
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
                Area += PolygonOperations.Subtract(css_i[i], SolidOriginal[(i * 2) - 1]).Select(x => x.Area).Sum();

            }
            if (SolidOriginal.Count() / css_i.Count() != 2) 
            {
                foreach (var A in SolidOriginal.Last())
                    Area += A.Area;
            }
            
            var newcss = new List<CrossSectionSolid>();
            var newcss2 = new List<Solid>();
            for (int i = 0; i < css_i.Count(); i++)
            {
                newcss.Add(TVGL.CrossSectionSolid.CreateConstantCrossSectionSolid(d, i*thickness, thickness, css_i[i], 1e-10, UnitType.unspecified));
                newcss2.Add(newcss[i].ConvertToTessellatedExtrusions(false, false));
            }
            Presenter.ShowAndHang(newcss2);
            return Math.Abs((Area2 * thickness / tessellatedSolid.Volume) - 1);
        }


        private TessellatedSolid GetPrintableShape(Vector3 d)
        {
            // printing bed is assumed to change in dimension 

            return new TessellatedSolid();
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
