using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using HelixToolkit.SharpDX.Core;
using OptimizationToolbox;
using TVGL;
using Polygon = TVGL.Polygon;

namespace SphericalOptimizationTest
{
    internal class FirstLayer : IObjectiveFunction
    {
        private int Case;
        private double thickness;
        private double LargestArea;
        private readonly TessellatedSolid tessellatedSolid;


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
                return PrintableVolumeScore(d);
                //return PrintableVolumeRatio135Degree(d);
            }
            else if (Case == 2)
            {
                var X2 = 0.0;
                foreach (var face in tessellatedSolid.Faces)
                {
                    var dot = d.Dot(face.Normal.Normalize());
                    X2 += face.Area * StaircaseEffectScore(dot);
                    //X2 += StaircaseEffectScore(dot, d, face, ref delta_worseCase);
                }
                return X2 / tessellatedSolid.Faces.Select(x => x.Area).Sum();
            }
            else if(Case == 3)
            {
                return FirstLayerScore(Loops.GetFirstLayer(tessellatedSolid, thickness, d), LargestArea, d);
                //return FirstLayerScore(QuadricSlicer.GetUniformlySpacedCrossSections(tessellatedSolid, d, 1E-6, -1, thickness)[0], LargestArea,d);
            }
            else
            {
                //Console.WriteLine("Current Direction: " + ((decimal)d.X) + " , " + ((decimal)d.Y) + " , " + ((decimal)d.Z));
                //Vector3 d = MakeUnitVectorFromSpherical(x[0], x[1]);

                var w1 = 0.5; var w2 = 0.35; var w3 = 0.15;

                var x1 = PrintableVolumeScore(d);
                //var x1 = PrintableVolumeRatio135Degree(d);

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


                var x3 = FirstLayerScore(firstLayer, LargestArea,d);

                double total = w1 * x1 + w2 * x2 + w3 * x3;

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
            if(FirstLayer == null)
            {
                FirstLayer = new List<Polygon>();
            }
            var NormScore = FirstLayer.Select(x => x.Area).Sum();
            var Area = 0.0;
            foreach (var poly in FirstLayer)
            {
                Area += poly.AllPolygons.Select(x => x.Area).Sum();
            }
            if (Area >= maxArea)
                return 0;
            return 1 - Area / maxArea;
            /*
            var tempv = 0.0;
            var cosang = Math.Cos(Math.PI / 18);
            foreach (var ch in tessellatedSolid.ConvexHull.Faces)
            {
                if (d != ch.Normal)
                    if (Math.Abs(ch.Normal.Normalize().Dot(d)) >= cosang)
                    {
                        var angle = Math.Acos(ch.Normal.Dot(d));
                        if (angle != 0)
                        {
                            var currentlayer = Loops.GetFirstLayer(tessellatedSolid, thickness, ch.Normal);
                            //Presenter.ShowAndHang(currentlayer);
                            var currentValue = 0.0;
                            foreach (var poly in currentlayer)
                            {
                                currentValue += poly.AllPolygons.Select(x => x.Area).Sum();
                            }
                            //var currentValue = (Math.PI/18 - angle) * QuadricSlicer.GetUniformlySpacedCrossSections(tessellatedSolid, ch.Normal, 1E-6, -1, thickness)[0].Select(x => x.Area).Sum();
                            currentValue = currentValue * (1 - (Math.PI / 18) - angle);
                            if (currentValue > tempv)
                                tempv = currentValue;
                        }
                    }
            }


            return 1 - (0.5 * (NormScore + tempv) / maxArea);
            */
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
            /*
            var direction = 0;
            if (dot > 0)
                direction = -1;
            else if (dot < 0)
                direction = 1;
            else
                direction = 0;

            var b_values = new List<(double,Vector3)>();
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                    if (i != j)
                    {
                        var dist = MiscFunctions.DistancePointToPlane(face.Vertices[i].Coordinates, d,
                            face.Vertices[j].Coordinates);
                        if (dist > 0)
                            b_values.Add((dist, face.Vertices[i].Coordinates));
                    }
            }
            b_values = b_values.OrderByDescending(x=>x.Item1).ToList();

            //var delta = b_dist[0].Item1 * dot;
            var delta = 0.0;
            if(dot ==0)
                dot = 0;
            //delta_worseCase += b_dist * 0.5 * Math.Sqrt(2);
            if (delta < 0)
                delta = 0;
            //return delta;*/
            return 4 * (-Math.Pow(dot, 4) + Math.Pow(dot, 2));

            //return dot * (1 - dot);
            //double score = (Math.Exp(b * (Math.Abs(dot) - 1)) + Math.Exp(-b * Math.Abs(dot))) / denomProx;
            //double score = (Math.Exp(b * ((dot*dot) - 1)) + Math.Exp(-b * (dot* dot))) / denomProx;

            //return 1 - score;
        }

        private double PrintableVolumeScore(Vector3 d, double SwitchingLayerLength = 50.0,  bool OverhangOver90Degree = false)
        {
            if (OverhangOver90Degree) // max overhang angle is 135 degrees
            {
                var OriginalSolid = TVGL.Slice.GetUniformlySpacedCrossSections(
                tessellatedSolid, d, double.NaN, -1, thickness).ToList();

                var ReachableSolid = new List<Polygon>[OriginalSolid.Count];
                ReachableSolid[0] = OriginalSolid[0];

                var TempOverhang = new Dictionary<int, List<Polygon>>();

                int SwitchingLayer = (int)(SwitchingLayerLength * thickness); // Slice number where the overhang angle is 90 degrees
                var tList = new double[OriginalSolid.Count];

                for (int i = 1; i < OriginalSolid.Count(); i++)
                {
                    if (OriginalSolid[i].Count == 0 || OriginalSolid[i - 1].Count == 0 || ReachableSolid[i - 1].Count == 0)
                        break;
                    else
                    {
                        TempOverhang.Add(i - 1, new List<Polygon>());
                        var t = thickness;
                        if (i < SwitchingLayer)
                        {
                            var angle = i * Math.PI * 0.5 / SwitchingLayer;
                            t *= Math.Tan(angle);
                        }
                        else if (i == SwitchingLayer)
                        {
                            t = double.NaN;
                        }
                        else if (i < (int)1.5 * SwitchingLayer)
                        {
                            var angle = Math.PI * (1 - (i * 0.5 / SwitchingLayer));
                            t /= Math.Tan(angle);
                        }
                        tList[i - 1] = t;

                        var tempLayer = new List<Polygon>();
                        for (int j = 0; j < OriginalSolid[i].Count(); j++)
                        {
                            var Empty = true;
                            for (int k = 0; k < ReachableSolid[i - 1].Count(); k++)
                            {
                                if (!OriginalSolid[i][j].GetPolygonInteraction(ReachableSolid[i - 1][k]).IntersectionWillBeEmpty())
                                {
                                    Empty = false;
                                    var poly = new List<Polygon>();
                                    if (i < SwitchingLayer)
                                    {

                                        var OffsettedPoly = PolygonOperations.OffsetRound(ReachableSolid[i - 1][k], t, 1e-7);
                                        foreach (var p in OffsettedPoly)
                                        {
                                            tempLayer = PolygonOperations.UnionPolygons(
                                                PolygonOperations.Intersect(OriginalSolid[i][j], p),
                                                tempLayer);

                                        }
                                    }
                                    else
                                        tempLayer.Add(OriginalSolid[i][j]);
                                    break;
                                }
                            }
                            if (Empty)
                                TempOverhang[i - 1].Add(OriginalSolid[i][j]);
                        }
                        ReachableSolid[i] = tempLayer;
                    }

                }


                #region WIP Processing 135overhang case

                var NewTempOverhang = TempOverhang.ToList();
                NewTempOverhang.Reverse();

                #region Collect overhang over 90 degrees
                /*
                foreach (var i in NewTempOverhang)
                {
                    if (i.Key < SolidOriginal.Count && i.Value.Count > 0 && i.Key > 5)  // i.key should not be the last layer 
                    {


                        //Presenter.ShowAndHang(AccessibleRegion[i.Key]);
                        //Presenter.ShowAndHang(AccessibleRegion[i.Key - 1]);
                        //Presenter.ShowAndHang(i.Value);
                        var connectedLayer = PolygonOperations.Subtract(AccessibleRegion[i.Key + 1], i.Value);
                        foreach (var poly1 in connectedLayer)
                        {
                            foreach (var poly2 in AccessibleRegion[i.Key])

                            {
                                if (poly1.GetPolygonInteraction(poly2).IntersectionWillBeEmpty())
                                {
                                    var offsetpoly = PolygonOperations.OffsetRound(poly1, tList[i.Key], 1e-7);
                                    var Polysubtract = PolygonOperations.Subtract(i.Value, offsetpoly);
                                    AccessibleRegion[i.Key] = PolygonOperations.UnionPolygons(Polysubtract,
                                        AccessibleRegion[i.Key]);
                                    break;
                                }
                            }
                        }

                        //Presenter.ShowAndHang(AccessibleRegion[i.Key]);
                    }
                }
                */
                #endregion


                #endregion

                #region Compute ratio of printable region to original
                var ReachableSolidArea = 0.0;
                for (int i = 0; i < OriginalSolid.Count; i++)
                {
                    if (ReachableSolid[i] != null)
                        foreach (var poly in ReachableSolid[i])
                        {
                            ReachableSolidArea += poly.AllPolygons.Select(x => x.Area).Sum();
                        }
                }
                var OriginalSolidArea = 0.0;
                for (int i = 0; i < OriginalSolid.Count; i++)
                {
                    if (OriginalSolid[i] != null)
                        foreach (var poly in OriginalSolid[i])
                        {
                            OriginalSolidArea += poly.AllPolygons.Select(x => x.Area).Sum();
                        }
                }
                #endregion
                //var volume2 = AccessibleRegion.Select(x => x.Select(p => p.Area).Sum()).Sum()*thickness;
                return Math.Abs(1 - (ReachableSolidArea / OriginalSolidArea));
            }

            else
            {
                var OriginalSolid = TVGL.Slice.GetUniformlySpacedCrossSections(
                tessellatedSolid, d, double.NaN, -1, thickness).ToList();
                var ReachableSolid = new List<Polygon>[OriginalSolid.Count];
                ReachableSolid[0] = OriginalSolid[0];

                int SwitchingLayer = 6; // Slice number where the overhang angle is 90 degrees

                for (int i = 1; i < OriginalSolid.Count(); i++)
                {
                    if (OriginalSolid[i].Count == 0 || OriginalSolid[i - 1].Count == 0 || ReachableSolid[i - 1].Count == 0)
                        break;
                    else
                    {

                        var t = thickness;
                        if (i < SwitchingLayer)
                        {
                            var angle = i * Math.PI * 0.5 / SwitchingLayer;
                            t *= Math.Tan(angle);
                        }
                        else if (i == SwitchingLayer)
                        {
                            t = double.NaN;
                        }
                        else if (i < (int)1.5 * SwitchingLayer)
                        {
                            var angle = Math.PI * (1 - (i * 0.5 / SwitchingLayer));
                            t /= Math.Tan(angle);
                        }

                        var tempLayer = new List<Polygon>();
                        for (int j = 0; j < OriginalSolid[i].Count(); j++)
                        {
                            for (int k = 0; k < ReachableSolid[i - 1].Count(); k++)
                            {
                                if (!OriginalSolid[i][j].GetPolygonInteraction(ReachableSolid[i - 1][k]).IntersectionWillBeEmpty())
                                {
                                    if (i < SwitchingLayer)
                                    {
                                        var OffsettedPoly = PolygonOperations.OffsetRound(ReachableSolid[i - 1][k], t, 1e-7);
                                        foreach (var p in OffsettedPoly)
                                        {
                                            tempLayer = PolygonOperations.UnionPolygons(
                                                PolygonOperations.Intersect(OriginalSolid[i][j], p),
                                                tempLayer);
                                        }
                                    }
                                    else
                                        tempLayer.Add(OriginalSolid[i][j]);
                                    break;
                                }
                            }
                        }
                        ReachableSolid[i] = tempLayer;
                    }
                }

                var OriginalSolidArea = 0.0;
                var ReachableSolidArea = 0.0;
                for (int i = 0; i < OriginalSolid.Count; i++)
                {
                    if (ReachableSolid[i] != null)
                        foreach (var poly in ReachableSolid[i])
                        {
                            ReachableSolidArea += poly.AllPolygons.Select(x => x.Area).Sum();
                        }
                    if (OriginalSolid[i] != null)
                        foreach (var poly in OriginalSolid[i])
                        {
                            OriginalSolidArea += poly.AllPolygons.Select(x => x.Area).Sum();
                        }
                }

                //var volume2 = AccessibleRegion.Select(x => x.Select(p => p.Area).Sum()).Sum()*thickness;

                return Math.Abs(1 - (ReachableSolidArea / OriginalSolidArea));
            }
            
        }

        private double PrintableVolumeRatio135Degree(Vector3 d)
        {

            var OriginalSolid = TVGL.Slice.GetUniformlySpacedCrossSections(
                tessellatedSolid, d, double.NaN, -1, thickness).ToList();


            var ReachableSolid = new List<Polygon>[OriginalSolid.Count];
            ReachableSolid[0] = OriginalSolid[0];
            
            var TempOverhang = new Dictionary<int, List<Polygon>>();

            int SwitchingLayer = 6; // Slice number where the overhang angle is 90 degrees
            var tList = new double[OriginalSolid.Count];
            for (int i = 1; i < OriginalSolid.Count(); i++)
            {
                if (OriginalSolid[i].Count == 0 || OriginalSolid[i - 1].Count == 0 || ReachableSolid[i - 1].Count == 0) 
                    break;
                else
                {
                    TempOverhang.Add(i-1, new List<Polygon>());
                    var t = thickness;
                    if (i < SwitchingLayer)
                    {
                        var angle = i * Math.PI * 0.5 / SwitchingLayer;
                        t *= Math.Tan(angle);
                    }
                    else if (i == SwitchingLayer)
                    {
                        t = double.NaN;
                    }
                    else if (i < (int)1.5 * SwitchingLayer)
                    {
                        var angle = Math.PI * (1 - (i * 0.5 / SwitchingLayer));
                        t /= Math.Tan(angle);
                    }
                    tList[i - 1] = t;

                    var tempLayer = new List<Polygon>();
                    for (int j = 0; j < OriginalSolid[i].Count(); j++)
                    {
                        var Empty = true;
                        for (int k = 0; k < ReachableSolid[i - 1].Count(); k++)
                        {
                            if (!OriginalSolid[i][j].GetPolygonInteraction(ReachableSolid[i - 1][k]).IntersectionWillBeEmpty())
                            {
                                Empty = false;
                                var poly = new List<Polygon>();
                                if (i < SwitchingLayer)
                                {

                                    var OffsettedPoly = PolygonOperations.OffsetRound(ReachableSolid[i - 1][k], t, 1e-7);
                                    foreach (var p in OffsettedPoly) 
                                    {
                                        tempLayer = PolygonOperations.UnionPolygons(
                                            PolygonOperations.Intersect(OriginalSolid[i][j], p),
                                            tempLayer);
                                        
                                    }
                                }
                                else
                                    tempLayer.Add(OriginalSolid[i][j]);
                                break;
                            }
                        }
                        if (Empty)
                            TempOverhang[i-1].Add(OriginalSolid[i][j]);
                    }
                    ReachableSolid[i] = tempLayer;
                    {
                        // Presenter.ShowAndHang(AccessibleRegion[i-1]);
                        // Presenter.ShowAndHang(SolidOriginal[i]);
                        // Presenter.ShowAndHang(AccessibleRegion[i]);

                    }
                }
               
            }


            #region WIP
            
            var NewTempOverhang = TempOverhang.ToList(); 
            NewTempOverhang.Reverse();
            
            var vect2 = new List<List<List<Vector3>>>();
            for (int i = 0; i < OriginalSolid.Count(); i++)
            {
                var tempv1 = new List<List<Vector3>>();
                if (ReachableSolid[i] != null)
                    foreach (var v1 in ReachableSolid[i])
                    {
                        var tempv2 = new List<Vector3>();

                        foreach (var v2 in v1.AllPolygons)
                        {
                            foreach (var v3 in v2.Path)
                                tempv2.Add(new Vector3(v3, thickness * i));
                        }
                        tempv1.Add(tempv2);
                    }
                vect2.Add(tempv1);
            }


            #region Collect overhang over 90 degrees
            /*
            foreach (var i in NewTempOverhang)
            {
                if (i.Key < SolidOriginal.Count && i.Value.Count > 0 && i.Key > 5)  // i.key should not be the last layer 
                {


                    //Presenter.ShowAndHang(AccessibleRegion[i.Key]);
                    //Presenter.ShowAndHang(AccessibleRegion[i.Key - 1]);
                    //Presenter.ShowAndHang(i.Value);
                    var connectedLayer = PolygonOperations.Subtract(AccessibleRegion[i.Key + 1], i.Value);
                    foreach (var poly1 in connectedLayer)
                    {
                        foreach (var poly2 in AccessibleRegion[i.Key])

                        {
                            if (poly1.GetPolygonInteraction(poly2).IntersectionWillBeEmpty())
                            {
                                var offsetpoly = PolygonOperations.OffsetRound(poly1, tList[i.Key], 1e-7);
                                var Polysubtract = PolygonOperations.Subtract(i.Value, offsetpoly);
                                AccessibleRegion[i.Key] = PolygonOperations.UnionPolygons(Polysubtract,
                                    AccessibleRegion[i.Key]);
                                break;
                            }
                        }
                    }

                    //Presenter.ShowAndHang(AccessibleRegion[i.Key]);
                }
            }
            */
            #endregion
            
            

            #endregion


            var vect = new List<List<List<Vector3>>>();
            for (int v = 0; v < OriginalSolid.Count(); v++)
            {
                var tempv1 = new List<List<Vector3>>();
                foreach (var v2 in OriginalSolid[v])
                {
                    var tempv2 = new List<Vector3>();
                    foreach (var v3 in v2.AllPolygons)
                    {
                        foreach (var v4 in v3.Path)
                            tempv2.Add(new Vector3(v4, thickness * v));
                    }
                    tempv1.Add(tempv2);
                }
                vect.Add(tempv1);
            }




            var ReachableSolidArea = 0.0;
            for (int i = 0; i < OriginalSolid.Count; i++) 
            {
                if (ReachableSolid[i] != null)
                    foreach (var poly in ReachableSolid[i])
                    {
                        ReachableSolidArea += poly.AllPolygons.Select(x => x.Area).Sum();
                    }
            }
            var OriginalSolidArea = 0.0;
            for (int i = 0; i < OriginalSolid.Count; i++)
            {
                if (OriginalSolid[i] != null)
                    foreach (var poly in OriginalSolid[i])
                    {
                        OriginalSolidArea += poly.AllPolygons.Select(x => x.Area).Sum();
                    }
            }

            //var volume2 = AccessibleRegion.Select(x => x.Select(p => p.Area).Sum()).Sum()*thickness;

            return Math.Abs(1 - (ReachableSolidArea / OriginalSolidArea));
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
        public static List<Polygon> GetFirstLayer(TessellatedSolid ts, double stepSize, Vector3 direction)
        {

            direction = direction.Normalize();
            var transform = direction.TransformToXYPlane(out _);
            var plane = new Plane(0.0, direction);
            //First, sort the vertices along the given axis. Duplicate distances are not important.
            var sortedVertices = ts.Vertices.OrderBy(v => v.Dot(direction)).ToArray();
            var firstDistance = sortedVertices[0].Dot(direction);
            var lastDistance = sortedVertices[^1].Dot(direction);
            var lengthAlongDir = lastDistance - firstDistance;
            stepSize = Math.Abs(stepSize);

            int numSlices = (int)(lengthAlongDir / stepSize);
            var startDistanceAlongDirection = firstDistance + 0.5 * stepSize;

            var result = new List<Polygon>();
            var currentEdges = new HashSet<Edge>();
            var nextDistance = sortedVertices.First().Dot(direction);
            var vIndex = 0;
            for (int step = 0; step < 1; step++)
            {
                var d = startDistanceAlongDirection + step * stepSize;
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
                    if (vIndex == sortedVertices.Length) break;
                    thisVertex = sortedVertices[vIndex];
                }
                if (needToOffset)
                    d += Math.Min(stepSize, sortedVertices[vIndex].Dot(direction) - d) / 10.0;
                plane.DistanceToOrigin = d;
                if (currentEdges.Any())
                    result = GetLoops(currentEdges.ToDictionary(ce => ce, ce =>
                       MiscFunctions.PointOnPlaneFromIntersectingLine(plane, ce.From.Coordinates, ce.To.Coordinates, out _)
                           .ConvertTo2DCoordinates(transform)), plane.Normal, plane.DistanceToOrigin, out _);
                else result = new List<Polygon>();
            }
            return result;


            /*
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
            */
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
