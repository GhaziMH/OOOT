﻿using System;
using System.Linq;
using StarMathLib;
using TVGL;
using OptimizationToolbox;
using System.Diagnostics;
using System.Collections.Generic;
using System.IO;
using CsvHelper;
using System.Threading.Tasks;
using SphericalOptimizationTest;

namespace SphericalOptimization
{
    class Program
    {
        static bool Run_CH = false;
        static bool RunNelderMead = false;
        static bool RunSingleObj = true;
        static bool showPlot = false;

        [STAThread]

        static void Main(string[] args)
        {

            var filename = FindParentDirectory("TestFiles").GetFiles("*").ToList();
            string DirData = Directory.GetParent(Directory.GetParent(filename[0].FullName).FullName).FullName + "\\Data\\";

            int counter = filename.Count;
            
            var Orientations = new List<double> { 0 };

            foreach (var anglei in Orientations)
            {
                for (int i = 1; i < counter; i++)
                {
                    SetUpFile(filename, i, counter, out TessellatedSolid ts, out double thickness);
                    double LargestArea = 0.0;
                    thickness = 0.5;
                    double MinAngle = 2.0; // in degree
                    var CH_Normals = GetCHNormal(ts, thickness, MinAngle, ref LargestArea);
                    if (Run_CH)
                    {
                        Run_convexHull(ts, thickness, LargestArea, filename, i, counter, DirData, anglei, CH_Normals);
                    }
                    if (RunNelderMead)
                    {
                        RunNelderMeadOpt(ts, thickness, LargestArea, filename, i, counter, DirData, anglei);
                    }
                    if (RunSingleObj) 
                    {
                        RunThetaAndPhi(ts, thickness, LargestArea, filename, i, counter, DirData, anglei);
                    }
                }

                
                #region Save Files, RunTime
                /*
                if (runobj)
                {


                    var n = Directory.GetFiles(@DirData, "*",
                        SearchOption.AllDirectories).Length;
                    string f;
                    if (n == 0)
                        f = "TestFile_001";
                    else
                    {
                        n++; n++;
                        if (n < 10)
                            f = "TestFile_00" + n;
                        else if (n < 100)
                            f = "TestFile_0" + n;
                        else
                            f = "TestFile_" + n;
                    }
                    int index = -1;
                    for (int i = 0; i < Data.Count; i++)
                    {
                        if (Data[i].Best == "1")
                        {
                            index++;
                            TVGL.IO.Open(filename[index].FullName, out TessellatedSolid ts);
                            double R = 0.0;
                            foreach (var v in ts.Vertices)
                            {
                                double DistSq = Math.Abs(ts.Center.DistanceSquared(v.Coordinates));
                                if (DistSq > R)
                                    R = DistSq;
                            }
                            double minHeight = (Math.Sqrt(R)) * 0.2;
                            double thickness = minHeight * 0.1;
                            if (showPlot)
                                PlotMixedSlices(ts, thickness, new Vector3(Data[i].X, Data[i].Y, Data[i].Z));
                        }
                    }

                    using (var writer = new StreamWriter(DirData + f + ".CSV"))
                    using (var csv = new CsvWriter(writer, System.Globalization.CultureInfo.InvariantCulture))
                    {
                        csv.WriteRecords(Data);
                    }
                }
                else
                {
                    using (var writer = new StreamWriter(DirData + "Timing2.CSV"))
                    using (var csv = new CsvWriter(writer, System.Globalization.CultureInfo.InvariantCulture))
                    {
                        csv.WriteRecords(Timing);
                    }
                }
                */
                #endregion
            }

            Console.ReadKey();
        }

        private static void SetUpFile(List<FileInfo> filename, int i, int counter,
             out TessellatedSolid ts, out double thickness)
        {
            TVGL.IO.Open(filename[i].FullName, out ts);
            Console.WriteLine("Current File " + (i + 1) + "/" + counter);
            //Presenter.ShowAndHang(ts);
            var angle1 = 0.1 * Math.PI;
            var angle2 = -2 * angle1;
            var angle3 = 3 * angle1;
            ts.Transform(Matrix4x4.CreateTranslation(-ts.Center.X, -ts.Center.Y, -ts.Center.Z));

            //Presenter.ShowAndHang(ts);
            double R = 0.0;
            foreach (var v in ts.Vertices)
            {
                double DistSq = Math.Abs(ts.Center.DistanceSquared(v.Coordinates));
                if (DistSq > R)
                    R = DistSq;
            }
            double minHeight = (Math.Sqrt(R)) * 0.2;
            thickness = minHeight * 0.1;
        }

        private static HashSet<Vector3> GetCHNormal(TessellatedSolid ts, double thickness, double MinAngle, ref double LargestArea)
        {
            HashSet<Vector3> CH_Normals = new HashSet<Vector3>();
            CH_Normals.Add(Vector3.UnitX);
            CH_Normals.Add(Vector3.UnitY);
            CH_Normals.Add(Vector3.UnitZ);
            CH_Normals.Add(-1 * Vector3.UnitX);
            CH_Normals.Add(-1 * Vector3.UnitY);
            CH_Normals.Add(-1 * Vector3.UnitZ);
            foreach (var f in ts.ConvexHull.Faces.OrderByDescending(x=>x.Area))
                if (f.Area > thickness * thickness)
                {
                    if (CH_Normals.Select(x => x.Dot(f.Normal)).Max() < Math.Cos(MinAngle * Math.PI / 180)) 
                        CH_Normals.Add(f.Normal);
                }


            
            foreach (var edge in ts.Edges)
            {
                {/*
                                var normal1 = edge.From.Faces.Select(x => x.Normal);
                                var normal2 = edge.To.Faces.Select(x => x.Normal);
                                var n = new Vector3(
                                    0.5 * (normal1.Select(x => x.X).Average() + normal2.Select(x => x.X).Average()),
                                    0.5 * (normal1.Select(x => x.Y).Average() + normal2.Select(x => x.Y).Average()),
                                    0.5 * (normal1.Select(x => x.Z).Average() + normal2.Select(x => x.Z).Average()));
                                */
                    var n = edge.OwnedFace.Normal + edge.OtherFace.Normal.Normalize();
                    if (CH_Normals.Select(x => x.Dot(n)).Max() < Math.Cos(MinAngle * Math.PI / 180)) 
                        CH_Normals.Add(n);
                }
            }
            var ch_vectors = new List<List<List<Vector3>>>();
            foreach (var v in CH_Normals)
            {
                ch_vectors.Add(new List<List<Vector3>> { new List<Vector3> { Vector3.Zero, v * 100 } });
            }
            var colors = new List<Color>();
            for (int ii = 0; ii < ts.ConvexHull.Faces.Count(); ii++)
                colors.Add(Color.HSVtoRGB(1, 1, 1, 0.2));
            //Presenter.ShowVertexPathsWithMultipleSolid(ch_vectors,new[] { new TessellatedSolid(ts.ConvexHull.Faces, true, false, null, colors), ts });
            foreach (var currentDir in CH_Normals)
            {
                var currentValue = 0.0;
                var currentlayer = Loops.GetFirstLayer(ts, thickness, currentDir);
                foreach (var poly in currentlayer)
                {
                    currentValue += poly.AllPolygons.Select(x => x.Area).Sum();
                }
                //var currenttestingLayer = SphericalOptimizationTest.QuadricSlicer.GetUniformlySpacedCrossSections(ts, currentDir, 1E-6, -1, thickness)[0].Select(x => x.Area).Sum();
                if (LargestArea < currentValue)
                {
                    LargestArea = currentValue;
                }
            }
            return CH_Normals;
        }

        private static void Run_convexHull(TessellatedSolid ts, double thickness, double LargestArea, List<FileInfo> filename, int i, int counter,
            string DirData, double anglei, HashSet<Vector3> CH_Normals)
        {
            var ConvexHullList = CH_Normals.ToList();
            for (int k = 2; k < 4; k++)
            {
                var DataCSV = new List<HeatmapData>();
                var Duration = new Stopwatch();
                Duration.Start();
                var objfn = new SphericalOptimizationTest.FirstLayer(ts, thickness, LargestArea, k); // obj fun
#if RELEASE
                Parallel.For(0, ConvexHullList.Count, ii =>
#else
                for (int ii = 0; ii < ConvexHullList.Count; ii++)
#endif
                {
                    var fun = objfn.calculate(new[] { ConvexHullList[ii].X, ConvexHullList[ii].Y, ConvexHullList[ii].Z });
                    DataCSV.Add(new HeatmapData { X = ConvexHullList[ii].X, Y = ConvexHullList[ii].Y, Z = ConvexHullList[ii].Z, fun = fun });
                }
#if RELEASE
                );
#endif

                var best = DataCSV[0];
                var Repeatedbest = new List<HeatmapData>();
                foreach (var v in DataCSV)
                {
                    if (v.fun < best.fun)
                        best = v;
                    else if (Math.Abs(v.fun - best.fun) <= 1e-12 && v != DataCSV[0])
                    {
                        if (Repeatedbest.Count == 0)
                            Repeatedbest.Add(v);
                        else
                        {
                            foreach (var x in Repeatedbest)
                            {
                                if (Math.Abs(x.fun - best.fun) > 1e-12)
                                {
                                    Repeatedbest = new List<HeatmapData> { v };
                                    break;
                                }
                            }
                            Repeatedbest.Add(v);
                        }
                    }
                }
                    

                Duration.Stop();
                Console.WriteLine("Current file " + (i + 1) + " / " + counter);
                Console.WriteLine("Current file name: " + filename[i].Name);
                Console.WriteLine("Search Method: Convex Hull" );
                Console.WriteLine("Current angle: " + anglei + "*pi");
                Console.WriteLine("optimizing..., X" + k);
                Console.WriteLine("NumEvals = " + CH_Normals.Count);
                Console.WriteLine("X* = {" + best.X + "," + best.Y + "," + best.Z + "}");
                if (Repeatedbest.Count > 0) 
                {
                    for (int j = 0; j < Repeatedbest.Count; j++)
                        Console.WriteLine("X* (Direction with same f* value) = {" + Repeatedbest[j].X + "," + Repeatedbest[j].Y + "," + Repeatedbest[j].Z + "}");
                }
                Console.WriteLine("f* = {" + best.fun + "}");
                Console.WriteLine("RunTime = " + Duration.Elapsed);
                Console.WriteLine("#######################################################");

                using (var writer = new StreamWriter(DirData + filename[i].Name + " objfn X" + k + " CH " + ".CSV"))
                using (var csv = new CsvWriter(writer, System.Globalization.CultureInfo.InvariantCulture))
                {
                    csv.WriteRecords(DataCSV);

                }
            }
        }

        private static void RunThetaAndPhi(TessellatedSolid ts, double thickness, double LargestArea, List<FileInfo> filename, int i, int counter,
            string DirData, double anglei)
        {
            for (int k = 1; k < 2; k++)
            {
                var DataCSV = new List<HeatmapData>();
                var Duration = new Stopwatch();
                Duration.Start();
                var map = new double[46, 91];
                var objfn = new SphericalOptimizationTest.FirstLayer(ts, thickness, LargestArea, k); // obj fun
#if RELEASE
                    Parallel.For(0, map.GetLength(0), iTheta =>
#else
                for (int iTheta = 0; iTheta < map.GetLength(0); iTheta++)
#endif
                {

                    var theta = iTheta * (Math.PI / (map.GetLength(0) - 1));
                    for (int iPhi = 0; iPhi < map.GetLength(1); iPhi++)
                    {
                        var phi = iPhi * (2 * Math.PI / (map.GetLength(1) - 1));
                        var x = Math.Sin(theta) * Math.Cos(phi);
                        var y = Math.Sin(theta) * Math.Sin(phi);
                        var z = Math.Cos(theta);
                        map[iTheta, iPhi] = objfn.calculate(new[] { x, y, z });
                        //DataCSV.Add(new HeatmapData { Theta= theta, Phi = phi, OptValue = map[iTheta, iPhi] });
                    }
                }
#if RELEASE
                    );
#endif
                Duration.Stop();
                Console.WriteLine("Current file " + (i + 1) + " / " + counter);
                Console.WriteLine("Current file name: " + filename[i].Name);
                Console.WriteLine("Search Method: Theta & Phi Angles");
                Console.WriteLine("Current angle: " + anglei + "*pi");
                Console.WriteLine("optimizing..., X" + k);
                Console.WriteLine("RunTime = " + Duration.Elapsed);
                Console.WriteLine("#######################################################");

                using (var writer = new StreamWriter(DirData + filename[i].Name + " objfn X" + k + " angle " + anglei + ".CSV"))
                //using (var csv = new CsvWriter(writer, System.Globalization.CultureInfo.InvariantCulture))
                {
                    //csv.WriteRecords(DataCSV);
                    for (int iRow = 0; iRow < map.GetLength(0); iRow++)
                        writer.WriteLine(string.Join(',', map.GetRow(iRow)));
                }
            }

        }

        private static void RunNelderMeadOpt(TessellatedSolid ts, double thickness, double LargestArea,List<FileInfo> filename, int i, int counter,
            string DirData, double anglei)
        {
            var Data = new List<SaveData>();
            if (RunSingleObj)
            {
                for (int j = 0; j < 4; j++)
                {
                    var Duration = new Stopwatch();
                    Duration.Start();
                    var optMethod = new SphericalOptimization();
                    optMethod.Add(new SphericalOptimizationTest.FirstLayer(ts, thickness, LargestArea, j)); // obj fun
                    optMethod.Add(new MaxSpanInPopulationConvergence(1e-3));
                    double[] xInit = new[] { 0.0, 0.0 };//,100};
                    var fStar = optMethod.Run(out var xStar, xInit);

                    Duration.Stop();
                    Console.WriteLine("Current file " + (i + 1) + " / " + counter);
                    Console.WriteLine("Current file name: " + filename[i].Name);
                    Console.WriteLine("Search Method: Nelder-Mead");
                    Console.WriteLine("Current angle: " + anglei + "*pi");
                    Console.WriteLine("optimizing..., X" + j);
                    Console.WriteLine("Convergence Declared by " + optMethod.ConvergenceDeclaredByTypeString);
                    Console.WriteLine("X* = " + optMethod.sortedBest.Values[0].MakePrintString());
                    Console.WriteLine("F* = " + optMethod.sortedBest.Keys[0], 1);
                    Console.WriteLine("NumEvals = " + optMethod.numEvals);
                    Console.WriteLine("RunTime = " + Duration.Elapsed);
                    Console.WriteLine("#######################################################");

                    for (int k = 0; k < optMethod.sortedBest.Count; k++)
                    {
                        Data.Add(new SaveData
                        {
                            Filename = filename[i].Name,
                            Best = (k + 1).ToString(),
                            Function = optMethod.sortedBest.Keys[k],
                            NumEvals = (int)optMethod.numEvals,
                            X = optMethod.sortedBest.Values[k][0],
                            Y = optMethod.sortedBest.Values[k][1],
                            Z = optMethod.sortedBest.Values[k][2],
                            RunTime_sec = Duration.Elapsed,
                            objf = j
                        });
                    }

                }
                using (var writer = new StreamWriter(DirData + "obj X0_4 " + filename[i].Name + anglei + ".CSV"))
                using (var csv = new CsvWriter(writer, System.Globalization.CultureInfo.InvariantCulture))
                {
                    csv.WriteRecords(Data);
                }



            }
            else
            {
#region SphericalOptimization
                var Duration = new Stopwatch();
                Duration.Start();
                var optMethod = new SphericalOptimization();

                optMethod.Add(new SphericalOptimizationTest.FirstLayer(ts, thickness, LargestArea)); // obj fun
                optMethod.Add(new MaxSpanInPopulationConvergence(1e-3));

                // Let us start the search from a specific point. 
                double[] xInit = new[] { 0.0, 0.0 };//,100};
                var fStar = optMethod.Run(out var xStar, xInit);
                //* output is provided from the optimization. Since the optMethod is an object
                // * we can probe it following the run to get at important data like how the 
                //* process converged. 


                //optMethod = new SphericalOptimization();
                //optMethod.Add(new SphericalOptimizationTest.FirstLayer(ts, thickness, LargestArea));
                //var f1limitConstraint = new SphericalOptimizationTest.MaxVolumeInequality(ts, thickness, LargestArea, fStar * 1.1);
                //optMethod.Add(f1limitConstraint);
                //optMethod.Add(new squaredExteriorPenalty(optMethod, 10.0));
                //optMethod.Add(new MaxSpanInPopulationConvergence(1e-3));
                //var fStar2 = optMethod.Run(out var xStar2, xInit);
                //var g = new List<double>();
                //foreach (var v in optMethod.sortedBest.Values)
                //   g.Add(f1limitConstraint.calculate(v));



                //dir_x.Add(xStar.)
                Duration.Stop();
                Console.WriteLine("Current file " + (i + 1) + " / " + counter);
                Console.WriteLine("Current file name: " + filename[i].Name);
                Console.WriteLine("Search Method: Nelder-Mead");
                Console.WriteLine("Current angle: " + anglei + "*pi");
                Console.WriteLine("optimizing...");
                Console.WriteLine("Convergence Declared by " + optMethod.ConvergenceDeclaredByTypeString);
                Console.WriteLine("X* = " + optMethod.sortedBest.Values[0].MakePrintString());
                Console.WriteLine("F* = " + optMethod.sortedBest.Keys[0], 1);
                Console.WriteLine("NumEvals = " + optMethod.numEvals);
                Console.WriteLine("RunTime = " + Duration.Elapsed);
                Console.WriteLine("#######################################################");
                //* Since there is no randomness in the process, you should get the following
                // * response:
                // * No inequalities specified.
                // * Convergence Declared by ToKnownBestFConvergence
                // * X* = {   1.079   ,  1.159    }
                // * F* = 0.00772036716239199
                // * NumEvals = 245
                //

                //var CurrentData = new List<SaveData>();
                for (int j = 0; j < optMethod.sortedBest.Count; j++)
                {
                    Data.Add(new SaveData
                    {
                        Filename = filename[i].Name,
                        Best = (j + 1).ToString(),
                        Function = optMethod.sortedBest.Keys[j],
                        NumEvals = (int)optMethod.numEvals,
                        X = optMethod.sortedBest.Values[j][0],
                        Y = optMethod.sortedBest.Values[j][1],
                        Z = optMethod.sortedBest.Values[j][2],
                        RunTime_sec = Duration.Elapsed,
                        objf = 0
                    });
                }
                using (var writer = new StreamWriter(DirData + filename[i].Name + "objfun X0 angle " + anglei + ".CSV"))
                using (var csv = new CsvWriter(writer, System.Globalization.CultureInfo.InvariantCulture))
                {
                    csv.WriteRecords(Data);
                }

                if (showPlot)
                {/*
                            Vector3 d = new Vector3(optMethod.sortedBest.Values[0]);
                            if (TVGL.EqualityExtensions.IsNegligible(d.Dot(Vector3.UnitZ), 1E-4))
                            {
                                if (EqualityExtensions.IsNegligible(d.X, 1E-2))
                                    d = new Vector3(0, d.Y, d.Z);
                                if (EqualityExtensions.IsNegligible(d.Y, 1E-2))
                                    d = new Vector3(d.X, 0, d.Z);
                                if (EqualityExtensions.IsNegligible(d.Z, 1E-2))
                                    d = new Vector3(d.X, d.Y, 0);
                            }
                            var anglex = Math.Atan2(d.Y, d.Z);
                            var angley = Math.Atan2(d.X, d.Z);
                            ts.Transform(Matrix4x4.CreateRotationX(anglex));
                            ts.Transform(Matrix4x4.CreateRotationY(angley));
                            ts.Transform(new Matrix4x4(Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ, new Vector3(-ts.XMin, -ts.YMin, -ts.ZMin)));
                            */
                    //ts.Transform(Matrix4x4.CreateFromYawPitchRoll(-angle1, -angle2, 0));
                    var Xstar = new Vector3(optMethod.sortedBest.Values[0]);
                    var minV = 0.0; var maxV = 0.0;
                    foreach (var vert in ts.Vertices)
                    {
                        var Dot = vert.Dot(Xstar);
                        if (Dot > maxV)
                            maxV = Dot;
                        if (Dot < minV)
                            minV = Dot;
                    }
                    var listVect = new List<Vector3> { Xstar * minV, Xstar * (maxV + 100) };
                    Presenter.ShowVertexPathsWithSolids(listVect, new[] { ts });
                    PlotMixedSlices(ts, thickness, new Vector3(optMethod.sortedBest.Values[0]));
                    //PlotDirections(optMethod.sortedBest.Keys, optMethod.sortedBest.Values, ts, thickness);
                }

#endregion
            }
        }

        static void PlotMixedSlices(TessellatedSolid ts, double thickness, Vector3 d)
        {
            d = d.Normalize();
            
            var PlanarSlices = TVGL.Slice.GetUniformlySpacedCrossSections(ts,Vector3.UnitZ, double.NaN, -1, thickness).ToList();

            var Slices = new List<List<List<Vector3>>>();
            for(int i = 0; i < 5; i++)
            {
                var CurrentLayer = new List<List<Vector3>>();
                for(int j= 0; j < PlanarSlices[i].Count; j++)
                {
                    var CurrentPoly = new List<Vector3>();

                    for (int k = 0; k < PlanarSlices[i][j].Path.Count; k++) 
                    {
                        CurrentPoly.Add(new Vector3(PlanarSlices[i][j].Path[k], i * thickness));
                    }
                    CurrentLayer.Add(CurrentPoly);
                }
                Slices.Add(CurrentLayer);
            }
            Vector3 center = new Vector3(ts.Center.X, ts.Center.Y, -0.1*ts.Bounds[1].Z);
            var sph = new Sphere(center, 10, true);

            //ts.SliceOnFlatAsSingleSolids(new Plane(new Vector3(10, 10, thickness * 5), Vector3.UnitZ), out var TS1, out var TS2);
            
            //Presenter.ShowAndHang(new[] { TS1, TS2 });
            var sphSlices = SphericalOptimizationTest.QuadricSlicer.GetSphericalSections(ts, sph, thickness).Item1.ToList();
            sphSlices.RemoveRange(0, 5);

            Slices.AddRange(sphSlices);
            Presenter.ShowVertexPathsWithSolids(Slices, null);
            

        }
        static void PlotDirections(IList<double> keys, IList<double[]> values, TessellatedSolid ts, double thickness)
        {
            var d = Vector3.UnitZ;
            var numToShow = keys.Count;
            var max = keys[0];
            var rank = Enumerable.Range(0, keys.Count);
            var rankFraction = rank.Select(x => x / (double)numToShow).ToList();
            //rankFraction.Reverse();
            var plotVertices = new List<List<Vertex>>();
            var plotvectors = new List<List<Vector3>>();
            var colors = new List<TVGL.Color>();
            for (int i = 0; i < numToShow; i++)
            {
                plotVertices.Add(new List<Vertex> { new Vertex(Vector3.Zero), new Vertex(0.03 * (19 + (keys[i] / max)) * new Vector3(values[i])) });
                plotvectors.Add(new List<Vector3> { Vector3.Zero, 0.03 * (19 + (keys[i] / max)) * new Vector3(values[i]) });
                //colors.Add(TVGL.Color.HSVtoRGB((float)rankFraction[i], 1, 1));
                colors.Add(TVGL.Color.HSVtoRGB((float)(keys[i] / max), 1, 1));
                //Console.WriteLine(keys[i]);
            }
            //var xxx = plotVertices.Select(x => new List<Vector3> { Vector3.Zero, x.Coordinates }).ToList();
            Presenter.ShowAndHang(ts);
            Presenter.ShowVertexPaths(plotVertices,null,0, colors);
            {
                var lineVisuals = Presenter.GetVertexPaths(plotvectors, thickness, colors, plotvectors.Select(x => false));
                var vm = new Window3DPlotViewModel();
                vm.Add(lineVisuals);
                //vm.Add(solids.Where(s => s != null).SelectMany(s => ConvertTessellatedSolidToMGM3D((TessellatedSolid)s)));
                
                var window = new Window3DPlot(vm);

                window.ShowDialog();
            }
            
            //Presenter.ShowVertexPaths(ts.GetUniformlySpacedCrossSections(d,)

            Presenter.ShowAndHang(SphericalOptimizationTest.ShowBestShape.ShowShape(new Vector3(d), ts, thickness));
        }
        public class SaveData
        {
            public String Filename { set; get; }
            public string Best {  set; get; }
            public double Function { set; get; }
            public int NumEvals { set; get; }
            public double X { set; get; }
            public double Y { set; get; }
            public double Z { set; get; }
            public TimeSpan RunTime_sec { set; get; }
            public double objf { set; get; }
        }

        public class HeatmapData
        {
            public double X { set; get; }
            public double Y { set; get; }
            public double Z { set; get; }
            public double fun { set; get; }
        }

        static DirectoryInfo FindParentDirectory(string targetFolder)
        {
            var dir = new DirectoryInfo(".");
            while (!Directory.Exists(Path.Combine(dir.FullName, targetFolder)))
            {
                if (dir == null) throw new FileNotFoundException("Target folder not found", targetFolder);
                dir = dir.Parent;
            }
            return new DirectoryInfo(Path.Combine(dir.FullName, targetFolder));
        }

    }
}



