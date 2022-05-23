using System;
using System.Linq;
using StarMathLib;
using TVGL;
using OptimizationToolbox;
using System.Diagnostics;
using System.Collections.Generic;
using System.IO;
using CsvHelper;

namespace SphericalOptimization
{
    class Program
    {
        [STAThread]
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Usage", "CA2252:This API requires opting into preview features", Justification = "<Pending>")]
        static void Main(string[] args)
        {
            //Parameters.Verbosity = OptimizationToolbox.VerbosityLevels.Everything;
            // this next line is to set the Debug statements from OOOT to the Console.
            //Trace.Listeners.Add(new TextWriterTraceListener(Console.Out));

            double maxAngle = Math.PI * 3 / 4;
            var f_values = new List<double>();
            var dir_x = new List<double>();
            var dir_y = new List<double>();
            var dir_z = new List<double>();
            var filename = FindParentDirectory("TestFiles").GetFiles("*").ToList();
            string DirData = Directory.GetParent(Directory.GetParent(filename[0].FullName).FullName).FullName + "\\Data\\";
            

            //var filename = Directory.GetFiles(@"C:\Users\galon\source\repos\OOOT\TestFiles", "*.stl",
            //    SearchOption.AllDirectories).ToList();

            int counter = filename.Count;
            //for(int i = 0; i < counter-2; i++)
            //   AllFiles.RemoveAt(0);
            

            var Data = new List<SaveData>();

            for (int i = 0; i < counter; i++)
            {
                var Duration = new Stopwatch();
                Duration.Start();
                TVGL.IO.Open(filename[i].FullName, out TessellatedSolid ts);
                string name = filename[i].Name;
                Console.WriteLine("Current file name: " + name);
                Console.WriteLine("Current file " + (i + 1) + " / " + counter);
                Console.WriteLine("optimizing...");
                /*
                ts.Transform(new Matrix4x4(1.0, 0.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0, 0.0,
                                  0.0, 0.0, 1.0, 0.0,
                                  -ts.Center.X, -ts.Center.Y, -ts.Center.Z, 1));
                */
                double R = 0.0;
                foreach (var v in ts.Vertices)
                {
                    double DistSq = Math.Abs(ts.Center.DistanceSquared(v.Coordinates));
                    if (DistSq > R)
                        R = DistSq;
                }
                double minHeight = (Math.Sqrt(R)) * 0.2;
                double thickness = minHeight * 0.1;
                var LargestArea = 0.0;
                var DirLargestArea = Vector3.Zero;
                var CH_Normals = ts.ConvexHull.Faces.Select(x => x.Normal).ToHashSet();
                CH_Normals.Add(Vector3.UnitX);
                CH_Normals.Add(Vector3.UnitY);
                CH_Normals.Add(Vector3.UnitZ);
                CH_Normals.Add(-1 * Vector3.UnitX);
                CH_Normals.Add(-1 * Vector3.UnitY);
                CH_Normals.Add(-1 * Vector3.UnitZ);

                foreach (var currentDir in CH_Normals)
                {
                    var currenttestingLayer = Slice.GetUniformlySpacedCrossSections(ts, currentDir, 1E-6, -1, thickness, true)[0].Select(x => x.Area).Sum();
                    if (LargestArea < currenttestingLayer)
                    {
                        LargestArea = currenttestingLayer;
                        DirLargestArea = currentDir;
                    }
                }

                // 10% of the total length is considered as a min height from printing bed

                //Presenter.ShowAndHang(ts);
                var optMethod = new SphericalOptimization();

                //ts.Transform(TVGL.Numerics.Matrix4x4.CreateRotationY(1.5));
                //ts.Transform(TVGL.Numerics.Matrix4x4.CreateRotationZ(1.25));
                optMethod.Add(new SphericalOptimizationTest.FloorAndWallFaceScore3(ts, thickness,LargestArea)); // obj fun
                optMethod.Add(new MaxSpanInPopulationConvergence(1e-1));

                /* Let us start the search from a specific point. */
                double[] xInit = new[] { 0.0, 0.0 };//,100};
                var fStar = optMethod.Run(out var xStar, xInit);
                /* output is provided from the optimization. Since the optMethod is an object
                 * we can probe it following the run to get at important data like how the 
                 * process converged. */

                f_values.Add(fStar);
                //dir_x.Add(xStar.)
                Duration.Stop(); var Duration2 = new Stopwatch();Duration2.Start();
                Console.WriteLine("Convergence Declared by " + optMethod.ConvergenceDeclaredByTypeString);
                Console.WriteLine("X* = " + xStar.MakePrintString());
                Console.WriteLine("F* = " + fStar, 1);
                Console.WriteLine("NumEvals = " + optMethod.numEvals);
                Console.WriteLine("RunTime = " + Duration.Elapsed);
                Console.WriteLine("#######################################################");
                /* Since there is no randomness in the process, you should get the following
                 * response:
                 * No inequalities specified.
                 * Convergence Declared by ToKnownBestFConvergence
                 * X* = {   1.079   ,  1.159    }
                 * F* = 0.00772036716239199
                 * NumEvals = 245
                 */

                //var CurrentData = new List<SaveData>();
                for (int j = 0; j < optMethod.sortedBest.Count; j++)
                {
                    Data.Add(new SaveData
                    {
                        Filename = name,
                        Best = (j + 1).ToString(),
                        Function = optMethod.sortedBest.Keys[j],
                        NumEvals = (int)optMethod.numEvals,
                        X = optMethod.sortedBest.Values[j][0],
                        Y = optMethod.sortedBest.Values[j][1],
                        Z = optMethod.sortedBest.Values[j][2],
                        RunTime_sec = Duration.Elapsed

                    });
                }

               

                /*
                 * Data.Add(new SaveData
                {
                    Filename = name,
                    Best = "First Best",
                    Function = fStar,
                    NumEvals = (int)optMethod.numEvals,
                    X = xStar[0],
                    Y = xStar[1],
                    Z = xStar[2],
                    RunTime_sec = Duration.Elapsed
                });
                for (int j = 1; j < optMethod.sortedBest.Count; j++)
                {
                    if (Math.Abs(optMethod.sortedBest.Keys[j] - optMethod.sortedBest.Keys[0]) <= 1e-10)
                    {
                        Data.Add(new SaveData
                        {
                            Filename = "",
                            Best = "Same cost, different direction",
                            Function = optMethod.sortedBest.Keys[j],
                            NumEvals = (int)optMethod.numEvals,
                            X = optMethod.sortedBest.Values[j][0],
                            Y = optMethod.sortedBest.Values[j][1],
                            Z = optMethod.sortedBest.Values[j][2],
                            RunTime_sec = Duration2.Elapsed
                        });
                    }
                    else
                    {
                        Data.Add(new SaveData
                        {
                            Filename = "",
                            Best = "Second Best",
                            Function = optMethod.sortedBest.Keys[j],
                            NumEvals = (int)optMethod.numEvals,
                            X = optMethod.sortedBest.Values[j][0],
                            Y = optMethod.sortedBest.Values[j][1],
                            Z = optMethod.sortedBest.Values[j][2],
                            RunTime_sec = Duration2.Elapsed
                        });
                        break;
                    }
                }
                
                 */
                //PlotDirections(optMethod.sortedBest.Keys, optMethod.sortedBest.Values, ts, thickness, xStar);

            }
            var n = Directory.GetFiles(@DirData, "*",
                SearchOption.AllDirectories).Length;
            string f;
            if (n == 0)
                f = "TestFile_001";
            else
            {
                n++;
                if (n < 10)
                    f = "TestFile_00" + n;
                else if (n < 100)
                    f = "TestFile_0" + n;
                else
                    f = "TestFile_" + n;
            }

            using (var writer = new StreamWriter(DirData + f + ".CSV"))
            using (var csv = new CsvWriter(writer, System.Globalization.CultureInfo.InvariantCulture))
            {
                csv.WriteRecords(Data);
            }

            Console.ReadKey();
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Usage", "CA2252:This API requires opting into preview features", Justification = "<Pending>")]
        static void PlotDirections(IList<double> keys, IList<double[]> values, TessellatedSolid ts, double thickness, double [] d)
        {
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
            
            //Presenter.ShowAndHang(SphericalOptimizationTest.ShowBestShape.ShowShape(new Vector3(d), ts, thickness));
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



