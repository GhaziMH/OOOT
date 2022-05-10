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
            var Data = new List<SaveData>();
            var AllFiles = Directory.GetFiles(@"C:\Users\galon\source\repos\OOOT\TestFiles", "*.stl",
                SearchOption.AllDirectories).ToList();
            int counter= AllFiles.Count;
            //for(int i = 0; i < counter-2; i++)
             //   AllFiles.RemoveAt(0);
            foreach (var filename in AllFiles) 
            {
                TVGL.IO.Open(filename, out TessellatedSolid ts);
                Console.WriteLine("Current File " + (AllFiles.IndexOf(filename)+1) + " / " + counter);
                Console.WriteLine("optimizing...");
                double R = 0.0;
                foreach (var v in ts.Vertices)
                {
                    double DistSq = Math.Abs(ts.Center.DistanceSquared(v.Coordinates));
                    if (DistSq > R)
                        R = DistSq;
                }
                double minHeight = (Math.Sqrt(R)) * 0.2;
                double thickness = minHeight * 0.1;
                // 10% of the total length is considered as a min height from printing bed

                //Presenter.ShowAndHang(ts);
                var optMethod = new SphericalOptimization();

                //ts.Transform(TVGL.Numerics.Matrix4x4.CreateRotationY(1.5));
                //ts.Transform(TVGL.Numerics.Matrix4x4.CreateRotationZ(1.25));
                optMethod.Add(new SphericalOptimizationTest.FloorAndWallFaceScore2(ts,thickness)); // obj fun
                optMethod.Add(new MaxSpanInPopulationConvergence(1e-1));
                /* Let us start the search from a specific point. */
                double[] xInit = new[] { 0.0, 0.0 };//,100};
                var fStar = optMethod.Run(out var xStar, xInit);
                /* output is provided from the optimization. Since the optMethod is an object
                 * we can probe it following the run to get at important data like how the 
                 * process converged. */

                f_values.Add(fStar);
                //dir_x.Add(xStar.)
                Console.WriteLine("Convergence Declared by " + optMethod.ConvergenceDeclaredByTypeString);
                Console.WriteLine("X* = " + xStar.MakePrintString());
                Console.WriteLine("F* = " + fStar, 1);
                Console.WriteLine("NumEvals = " + optMethod.numEvals);
                /* Since there is no randomness in the process, you should get the following
                 * response:
                 * No inequalities specified.
                 * Convergence Declared by ToKnownBestFConvergence
                 * X* = {   1.079   ,  1.159    }
                 * F* = 0.00772036716239199
                 * NumEvals = 245
                 */
                Data.Add(new SaveData
                {
                    File = filename,
                    Cost = fStar,
                    NumEvals = (int)optMethod.numEvals,
                    Best_Dir_X = xStar[0],
                    Best_Dir_Y = xStar[1],
                    Best_Dir_Z = xStar[2],
                });
                //PlotDirections(optMethod.sortedBest.Keys, optMethod.sortedBest.Values, ts, thickness, xStar);

            }
            using (var writer = new StreamWriter(@"C:\\Users\\galon\\source\\repos\\OOOT\\Data\\BestOrientations.CSV"))
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
            var plotVertices = new List<Vertex>();
            var colors = new List<TVGL.Color>();
            for (int i = 0; i < numToShow; i++)
            {
                plotVertices.Add(new Vertex(0.03 * (19 + (keys[i] / max)) * (new TVGL.Vector3(values[i]))));
                //colors.Add(TVGL.Color.HSVtoRGB((float)rankFraction[i], 1, 1));
                colors.Add(TVGL.Color.HSVtoRGB((float)(keys[i] / max), 1, 1));
                Console.WriteLine(keys[i]);
            }
            IEnumerable<IEnumerable<Vector3>> xxx = plotVertices.Select(x => new List<Vector3> { Vector3.Zero, x.Coordinates });
            //Presenter.ShowAndHang(ts);
            //Presenter.ShowVertexPaths(xxx,null,1);
            //Presenter.ShowAndHang(SphericalOptimizationTest.ShowBestShape.ShowShape(new Vector3(d), ts, thickness));
        }

        public class SaveData
        {
            public String File { set; get; }
            public double Cost { set; get; }
            public int NumEvals { set; get; }
            public double Best_Dir_X { set; get; }
            public double Best_Dir_Y { set; get; }
            public double Best_Dir_Z { set; get; }

        }
    }
}



