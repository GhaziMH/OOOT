using SharpDX.Direct3D11;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;
using TVGL;

namespace SphericalOptimizationTest
{
    public static class QuadricSlicer
    {
       
        #region Cylindrical Slicing

        public static (List<List<List<Vector3>>>, List<List<List<PolygonalFace>>>, double[]) GetCylSectionswRadius(TessellatedSolid ts, Cylinder cyl,
           double thickness)
        {
            cyl.Axis = cyl.Axis.Normalize();
            var maxRadius = MaxcylRadius(ts, cyl);
            int numberOfLayers = (int)Math.Ceiling((maxRadius.Item1 - maxRadius.Item2) / thickness);
            var AllPolygonLayers = new List<List<Vector3>>[numberOfLayers];
            var AllPolygonLayersFace = new List<List<PolygonalFace>>[numberOfLayers];
            var radius = new double[numberOfLayers];


#if PARALLEL
            Parallel.For(0, numberOfLayers, i =>
#else
            for (int i = 0; i < numberOfLayers; i++)
#endif
            {
                cyl.Radius = i >= numberOfLayers ?
                    maxRadius.Item1 : i * thickness + maxRadius.Item2;
                var edgeStatus = GetEdgeStatus(ts, cyl);
                var faceStatus = GetFaceStatus(ts, cyl, edgeStatus);
                (AllPolygonLayers[i], AllPolygonLayersFace[i]) = MakeLoops(edgeStatus, faceStatus);
                radius[i] = cyl.Radius;
            }
#if PARALLEL
            );
#endif
            List<List<List<Vector3>>> Layers = AllPolygonLayers.Where(x => x != null && x.Count() > 0).ToList();
            List<List<List<PolygonalFace>>> Faces = AllPolygonLayersFace.Where(x => x != null && x.Count() > 0).ToList();
            return (Layers, Faces, radius);
        }




        /// <summary>
        ///  This method slices TS using Layer's thickness. 
        ///  This method slices a tessellated solid using cylindrical surface.
        /// </summary>
        /// <param name="ts">The tessellated solid.</param>
        /// <param name="cyl">A cylinder.</param>
        /// <param name="thickness">Layer's thickness.</param>
        /// <returns>List of polygons and faces of the intersection between tessellated solid and cylinder.</returns>
        public static (List<List<List<Vector3>>>, List<List<List<PolygonalFace>>>) GetCylSections(TessellatedSolid ts, Cylinder cyl,
            double thickness, bool epsilonOnOff = false, double epsilon = 0.1)
        {
            cyl.Axis = cyl.Axis.Normalize();
            var maxRadius = MaxcylRadius(ts, cyl).Item1;
            int numberOfLayers = (int)Math.Ceiling(maxRadius / thickness);
            var AllPolygonLayers = new List<List<Vector3>>[numberOfLayers];
            var AllPolygonLayersFace = new List<List<PolygonalFace>>[numberOfLayers];
            var offset_value = 0.0;
            var t = thickness;
            (AllPolygonLayers[0], AllPolygonLayersFace[0]) = GetFirstLayer(ts, numberOfLayers, maxRadius,
                ref cyl, ref offset_value, ref t);
            numberOfLayers = (int)Math.Ceiling((maxRadius - offset_value) / thickness);
            if (epsilonOnOff)
            {
                var newpolygonList = new List<List<Vector3>>();
                for (var j = 0; j < AllPolygonLayers[0].Count; j++)
                {
                    newpolygonList.Add(GetEllipticalSegment(AllPolygonLayers[0][j], cyl, epsilon, AllPolygonLayersFace[0][j]));
                }
                AllPolygonLayers[0] = newpolygonList;
            }


#if PARALLEL
            Parallel.For(1, numberOfLayers, i =>
#else
            for (int i = 1; i < numberOfLayers; i++)
#endif
            {
                cyl.Radius = i >= numberOfLayers ?
                    maxRadius : i * thickness + offset_value;
                var edgeStatus = GetEdgeStatus(ts, cyl);
                var faceStatus = GetFaceStatus(ts, cyl, edgeStatus);
                (AllPolygonLayers[i], AllPolygonLayersFace[i]) = MakeLoops(edgeStatus, faceStatus);

                if (epsilonOnOff)
                {
                    var newpolygonList = new List<List<Vector3>>();
                    for (var j = 0; j < AllPolygonLayers[i].Count; j++)
                    {
                        newpolygonList.Add(GetEllipticalSegment(AllPolygonLayers[i][j], cyl, epsilon, AllPolygonLayersFace[i][j]));
                    }
                    AllPolygonLayers[i] = newpolygonList;
                }
            }
#if PARALLEL
            );
#endif
            List<List<List<Vector3>>> Layers = AllPolygonLayers.Where(x => x != null && x.Count() > 0).ToList();
            List<List<List<PolygonalFace>>> Faces = AllPolygonLayersFace.Where(x => x != null && x.Count() > 0).ToList();
            return (Layers, Faces);
        }


        /// <summary>
        /// This method slices TS using number of layers 
        /// This method slices a tessellated solid using cylindrical surface.
        /// </summary>
        /// <param name="ts">The tessellated solid.</param>
        /// <param name="cyl">A cylinder.</param>
        /// <param name="numberOfLayers">Number Of Layers.</param>
        /// <returns>List of polygons and faces of the intersection between tessellated solid and cylinder.</returns> 
        public static (List<List<Vector3>>[], List<List<PolygonalFace>>[]) GetCylSections(TessellatedSolid ts, Cylinder cyl,
            int numberOfLayers, bool epsilonOnOff = false, double epsilon = 0.1)
        {
            epsilon = epsilon * epsilon;
            cyl.Axis = cyl.Axis.Normalize();
            var AllPolygonLayers = new List<List<Vector3>>[numberOfLayers];
            var AllPolygonLayersFace = new List<List<PolygonalFace>>[numberOfLayers];
            var maxRadius = MaxcylRadius(ts, cyl).Item1;
            var thickness = maxRadius / numberOfLayers;
            cyl.Radius = thickness;
            var offset_value = 0.0;

            (AllPolygonLayers[0], AllPolygonLayersFace[0]) = GetFirstLayer(ts, numberOfLayers, maxRadius,
                ref cyl, ref offset_value, ref thickness);
            if (epsilonOnOff)
            {
                var newpolygonList = new List<List<Vector3>>();
                for (var j = 0; j < AllPolygonLayers[0].Count; j++)
                {
                    newpolygonList.Add(GetEllipticalSegment(AllPolygonLayers[0][j], cyl, epsilon, AllPolygonLayersFace[0][j]));
                }
                AllPolygonLayers[0] = newpolygonList;
            }
#if PARALLEL
            Parallel.For(1, numberOfLayers, i =>
#else
            for (int i = 1; i < numberOfLayers; i++)
#endif
            {
                cyl.Radius = i >= numberOfLayers ?
                    maxRadius : i * thickness + offset_value;
                var edgeStatus = GetEdgeStatus(ts, cyl);
                var faceStatus = GetFaceStatus(ts, cyl, edgeStatus);
                (AllPolygonLayers[i], AllPolygonLayersFace[i]) = MakeLoops(edgeStatus, faceStatus);
                if (epsilonOnOff)
                {
                    var newpolygonList = new List<List<Vector3>>();
                    for (var j = 0; j < AllPolygonLayers[i].Count; j++)
                    {
                        newpolygonList.Add(GetEllipticalSegment(AllPolygonLayers[i][j], cyl, epsilon, AllPolygonLayersFace[i][j]));
                    }
                    AllPolygonLayers[i] = newpolygonList;
                }
            }
#if PARALLEL
            );
#endif

            return (AllPolygonLayers, AllPolygonLayersFace);
        }




        public static (List<List<Vector3>>, List<List<PolygonalFace>>) GetFirstLayer(TessellatedSolid ts, int NumberOfLayers,
            double maxRadius, ref Cylinder cyl, ref double offset_value, ref double thickness)
        {

            for (int i = 0; i < NumberOfLayers; i++)
            {
                cyl.Radius = cyl.Radius + thickness > maxRadius ?
                    maxRadius : i * thickness + offset_value;

                var edgeStatus = GetEdgeStatus(ts, cyl);

                // if the first cylinder does not intersect with any edge
                // assume the second cylinder as the first one & recalculate a new thickness 
                if (edgeStatus.Count <= 2 && i == 0)
                {
                    i -= 1;
                    offset_value += thickness;
                    thickness = (maxRadius - offset_value) / NumberOfLayers;
                }

                else
                {
                    var faceStatus = GetFaceStatus(ts, cyl, edgeStatus);
                    return MakeLoops(edgeStatus, faceStatus);
                }
            }
            return (null, null);
        }

        public static List<List<Vector3>> GetCylSectionsOnlyPoints(TessellatedSolid ts, Cylinder cyl, int NumberOfLayers)
        {
            cyl.Axis = cyl.Axis.Normalize();
            var AllPolygonLayers = new List<List<Vector3>>();
            var maxRadius = MaxcylRadius(ts, cyl);
            var thickness = (maxRadius.Item1 - maxRadius.Item2) / NumberOfLayers;
            cyl.Radius = thickness;
            var offseted = false;
            var offset_value = 0.0;
            for (int i = 0; i < NumberOfLayers; i++)
            {
                cyl.Radius = cyl.Radius + thickness > maxRadius.Item1 ?
                    maxRadius.Item1 : offseted ?
                    i * thickness + offset_value : i * thickness;

                var edgeStatus = GetEdgeStatus(ts, cyl);

                // if the first cylinder does not intersect with any edge
                // assume the second cylinder as the first one & recalculate a new thickness 
                if (edgeStatus.Count <= 2 && i == 0)
                {
                    i -= 1;
                    offseted = true;
                    offset_value += thickness;
                    thickness = (maxRadius.Item1 - thickness) / NumberOfLayers;
                }

                else
                {
                    var pts = new List<Vector3>(edgeStatus.Values.SelectMany(v => new[] { v.Item3, v.Item4 }
                    ).Where(p => !p.IsNull()));
                    foreach (var pt in pts)
                    {
                        var P = new List<Vector3> { pt };
                        AllPolygonLayers.AddRange(new[] { P });
                    }
                }
            }
            return AllPolygonLayers;
        }


        /// <summary>
        ///  This method returns the max cylidner radius.
        /// </summary>
        /// <param name="ts">The tessellated solid.</param>
        /// <param name="cyl">a cylinder.</param>
        /// <returns>Maximum Cylindrical Radius.</returns>
        internal static (double, double) MaxcylRadius(TessellatedSolid ts, Cylinder cyl)
        {
            var minRadius = double.PositiveInfinity;
            var maxRadius = 0.0;
            var minVertex = Vector3.Null;
            var maxVertex = Vector3.Null;

            foreach (var v in ts.Vertices)
            {
                var dist = (v.Coordinates - cyl.Anchor).Cross(cyl.Axis).LengthSquared();
                if (minRadius > dist)
                {
                    minRadius = dist;
                    minVertex = v.Coordinates;
                }
                if (maxRadius < dist)
                {
                    maxRadius = dist;
                    maxVertex = v.Coordinates;
                }
            }
            //var smallOffset = (maxVertex- minVertex).Normalize() * (minVertex.Distance(maxVertex) * 0.02);
            //var maxVertex_offseted = maxVertex + smallOffset;
            //var minVertex_offseted = minVertex - smallOffset;
            maxRadius = Math.Sqrt(maxRadius);
            minRadius = Math.Sqrt(minRadius);

            //ts.SolidColor = new Color(222, 255, 255, 255);
            //Presenter.ShowVertexPathsWithSolid(new[] { maxVertex_offseted, minVertex_offseted }, new[] { ts });
            //Presenter.ShowVertexPaths(new[] { minVertex_offseted, maxVertex_offseted });


            return (maxRadius, minRadius);
        }


        /// <summary>
        ///  This method returns all intersected edges with the number of intersection points and their positioins.
        /// </summary>
        /// <param name="ts">The tessellated solid.</param>
        /// <param name="cyl">a cylinder.</param>
        /// <returns>Dictionary of edge with number of intersection points and positions.</returns>
        ///
        public static Dictionary<Edge, (int, int, Vector3, Vector3)> GetEdgeStatus(TessellatedSolid ts, Cylinder cyl)
        {
            ///  This method does not use matrices to find the intersection (translation and rotation are not needed)
            ///  between an arbitrary line and arbitrary cylinder
            /// 
            ///  Loop over every edge in tessellatedsolid, look for the intersection points between line-cylinder
            ///  
            ///  Starting from the dot product of the intersected point:
            ///  ( (V-a)xd).((V-a)xd) = R^2
            ///  
            ///  V: intersected point (V = x0 + t*dL), along the current edge, 
            ///  a: Anchor point (cylinder), 
            ///  d: cylinder's direction (unit vector),
            ///  x0: The nearest point on the current edge to the cylinder's center line
            ///  dL: edge's direction (unit vector),
            ///  
            ///  Using the following two properties:
            ///  1. U.(VxW) = (UxV).(W)
            ///  2. Ux(VxW) = (U.W)*V - (U.V)*W
            ///  
            ///  Now, the dot product of the of two cross products can be represented as following:
            ///  ( [ (V-a).(V-a)*d ]-  [(V-a).(d)*(V-a)]).d = R^2
            ///  Now, find the value of t. Then, check if the new point is between the two ends of the current edge.
            ///  
            ///  End

            var Edges = ts.Edges;
            var R = cyl.Radius;

            var EdgeCylIntersect = new Dictionary<Edge, (int, int, Vector3, Vector3)>();
            foreach (var edge in Edges)
            {
                var V1 = Vector3.Null; var V2 = Vector3.Null;
                int NumberOfIntersection = 0; var d = cyl.Axis; var t1 = 0.0; var t2 = 0.0;
                var dL = edge.UnitVector;

                if (d.Cross(dL) == Vector3.Zero)
                {
                    if ((edge.From.Coordinates - cyl.Anchor).Cross(d).LengthSquared() == R * R)
                    {
                        //EdgeCylIntersect.Add(edge, (2, 2, edge.From.Coordinates, edge.To.Coordinates));
                    }
                }
                else
                {
                    // start the search for intersection poionts after checking
                    // the current edge is NOT parallel to the cylinder's direction 

                    var distance = SkewedLineIntersection(cyl.Anchor, d, edge.From.Coordinates,
                      dL, out _, out var interSect1, out var interSect2, out _, out var t);
                    var distance_From = (edge.From.Coordinates - cyl.Anchor).Cross(d).LengthSquared();
                    var distance_To = (edge.To.Coordinates - cyl.Anchor).Cross(d).LengthSquared();
                    if (distance < R)
                    {
                        var x0 = interSect2;
                        var a = cyl.Anchor;

                        var value_a = 1 - (dL.Dot(d) * dL).Dot(d);
                        var value_b = (2 * dL.Dot(x0) * d - 2 * dL.Dot(a) * d - (x0 - a).Dot(d) * dL
                            - dL.Dot(d) * (x0 - a)).Dot(d);
                        var value_c = -(R * R) + (x0.Dot(x0) * d).Dot(d) + (a.Dot(a) * d).Dot(d)
                            - (2 * (x0.Dot(a) * d)).Dot(d) - (x0 * (x0.Dot(d) - a.Dot(d))).Dot(d)
                            + (a * (x0.Dot(a) - a.Dot(d))).Dot(d);

                        t1 = (value_b + Math.Sqrt(value_b * value_b - 4 * value_a * value_c)) / (2 * value_a);
                        t2 = (value_b - Math.Sqrt(value_b * value_b - 4 * value_a * value_c)) / (2 * value_a);
                        V1 = x0 + t1 * dL;
                        V2 = x0 + t2 * dL;

                        t1 += t;
                        t2 += t;
                        if (t1 <= edge.Length && t1 >= 0)
                        {
                            // add the intersections only when it is between the two points edge.From & edge.To
                            NumberOfIntersection = 1;
                            if (t2 <= edge.Length && t2 >= 0)
                            {
                                NumberOfIntersection = 2;
                            }
                            else
                            {
                                // set the value of V2 to Null
                                V2 = Vector3.Null;
                            }
                        }
                        else
                        {
                            if (t2 <= edge.Length && t2 >= 0)
                            {
                                // if t2 is the only value between 0-1, set V1=V2 and V2=Null
                                V1 = V2;
                                V2 = Vector3.Null;
                                NumberOfIntersection = 1;
                            }
                            // if both t1&t2 below or above the limits 0-1, ignore the current edge 

                        }

                    }
                    // Identify tanget cases where the two ends of an edge are outside
                    if (NumberOfIntersection == 1 && distance_From > R * R && distance_To > R * R)
                    {
                        // ignore this type of intersection
                    }
                    else if (NumberOfIntersection > 0)
                    {
                        EdgeCylIntersect.Add(edge, (NumberOfIntersection, 2, V1, V2));
                    }

                }
            }
            return EdgeCylIntersect;
        }


        /// <summary>
        ///  This method returns all intersected faces with the start and end points and their edges.
        /// </summary>
        /// <param name="ts">The tessellated solid.</param>
        /// <param name="cyl">a cylinder.</param>
        /// <param name="EdgeStatus">EdgeStatus from GetEdgeStatus method.</param>
        /// <returns>Dictionary of intersection faces with a list of each line (start & end points with their edges) </returns>
        private static Dictionary<PolygonalFace, List<(Vector3, Edge)>> GetFaceStatus(TessellatedSolid ts, Cylinder cyl, Dictionary<Edge, (int, int, Vector3, Vector3)> EdgeStatus)
        {
            var SpecialCases = new List<(PolygonalFace, double)>();
            var res = new Dictionary<PolygonalFace, List<(Vector3, Edge)>>();
            var CylArea = Math.PI * cyl.Radius * cyl.Radius;
            foreach (var f in ts.Faces)
            {
                var IntersectedPoints = new List<Vector3>();
                var ListOfEdges = new List<Edge>();
                var NumberOfIntersection = 0;
                foreach (var e in f.Edges)
                {
                    if (EdgeStatus.ContainsKey(e))
                    {
                        NumberOfIntersection += EdgeStatus[e].Item2;
                        if (EdgeStatus[e].Item1 == 1)
                        {
                            IntersectedPoints.Add(EdgeStatus[e].Item3);
                            ListOfEdges.Add(e);
                        }
                        else if (EdgeStatus[e].Item1 == 2)
                        {
                            IntersectedPoints.Add(EdgeStatus[e].Item3);
                            IntersectedPoints.Add(EdgeStatus[e].Item4);
                            ListOfEdges.Add(e);
                            ListOfEdges.Add(e);
                        }
                    }
                }
                if (IntersectedPoints.Count == 0 && NumberOfIntersection == 0 && f.Area > CylArea)
                {
                    // check if a triangle encampasses the cylinder without edge intersection
                    var den = cyl.Axis.Dot(f.Normal);
                    if (den != 0)
                    {
                        var PtOnLine = MiscFunctions.PointOnTriangleFromRay(f, cyl.Anchor, cyl.Axis, out _, false);
                        if (PtOnLine != Vector3.Null)
                        {
                            var Test1 = MiscFunctions.IsVertexInsideTriangle(f.Vertices, PtOnLine, false);
                            var Test2 = IsVertexInsideTriangle(f, PtOnLine, false);
                            if (Test2 == true)
                            {
                                SpecialCases.Add((f, cyl.Radius));
                            }
                        }

                    }
                }

                else if (IntersectedPoints.Count >= 2)
                {
                    var r = GetOrder(IntersectedPoints, ListOfEdges, cyl, f);
                    res.Add(f, r);
                }
            }
            return res;
        }
        public static List<(Vector3, Edge)> GetOrder(List<Vector3> pts, List<Edge> edges, Cylinder cyl, PolygonalFace f)
        {
            List<(Vector3, Edge)> result = new List<(Vector3, Edge)>();
            if (pts.Count == 5)
            {
                var ListEdge = new Dictionary<Edge, int>();
                var hashsetEdges = edges.ToHashSet();
                foreach (var edge in hashsetEdges)
                    ListEdge.Add(edge, 0);
                for (int i = 0; i < edges.Count; i++)
                {
                    ListEdge[edges[i]] += 1;
                }
                int index = ListEdge.MinBy(x => x.Value).Value;
                pts.Insert(index, pts[index]);
                edges.Insert(index, edges[index]);
                bool sameEdge = false; Vertex v = f.A;
                var OrderedPoints = new List<(double, Vector3, Edge)>();
                double dist = 0.0;
                for (int i = 0; i < edges.Count; i++)
                {
                    if (i != 0)
                        if (edges[i] == edges[i - 1])
                            sameEdge = true;
                        else
                            sameEdge = false;
                    if (sameEdge)
                    {
                        dist += (v.Coordinates - pts[i]).LengthSquared();
                        OrderedPoints.Add((dist, pts[i], edges[i]));
                    }
                    else
                    {


                        if ((edges[i].From == f.A && edges[i].To == f.NextVertexCCW(f.A)) ||
                            (edges[i].To == f.A && edges[i].From == f.NextVertexCCW(f.A)))
                        {
                            v = f.A;
                            dist += (v.Coordinates - pts[i]).LengthSquared();
                            OrderedPoints.Add((dist, pts[i], edges[i]));
                        }
                        else if ((edges[i].From == f.NextVertexCCW(f.A)) && edges[i].To == f.NextVertexCCW(f.NextVertexCCW(f.A))
                            || (edges[i].To == f.NextVertexCCW(f.A)) && edges[i].From == f.NextVertexCCW(f.NextVertexCCW(f.A)))
                        {
                            v = f.NextVertexCCW(f.A);
                            dist += (v.Coordinates - pts[i]).LengthSquared();
                            OrderedPoints.Add((dist, pts[i], edges[i]));
                        }
                        else
                        {
                            v = f.NextVertexCCW(f.NextVertexCCW(f.A));
                            dist += (v.Coordinates - pts[i]).LengthSquared();
                            OrderedPoints.Add((dist, pts[i], edges[i]));
                        }

                    }
                }
                OrderedPoints = OrderedPoints.OrderBy(x => x.Item1).ToList();
                for (int i = 0; i < OrderedPoints.Count; i++)
                    result.Add((OrderedPoints[i].Item2, OrderedPoints[i].Item3));
                return result;
            }
            else if (pts.Count == 3)
            {
                if (edges.ToHashSet().Count == 2)
                {
                    if (edges[0] == edges[1])
                    {
                        pts.Add(pts[2]); pts.Add(pts[0]); pts.RemoveAt(0);
                        edges.Add(edges[2]); edges.Add(edges[0]); edges.RemoveAt(0);

                    }
                    else if (edges[0] == edges[2])
                    {
                        pts.Insert(1, pts[1]);
                        edges.Insert(1, edges[1]);
                    }
                    else if (edges[1] == edges[2])
                    {
                        pts.Insert(0, pts[2]); pts.Insert(1, pts[1]); pts.RemoveAt(pts.Count - 1);
                        edges.Insert(0, edges[2]); edges.Insert(1, edges[1]); edges.RemoveAt(edges.Count - 1);
                    }
                    for (int i = 0; i < pts.Count; i++)
                        result.Add((pts[i], edges[i]));
                    return result;
                }
                else
                {
                    var distance01 = SkewedLineIntersection(cyl.Anchor, cyl.Axis, pts[0],
                      pts[0] - pts[1], out _, out var interSect1, out var interSect2, out _, out _);

                    var distance02 = SkewedLineIntersection(cyl.Anchor, cyl.Axis, pts[0],
                      pts[0] - pts[2], out _, out var interSect3, out var interSect4, out _, out _);
                    var distance12 = SkewedLineIntersection(cyl.Anchor, cyl.Axis, pts[0],
                      pts[1] - pts[2], out _, out var interSect5, out var interSect6, out _, out _);
                    if (distance01 < distance02 && distance01 < distance12)
                    {
                        pts = new List<Vector3> { pts[0], pts[2], pts[2], pts[1] };
                        edges = new List<Edge> { edges[0], edges[2], edges[2], edges[1] };
                        for (int i = 0; i < pts.Count; i++)
                            result.Add((pts[i], edges[i]));
                        return result;
                    }
                    else if (distance02 < distance12 && distance02 < distance01)
                    {
                        pts = new List<Vector3> { pts[2], pts[1], pts[1], pts[0] };
                        edges = new List<Edge> { edges[2], edges[1], edges[1], edges[0] };
                        for (int i = 0; i < pts.Count; i++)
                            result.Add((pts[i], edges[i]));
                        return result;
                    }
                    else
                    {
                        pts = new List<Vector3> { pts[1], pts[0], pts[0], pts[2] };
                        edges = new List<Edge> { edges[1], edges[0], edges[0], edges[2] };
                        for (int i = 0; i < pts.Count; i++)
                            result.Add((pts[i], edges[i]));
                        return result;
                    }
                }
            }
            var n = pts.Count;
            if (n == 2)
                return new List<(Vector3, Edge)> { (pts[0], edges[0]), (pts[1], edges[1]) };
            else if (n == 4)
            {
                if (edges.Count - edges.ToHashSet().Count == 2)
                {
                    if (edges[0] == edges[1])
                    {
                        pts.Add(pts[0]); pts.RemoveAt(0);
                        edges.Add(edges[0]); edges.RemoveAt(0);
                    }
                    else if (edges[2] == edges[3])
                    {
                        pts.Insert(0, pts[3]); pts.RemoveAt(pts.Count - 1);
                        edges.Insert(0, edges[3]); edges.RemoveAt(pts.Count - 1);
                    }
                    SkewedLineIntersection(pts[0], pts[1] - pts[0], pts[2], pts[3] - pts[2], out _, out _, out _, out double t1, out _);
                    if (t1 >= 0 && t1 <= 1)
                    {
                        pts.Insert(1, pts[2]); pts.RemoveAt(3);
                        edges.Insert(1, edges[2]); edges.RemoveAt(3);
                    }
                }
                else
                {
                    if (edges[0] == edges[1])
                    {
                        pts.Add(pts[0]); pts.RemoveAt(0);
                        edges.Add(edges[0]); edges.RemoveAt(0);
                    }
                    else if (edges[2] == edges[3])
                    {
                        pts.Insert(0, pts[3]); pts.RemoveAt(pts.Count - 1);
                        edges.Insert(0, edges[3]); edges.RemoveAt(pts.Count - 1);
                    }
                    SkewedLineIntersection(pts[0], pts[1] - pts[0], pts[2], pts[3] - pts[2], out _, out _, out _, out double t1, out _);
                    if (t1 >= 0 && t1 <= 1)
                    {
                        pts.Insert(1, pts[2]); pts.RemoveAt(3);
                        edges.Insert(1, edges[2]); edges.RemoveAt(3);
                    }
                }

            }
            else if (n == 6)
            {
                var OrderedList = new List<(double, Vector3, Edge)>();
                var OrderedList2 = new List<(Vector3, Edge)>();

                if (edges[0] == edges[1])
                {
                    Vector3 VertexA = Vector3.Null; ;

                    if (edges[1].From.Coordinates == edges[2].From.Coordinates ||
                        edges[1].To.Coordinates == edges[2].From.Coordinates)
                    {
                        VertexA = edges[2].From.Coordinates;
                    }
                    else
                    {
                        VertexA = edges[2].To.Coordinates;
                    }
                    var pt = Vector3.Zero;
                    if ((pts[1] - VertexA).LengthSquared() > (pts[0] - VertexA).LengthSquared())
                    {
                        pt = pts[1];
                        pts[1] = pts[0];
                        pts[0] = pt;
                    }
                    OrderedList.Add((1, pts[0], edges[0]));
                    OrderedList.Add((6, pts[1], edges[1]));

                    if (edges[2] == edges[3])
                    {
                        if (edges[3].From.Coordinates == edges[4].From.Coordinates ||
                            edges[3].To.Coordinates == edges[4].From.Coordinates)
                        {
                            VertexA = edges[4].From.Coordinates;
                        }
                        else
                        {
                            VertexA = edges[4].To.Coordinates;
                        }
                        if ((pts[3] - VertexA).LengthSquared() > (pts[2] - VertexA).LengthSquared())
                        {
                            pt = pts[2];
                            pts[2] = pts[3];
                            pts[3] = pt;
                        }
                        OrderedList.Add((2, pts[2], edges[2]));
                        OrderedList.Add((3, pts[3], edges[3]));
                        if (edges[4] == edges[5])
                        {
                            if ((pts[4] - VertexA).LengthSquared() > (pts[5] - VertexA).LengthSquared())
                            {
                                pt = pts[4];
                                pts[4] = pts[5];
                                pts[5] = pt;
                            }
                            OrderedList.Add((4, pts[4], edges[4]));
                            OrderedList.Add((5, pts[5], edges[5]));
                            OrderedList.Sort();
                            foreach (var l in OrderedList)
                            {
                                OrderedList2.Add((l.Item2, l.Item3));
                            }

                            var t = SkewedLineIntersectionOnlyT1(OrderedList2[0].Item1, OrderedList2[1].Item1 - OrderedList2[0].Item1,
                                OrderedList2[2].Item1, OrderedList2[3].Item1 - OrderedList2[2].Item1);
                            if (t >= 0 && t <= 1)
                            {
                                var r = OrderedList2[1];
                                OrderedList2[1] = OrderedList2[2];
                                OrderedList2[2] = r;
                            }
                            t = SkewedLineIntersectionOnlyT1(OrderedList2[0].Item1, OrderedList2[5].Item1 - OrderedList2[0].Item1,
                                OrderedList2[4].Item1, OrderedList2[1].Item1 - OrderedList2[4].Item1);
                            if (t >= 0 && t <= 1)
                            {
                                var r = OrderedList2[4];
                                OrderedList2[4] = OrderedList2[5];
                                OrderedList2[5] = r;
                            }
                            return OrderedList2;
                        }
                    }
                }
            }

            else
                throw new Exception("A tangent point was not resolved");
            for (int i = 0; i < pts.Count; i++)
                result.Add((pts[i], edges[i]));
            return result;
        }

        private static (List<List<Vector3>>, List<List<PolygonalFace>>) MakeLoops(Dictionary<Edge, (int, int, Vector3, Vector3)> edgesStatus,
            Dictionary<PolygonalFace, List<(Vector3, Edge)>> faceStatus)
        {
            var loopFaces = new List<List<PolygonalFace>>();
            var loops = new List<List<Vector3>>();
            var edgeList = edgesStatus.Keys.ToList();
            var unvisitedPointHashSet = new HashSet<Vector3>(edgesStatus.Values.SelectMany(v => new[] { v.Item3, v.Item4 })
                .Where(p => !p.IsNull())); //this makes a queue of all the points in edgesStatuses

            while (edgeList.Any())
            {
                var thisEdgeStatus = edgesStatus[edgeList[0]];
                var start = thisEdgeStatus.Item3;
                if (start.IsNull() || !unvisitedPointHashSet.Contains(start))
                    start = thisEdgeStatus.Item4;
                if (start.IsNull() || !unvisitedPointHashSet.Contains(start))
                {
                    edgeList.RemoveAt(0);
                    continue;
                }
                var loop = new List<Vector3>();
                loop.Add(start);
                unvisitedPointHashSet.Remove(start);
                //Edge CurrentEdge = GetCurrentEdge(edgesStatus, start);
                var CurrentEdge = edgeList[0];
                PolygonalFace CurrentFace = null;
                var FaceList = new List<PolygonalFace>();

                foreach (var f in faceStatus)
                {
                    foreach (var line in f.Value)
                    {
                        if (line.Item1 == start && CurrentEdge == line.Item2)
                        {
                            CurrentFace = f.Key;
                            break;
                        }
                    }
                }
                FaceList.Add(CurrentFace);
                int i = 0;

                while (loop[0] != start || i == 0)
                {
                    i += 1;
                    start = GetNextPoint(start, faceStatus, loop, ref CurrentEdge, ref CurrentFace);
                    loop.Add(start);
                    FaceList.Add(CurrentFace);
                    unvisitedPointHashSet.Remove(start);

                }

                if (loop.Count > 3) // To ignore tangent points
                {
                    loop.RemoveAt(0);
                    FaceList.RemoveAt(0);
                    loops.Add(loop);
                    loopFaces.Add(FaceList);
                }
            }
            var Removeidex = new HashSet<int>();

            for (var i = 0; i < loops.Count; i++)
            {
                for (var j = 0; j < loops.Count; j++)
                {
                    if (i != j)
                    {
                        var a = loops[i].ToHashSet(); var b = loops[j].ToHashSet();
                        if (b.IsSubsetOf(a))
                        {
                            Removeidex.Add(j);
                        }
                    }
                }
            }
            var Removeidex2 = Removeidex.ToList(); Removeidex2.Sort(); Removeidex2.Reverse();
            foreach (var i in Removeidex2)
            {
                loops.RemoveAt(i);
                loopFaces.RemoveAt(i);
            }

            for (int i = 0; i < loops.Count; i++)
            {
                loops[i].Reverse();
                loopFaces[i].Reverse();
                loops[i].Add(loops[i][0]);
                loopFaces[i].Add(loopFaces[i][0]);
            }
            return (loops, loopFaces);
        }

        #endregion

        #region Spherical Slicing 
        /// <summary>
        ///  This method slices a tessellated solid using spherical surface.
        /// </summary>
        /// <param name="ts">The tessellated solid.</param>
        /// <param name="cyl">A sphere.</param>
        /// <param name="thickness">Layer's thickness.</param>
        /// <returns>List of polygons and faces of the intersection between tessellated solid and sphere.</returns>
        public static (List<List<Vector3>>[], List<List<PolygonalFace>>[]) GetSphericalSections(TessellatedSolid ts, Sphere sph, int NumberOfLayers)
        {
            var AllPolygonLayers = new List<List<Vector3>>[NumberOfLayers];
            var AllPolygonLayersFace = new List<List<PolygonalFace>>[NumberOfLayers];
            var (minRadius, maxRadius) = GetsphRadius(ts, sph);
            var thickness = (maxRadius - minRadius) / NumberOfLayers;
            sph.Radius = minRadius;
            var offset_value = 0.0;
            var t = thickness;
            (AllPolygonLayers[0], AllPolygonLayersFace[0]) = GetFirstLayer(ts, NumberOfLayers, maxRadius, ref sph, ref offset_value, ref t);

#if PARALLEL
            Parallel.For(1, NumberOfLayers, i =>
#else
            for (int i = 1; i < NumberOfLayers; i++)
#endif

            {
                if (i + 1 != NumberOfLayers)
                {
                    sph.Radius = minRadius + i * thickness;
                }
                else
                {
                    sph.Radius = maxRadius;
                }

                var edgeStatus = GetEdgeStatus(ts, sph);
                //var matrix = GetSphereMatrix(ts, sph);
                //var edgeStatus = GetSlicesWMatrix(ts, matrix);

                var faceStatus = GetFaceStatus(ts, sph, edgeStatus);
                (AllPolygonLayers[i], AllPolygonLayersFace[i]) = MakeLoops(edgeStatus, faceStatus);

            }

#if PARALLEL
            );
#endif
            return (AllPolygonLayers, AllPolygonLayersFace);
        }

        public static (List<List<Vector3>>[], List<List<PolygonalFace>>[], Sphere[]) GetSphericalSections(TessellatedSolid ts, Sphere sph, double thickness)
        {
            var (minRadius, maxRadius) = GetsphRadius(ts, sph);
            int NumberOfLayers = (int)Math.Ceiling(maxRadius / thickness);
            var AllPolygonLayers = new List<List<Vector3>>[NumberOfLayers];
            var AllPolygonLayersFace = new List<List<PolygonalFace>>[NumberOfLayers];
            var Spheres = new Sphere[NumberOfLayers];
            sph.Radius = minRadius;
            var offset_value = 0.0;
            var t = thickness;
            (AllPolygonLayers[0], AllPolygonLayersFace[0]) = GetFirstLayer(ts, NumberOfLayers, maxRadius, ref sph, ref offset_value, ref t);
            Spheres[0] = sph;
#if PARALLEL
            Parallel.For(0, NumberOfLayers, i =>
#else
            for (int i = 0; i < NumberOfLayers; i++)
#endif

            {
                if (i + 1 != NumberOfLayers)
                {
                    sph.Radius = minRadius + i * thickness;
                }
                else
                {
                    sph.Radius = maxRadius;
                }

                var edgeStatus = GetEdgeStatus(ts, sph);


                var faceStatus = GetFaceStatus(ts, sph, edgeStatus);
                (AllPolygonLayers[i], AllPolygonLayersFace[i]) = MakeLoops(edgeStatus, faceStatus);
                Spheres[i] = sph;

            }

#if PARALLEL
            );
#endif
            return (AllPolygonLayers, AllPolygonLayersFace, Spheres);
        }

        public static (List<List<Vector3>>, List<List<PolygonalFace>>) GetFirstLayer(TessellatedSolid ts, int NumberOfLayers,
            double maxRadius, ref Sphere Sph, ref double offset_value, ref double thickness)
        {

            for (int i = 0; i < NumberOfLayers; i++)
            {
                Sph.Radius = Sph.Radius + thickness > maxRadius ?
                    maxRadius : i * thickness + offset_value;

                //var edgeStatus = GetEdgeStatus(ts, Sph);
                //var matrix = GetSphereMatrix(ts, Sph);
                //var edgeStatus = GetSlicesWMatrix(ts, matrix);
                // if the first cylinder does not intersect with any edge
                // assume the second cylinder as the first one & recalculate a new thickness 
                //if (edgeStatus.Count <= 2 && i == 0)
                //{
                //    i -= 1;
                //    offset_value += thickness;
                //    thickness = (maxRadius - offset_value) / NumberOfLayers;
                // }

                // else
                // {
                //     var faceStatus = GetFaceStatus(ts, Sph, edgeStatus);
                //     return MakeLoops(edgeStatus, faceStatus);
                // }
            }
            return (null, null);
        }



        private static (List<List<Vector3>>, List<List<PolygonalFace>>) MakeLoops(Dictionary<Edge, (int, Vector3, Vector3)> edgesStatus,
            Dictionary<PolygonalFace, List<(Vector3, Edge)>> faceStatus)
        {
            var loopFaces = new List<List<PolygonalFace>>();
            var loops = new List<List<Vector3>>();
            var edgeList = edgesStatus.Keys.ToList();
            var unvisitedPointHashSet = new HashSet<Vector3>(edgesStatus.Values.SelectMany(v => new[] { v.Item2, v.Item3 })
                .Where(p => !p.IsNull())); //this makes a queue of all the points in edgesStatuses

            while (edgeList.Any())
            {
                var thisEdgeStatus = edgesStatus[edgeList[0]];
                var start = thisEdgeStatus.Item2;
                if (start.IsNull() || !unvisitedPointHashSet.Contains(start))
                    start = thisEdgeStatus.Item3;
                if (start.IsNull() || !unvisitedPointHashSet.Contains(start))
                {
                    edgeList.RemoveAt(0);
                    continue;
                }
                var loop = new List<Vector3>();
                loop.Add(start);
                unvisitedPointHashSet.Remove(start);
                //Edge CurrentEdge = GetCurrentEdge(edgesStatus, start);
                var CurrentEdge = edgeList[0];
                PolygonalFace CurrentFace = null;
                var FaceList = new List<PolygonalFace>();

                foreach (var f in faceStatus)
                {
                    foreach (var line in f.Value)
                    {
                        if (line.Item1 == start && CurrentEdge == line.Item2)
                        {
                            CurrentFace = f.Key;
                            break;
                        }
                    }
                }
                FaceList.Add(CurrentFace);
                int i = 0;

                while (loop[0] != start || i == 0)
                {
                    i += 1;
                    start = GetNextPoint(start, faceStatus, loop, ref CurrentEdge, ref CurrentFace);
                    loop.Add(start);
                    FaceList.Add(CurrentFace);
                    unvisitedPointHashSet.Remove(start);

                }

                if (loop.Count > 3) // To ignore tangent points
                {
                    loop.RemoveAt(0);
                    FaceList.RemoveAt(0);
                    loops.Add(loop);
                    loopFaces.Add(FaceList);
                }
            }
            var Removeidex = new HashSet<int>();

            for (var i = 0; i < loops.Count; i++)
            {
                for (var j = 0; j < loops.Count; j++)
                {
                    if (i != j)
                    {
                        var a = loops[i].ToHashSet(); var b = loops[j].ToHashSet();
                        if (b.IsSubsetOf(a))
                        {
                            Removeidex.Add(j);
                        }
                    }
                }
            }
            var Removeidex2 = Removeidex.ToList(); Removeidex2.Sort(); Removeidex2.Reverse();
            foreach (var i in Removeidex2)
            {
                loops.RemoveAt(i);
                loopFaces.RemoveAt(i);
            }

            for (int i = 0; i < loops.Count; i++)
            {
                loops[i].Reverse();
                loopFaces[i].Reverse();
                loops[i].Add(loops[i][0]);
                loopFaces[i].Add(loopFaces[i][0]);
            }

            return (loops, loopFaces);
        }

        private static Dictionary<PolygonalFace, List<(Vector3, Edge)>> GetFaceStatus(TessellatedSolid ts, Sphere sph,
            Dictionary<Edge, (int, Vector3, Vector3)> edgeStatus)
        {
            var SpecialCases = new List<(PolygonalFace, double)>();
            var res = new Dictionary<PolygonalFace, List<(Vector3, Edge)>>();
            foreach (var f in ts.Faces)
            {
                var IntersectedPoints = new List<Vector3>();
                var ListOfEdges = new List<Edge>();
                foreach (var e in f.Edges)
                {
                    if (edgeStatus.ContainsKey(e))
                    {
                        if (edgeStatus[e].Item1 == 1)
                        {
                            IntersectedPoints.Add(edgeStatus[e].Item2);
                            ListOfEdges.Add(e);
                        }
                        else if (edgeStatus[e].Item1 == 2)
                        {
                            IntersectedPoints.Add(edgeStatus[e].Item2);
                            IntersectedPoints.Add(edgeStatus[e].Item3);
                            ListOfEdges.Add(e);
                            ListOfEdges.Add(e);
                        }
                    }
                }
                if (IntersectedPoints.Count == 0)
                {
                    // Speical Case
                    var dist = MiscFunctions.DistancePointToPlane(sph.Center, f.Normal, f.Vertices[0].Coordinates);

                }
                if (IntersectedPoints.Count >= 2 && IntersectedPoints.ToHashSet().Count() > 1)
                {
                    var r = GetOrderCCW(IntersectedPoints, ListOfEdges);
                    res.Add(f, r);
                }
            }

            return res;

        }

        private static Dictionary<Edge, (int, Vector3, Vector3)> GetEdgeStatus(TessellatedSolid ts, Sphere sph)
        {
            var Edges = ts.Edges;
            var R = sph.Radius;
            var EdgeSphIntersect = new Dictionary<Edge, (int, Vector3, Vector3)>();
            foreach (var edge in Edges)
            {
                var d = edge.Vector;
                var a = d.Dot(d);
                var b = -2 * (sph.Center - edge.From.Coordinates).Dot(d);
                var c = (sph.Center - edge.From.Coordinates).Dot(sph.Center - edge.From.Coordinates) - R * R;
                var V1 = Vector3.Null; var V2 = Vector3.Null;
                var SQRT = b * b - 4 * a * c;
                var NumberOfIntersection = 0;
                if (SQRT >= 0)
                {
                    var t1 = (-b + Math.Sqrt(SQRT)) / (2 * a);
                    var t2 = (-b - Math.Sqrt(SQRT)) / (2 * a);
                    if (t1 >= 0 && t1 <= 1)
                    {
                        V1 = edge.From.Coordinates + t1 * d;
                        NumberOfIntersection = 1;
                        if (t2 >= 0 && t2 <= 1)
                        {
                            NumberOfIntersection = 2;
                            V2 = edge.From.Coordinates + t2 * d;
                        }
                    }
                    else if (t2 >= 0 && t2 <= 1)
                    {
                        NumberOfIntersection = 1;
                        V1 = edge.From.Coordinates + t2 * d;
                    }
                    if (NumberOfIntersection > 0)
                    {
                        EdgeSphIntersect.Add(edge, (NumberOfIntersection, V1, V2));
                    }
                }

            }
            return EdgeSphIntersect;
        }

        private static (double, double) GetsphRadius(TessellatedSolid ts, Sphere sph)
        {
            var minRadius = double.PositiveInfinity;
            var maxRadius = 0.0;
            var minVertex = Vector3.Null;
            var maxVertex = Vector3.Null;
            foreach (var v in ts.Vertices)
            {
                var Vect = v.Coordinates - sph.Center;
                var dist = Vect.Dot(Vect);
                if (minRadius > dist)
                {
                    minRadius = dist;
                    minVertex = v.Coordinates;
                }
                if (maxRadius < dist)
                {
                    maxRadius = dist;
                    maxVertex = v.Coordinates;
                }
            }
            minRadius = Math.Sqrt(minRadius);
            maxRadius = Math.Sqrt(maxRadius);
            return (minRadius, maxRadius);

        }


        public static List<List<(List<Vector3>, List<List<Vector3>>)>> ConverTo3DPolygonWithInnerPolygon(List<List<List<Vector3>>> sphSlices, Plane plane)
        {
            var PtTo2DPolygon = new List<List<(Polygon, int)>>();
            var ConvertedPolygons = new List<List<(List<Vector3>, List<List<Vector3>>)>>();
            sphSlices = sphSlices.Where(x => x != null && x.Count() > 0).ToList();
            // Sort by area first
            // test inside polyogn condition  
            for (int i = 0; i < sphSlices.Count; i++)
            {
                var TempIndex = new List<(int, List<int>)>();
                var CurrentPoly = new List<(Polygon, int)>();
                if (sphSlices[i].Count > 1)
                {
                    for (int j = 0; j < sphSlices[i].Count; j++)
                        CurrentPoly.Add((new Polygon(MiscFunctions.ProjectTo2DCoordinates(sphSlices[i][j], plane.Normal, out _)), j));
                    CurrentPoly = CurrentPoly.OrderByDescending(x => x.Item1.Area).ToList();

                    var id = new List<int>();
                    while (CurrentPoly.Count > 0)
                    {
                        var TempPoly = CurrentPoly[0];
                        CurrentPoly.RemoveAt(0);


                        for (int j = 0; j < CurrentPoly.Count; j++)
                        {
                            if (!id.Contains(CurrentPoly[j].Item2) &&
                                TempPoly.Item1.GetPolygonInteraction(CurrentPoly[j].Item1).Relationship
                                == PolygonRelationship.BInsideA)
                            {
                                id.Add(CurrentPoly[j].Item2);
                            }

                        }
                        if (!id.Contains(TempPoly.Item2))
                            TempIndex.Add((TempPoly.Item2, id));
                    }
                    var PolygonwithHoles = new List<(List<Vector3>, List<List<Vector3>>)>();
                    foreach (var index in TempIndex)
                    {
                        var innerPoly = new List<List<Vector3>>();
                        foreach (var ids in index.Item2)
                            innerPoly.Add(sphSlices[i][ids]);
                        PolygonwithHoles.Add((sphSlices[i][index.Item1], innerPoly));
                    }
                    if (PolygonwithHoles.Count > 0)
                        ConvertedPolygons.Add(PolygonwithHoles);
                }

                else if (sphSlices.Count == 1)
                {
                    var innerPoly = new List<List<Vector3>>();

                    ConvertedPolygons.Add(new List<(List<Vector3>, List<List<Vector3>>)> { (sphSlices[i][0], innerPoly) });
                }
            }
            return ConvertedPolygons;
        }


        #endregion

        #region MiscFunctions

        public static List<Polygon>[] GetUniformlySpacedCrossSections(this TessellatedSolid ts, Vector3 direction, double startDistanceAlongDirection = double.NaN,
                int numSlices = -1, double stepSize = double.NaN)
        {
            if (double.IsNaN(stepSize) && numSlices < 1) throw new ArgumentException("Either a valid stepSize or a number of slices greater than zero must be specified.");
            direction = direction.Normalize();
            var transform = direction.TransformToXYPlane(out _);
            var plane = new Plane(0.0, direction);
            //First, sort the vertices along the given axis. Duplicate distances are not important.
            var sortedVertices = ts.Vertices.OrderBy(v => v.Dot(direction)).ToArray();
            var firstDistance = sortedVertices[0].Dot(direction);
            var lastDistance = sortedVertices[^1].Dot(direction);
            var lengthAlongDir = lastDistance - firstDistance;
            stepSize = Math.Abs(stepSize);
            if (double.IsNaN(stepSize)) stepSize = lengthAlongDir / numSlices;
            if (numSlices < 1) numSlices = (int)(lengthAlongDir / stepSize);
            if (double.IsNaN(startDistanceAlongDirection))
                startDistanceAlongDirection = firstDistance ;

            var result = new List<Polygon>[numSlices];
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
                    result[step] = GetLoops(currentEdges.ToDictionary(ce => ce, ce =>
                       MiscFunctions.PointOnPlaneFromIntersectingLine(plane, ce.From.Coordinates, ce.To.Coordinates, out _)
                           .ConvertTo2DCoordinates(transform)), plane.Normal, plane.DistanceToOrigin, out _);
                else result[step] = new List<Polygon>();
            }
            return result;
        }
        static List<Polygon> GetLoops(Dictionary<Edge, Vector2> edgeDictionary, Vector3 normal, double distanceToOrigin,
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
                        Debug.WriteLine("Incomplete loop.");
                        if (path.Count > 2)
                            AddToPolygons(path, edgesInLoop, polygons, e2VDictionary);
                        finishedLoop = true;
                    }
                    else currentEdge = nextEdge;
                } while (!finishedLoop);
            }
            return polygons.CreateShallowPolygonTrees(false);
        }
        private static void AddToPolygons(List<Vector2> path, List<Edge> edgesInLoop, List<Polygon> polygons, Dictionary<Vertex2D, Edge> e2VDictionary)
        {
            var polygon = new Polygon(path);
            polygons.Add(polygon);
            for (int i = 0; i < polygon.Vertices.Count; i++)
                e2VDictionary.Add(polygon.Vertices[i], edgesInLoop[i]);
        }
        public static double V4Dot(Vector4 v1, Vector4 v2)
        {
            return v1.Dot(v2);
        }
        public static Vector4 M_mult_V(Matrix4x4 m, Vector4 v)
        {
            return m * v;
            var L1 = new Vector4(m.M11, m.M12, m.M13, m.M14);
            var L2 = new Vector4(m.M21, m.M22, m.M23, m.M24);
            var L3 = new Vector4(m.M31, m.M32, m.M33, m.M34);
            var L4 = new Vector4(m.M41, m.M42, m.M43, m.M44);

            return new Vector4(V4Dot(v, L1), V4Dot(v, L2), V4Dot(v, L3), V4Dot(v, L4));
        }
        public static List<(Vector3, Edge)> GetOrderCCW(List<Vector3> IntersectedPoints, List<Edge> ListOfEdge)
        {
            var PointHashSet = new List<Vector3>();
            var PointEdgeHashSet = new List<(Vector3, Edge)>();
            for (int i = 0; i < IntersectedPoints.Count(); i++)
            {
                if (!PointHashSet.Contains(IntersectedPoints[i]))
                {
                    PointHashSet.Add(IntersectedPoints[i]);
                    PointEdgeHashSet.Add((IntersectedPoints[i], ListOfEdge[i]));
                }
            }
            var c = PointHashSet.Count();
            var OrderedList = new List<(double, Vector3, Edge)>();
            var OrderedList2 = new List<(Vector3, Edge)>();
            if (c == 2)
                return PointEdgeHashSet;


            if (c == 3 || c == 5)
            {
                //Tanget cases was not resolved 
            }
            if (ListOfEdge.ToHashSet().Count > 2)
            {
                if (c == 4)
                {
                    if (ListOfEdge[0] != ListOfEdge[1])
                    {
                        if (ListOfEdge[1] == ListOfEdge[2])
                        {
                            ListOfEdge.Add(ListOfEdge[0]);
                            ListOfEdge.RemoveAt(0);
                            IntersectedPoints.Add(IntersectedPoints[0]);
                            IntersectedPoints.RemoveAt(0);
                        }
                        else
                        {
                            ListOfEdge.Reverse();
                            IntersectedPoints.Reverse();
                        }
                    }
                }
            }

            if (c == 4)
            {
                if (ListOfEdge[0] == ListOfEdge[1])
                {
                    Vector3 VertexA = Vector3.Null;

                    if (ListOfEdge[1].From.Coordinates == ListOfEdge[2].From.Coordinates ||
                        ListOfEdge[1].To.Coordinates == ListOfEdge[2].From.Coordinates)
                    {
                        VertexA = ListOfEdge[2].From.Coordinates;
                    }
                    else
                    {
                        VertexA = ListOfEdge[2].To.Coordinates;
                    }
                    var pt = Vector3.Zero;
                    if ((IntersectedPoints[1] - VertexA).LengthSquared() > (IntersectedPoints[0] - VertexA).LengthSquared())
                    {
                        pt = IntersectedPoints[1];
                        IntersectedPoints[1] = IntersectedPoints[0];
                        IntersectedPoints[0] = pt;
                    }
                    OrderedList.Add((1, IntersectedPoints[0], ListOfEdge[0]));
                    OrderedList.Add((4, IntersectedPoints[1], ListOfEdge[1]));

                    if (ListOfEdge.ToHashSet().Count == 2)
                    {
                        if ((IntersectedPoints[2] - VertexA).LengthSquared() > (IntersectedPoints[3] - VertexA).LengthSquared())
                        {
                            pt = IntersectedPoints[2];
                            IntersectedPoints[2] = IntersectedPoints[3];
                            IntersectedPoints[3] = pt;
                        }
                    }

                    OrderedList.Add((2, IntersectedPoints[2], ListOfEdge[2]));
                    OrderedList.Add((3, IntersectedPoints[3], ListOfEdge[3]));
                    OrderedList.Sort();
                    foreach (var l in OrderedList)
                    {
                        OrderedList2.Add((l.Item2, l.Item3));
                    }
                    var t = SkewedLineIntersectionOnlyT1(OrderedList2[0].Item1, OrderedList2[1].Item1 - OrderedList2[0].Item1,
                                OrderedList2[2].Item1, OrderedList2[3].Item1 - OrderedList2[2].Item1);
                    if (t >= 0 && t <= 1)
                    {
                        var r = OrderedList2[1];
                        OrderedList2[1] = OrderedList2[2];
                        OrderedList2[2] = r;
                    }

                    return OrderedList2;
                }



            }

            else if (c == 6)
            {
                if (ListOfEdge[0] == ListOfEdge[1])
                {
                    Vector3 VertexA = Vector3.Null; ;

                    if (ListOfEdge[1].From.Coordinates == ListOfEdge[2].From.Coordinates ||
                        ListOfEdge[1].To.Coordinates == ListOfEdge[2].From.Coordinates)
                    {
                        VertexA = ListOfEdge[2].From.Coordinates;
                    }
                    else
                    {
                        VertexA = ListOfEdge[2].To.Coordinates;
                    }
                    var pt = Vector3.Zero;
                    if ((IntersectedPoints[1] - VertexA).LengthSquared() > (IntersectedPoints[0] - VertexA).LengthSquared())
                    {
                        pt = IntersectedPoints[1];
                        IntersectedPoints[1] = IntersectedPoints[0];
                        IntersectedPoints[0] = pt;
                    }
                    OrderedList.Add((1, IntersectedPoints[0], ListOfEdge[0]));
                    OrderedList.Add((6, IntersectedPoints[1], ListOfEdge[1]));

                    if (ListOfEdge[2] == ListOfEdge[3])
                    {
                        if (ListOfEdge[3].From.Coordinates == ListOfEdge[4].From.Coordinates ||
                            ListOfEdge[3].To.Coordinates == ListOfEdge[4].From.Coordinates)
                        {
                            VertexA = ListOfEdge[4].From.Coordinates;
                        }
                        else
                        {
                            VertexA = ListOfEdge[4].To.Coordinates;
                        }
                        if ((IntersectedPoints[3] - VertexA).LengthSquared() > (IntersectedPoints[2] - VertexA).LengthSquared())
                        {
                            pt = IntersectedPoints[2];
                            IntersectedPoints[2] = IntersectedPoints[3];
                            IntersectedPoints[3] = pt;
                        }
                        OrderedList.Add((2, IntersectedPoints[2], ListOfEdge[2]));
                        OrderedList.Add((3, IntersectedPoints[3], ListOfEdge[3]));
                        if (ListOfEdge[4] == ListOfEdge[5])
                        {
                            if ((IntersectedPoints[4] - VertexA).LengthSquared() > (IntersectedPoints[5] - VertexA).LengthSquared())
                            {
                                pt = IntersectedPoints[4];
                                IntersectedPoints[4] = IntersectedPoints[5];
                                IntersectedPoints[5] = pt;
                            }
                            OrderedList.Add((4, IntersectedPoints[4], ListOfEdge[4]));
                            OrderedList.Add((5, IntersectedPoints[5], ListOfEdge[5]));
                            OrderedList.Sort();
                            foreach (var l in OrderedList)
                            {
                                OrderedList2.Add((l.Item2, l.Item3));
                            }

                            var t = SkewedLineIntersectionOnlyT1(OrderedList2[0].Item1, OrderedList2[1].Item1 - OrderedList2[0].Item1,
                                OrderedList2[2].Item1, OrderedList2[3].Item1 - OrderedList2[2].Item1);
                            if (t >= 0 && t <= 1)
                            {
                                var r = OrderedList2[1];
                                OrderedList2[1] = OrderedList2[2];
                                OrderedList2[2] = r;
                            }
                            t = SkewedLineIntersectionOnlyT1(OrderedList2[0].Item1, OrderedList2[5].Item1 - OrderedList2[0].Item1,
                                OrderedList2[4].Item1, OrderedList2[1].Item1 - OrderedList2[4].Item1);
                            if (t >= 0 && t <= 1)
                            {
                                var r = OrderedList2[4];
                                OrderedList2[4] = OrderedList2[5];
                                OrderedList2[5] = r;
                            }
                            return OrderedList2;
                        }
                    }
                }
            }
            return OrderedList2;
        }
        private static Vector3 GetNextPoint(Vector3 start,
            Dictionary<PolygonalFace, List<(Vector3, Edge)>> faceStatus, List<Vector3> loop,
            ref Edge CurrentEdge, ref PolygonalFace CurrentFace)
        {
            var TwoFaces = new List<PolygonalFace>();
            TwoFaces.Add(CurrentEdge.OtherFace);
            TwoFaces.Add(CurrentEdge.OwnedFace);

            foreach (var f in TwoFaces)
            {
                if (faceStatus.ContainsKey(f))
                {
                    var lines = faceStatus[f];
                    for (var i = 0; i < lines.Count; i += 2)
                    {
                        if (lines[i].Item1 == start && !loop.Contains(lines[i + 1].Item1))
                        {
                            CurrentEdge = lines[i + 1].Item2;
                            CurrentFace = f;
                            return lines[i + 1].Item1;
                        }
                        else if (lines[i + 1].Item1 == start && !loop.Contains(lines[i].Item1))
                        {
                            CurrentEdge = lines[i].Item2;
                            CurrentFace = f;
                            return lines[i].Item1;
                        }
                    }
                }
            }
            foreach (var f in faceStatus)
            {
                var lines = faceStatus[f.Key];
                for (var i = 0; i < lines.Count; i += 2)
                {
                    if (lines[i].Item1 == start && !loop.Contains(lines[i + 1].Item1))
                    {
                        CurrentEdge = lines[i + 1].Item2;
                        CurrentFace = f.Key;
                        return lines[i + 1].Item1;
                    }
                    else if (lines[i + 1].Item1 == start && !loop.Contains(lines[i].Item1))
                    {
                        CurrentEdge = lines[i].Item2;
                        CurrentFace = f.Key;
                        return lines[i].Item1;
                    }
                }
            }


            return loop[0];

        }
        public static bool IsVertexInsideTriangle(PolygonalFace face, Vector3 pointInQuestion,
            bool onBoundaryIsInside = true)
        {
            if (face.Vertices.Count != 3) throw new Exception("Incorrect number of points in triangle");
            var p = pointInQuestion;
            var a = face.Vertices[0].Coordinates;
            var b = face.Vertices[1].Coordinates;
            var c = face.Vertices[2].Coordinates;
            var n = face.Normal;
            return (b - a).Cross(p - a).Dot(n) >= 0 &&
                (c - b).Cross(p - b).Dot(n) >= 0 &&
                (a - c).Cross(p - c).Dot(n) >= 0;
        }
        public static double SkewedLineIntersectionOnlyT1(Vector3 p1, Vector3 n1, Vector3 p2, Vector3 n2)
        {
            var a11 = n1.LengthSquared();
            var a22 = -n2.LengthSquared(); var a12 = -n1.Dot(n2);
            var a21 = -a12;
            var b1 = n1.Dot(p2 - p1);
            var b2 = n2.Dot(p2 - p1);

            var aDetInverse = 1 / (a11 * a22 - a21 * a12);
            //var aInv = new[,] { { a22, -a12 }, {-a21,a11 } };
            return (a22 * b1 - a12 * b2) * aDetInverse;
        }
        public static double SkewedLineIntersection(Vector3 p1, Vector3 n1, Vector3 p2, Vector3 n2,
            out Vector3 center,
            out Vector3 interSect1, out Vector3 interSect2, out double t1, out double t2)
        {
            //var a11 = n1.X * n1.X + n1.Y * n1.Y + n1.Z * n1.Z;
            var a11 = n1.LengthSquared();
            //var a12 = -n1.X * n2.X - n1.Y * n2.Y - n1.Z * n2.Z;
            var a12 = -n1.Dot(n2);
            //var a21 = n1.X * n2.X + n1.Y * n2.Y + n1.Z * n2.Z;
            var a21 = -a12;
            //var a22 = -n2.X * n2.X - n2.Y * n2.Y - n2.Z * n2.Z;
            var a22 = -n2.LengthSquared();
            //var b1 = n1.X * (p2.X - p1.X) + n1.Y * (p2.Y - p1.Y) + n1.Z * (p2.Z - p1.Z);
            var b1 = n1.Dot(p2 - p1);
            //var b2 = n2.X * (p2.X - p1.X) + n2.Y * (p2.Y - p1.Y) + n2.Z * (p2.Z - p1.Z);
            var b2 = n2.Dot(p2 - p1);
            //var a = new[,] { { a11, a12 }, { a21, a22 } };
            var aDetInverse = 1 / (a11 * a22 - a21 * a12);
            //var aInv = new[,] { { a22, -a12 }, {-a21,a11 } };
            t1 = (a22 * b1 - a12 * b2) * aDetInverse;
            t2 = (-a21 * b1 + a11 * b2) * aDetInverse;
            interSect1 = new Vector3(p1.X + n1.X * t1, p1.Y + n1.Y * t1, p1.Z + n1.Z * t1);
            interSect2 = new Vector3(p2.X + n2.X * t2, p2.Y + n2.Y * t2, p2.Z + n2.Z * t2);
            center = new Vector3((interSect1.X + interSect2.X) / 2, (interSect1.Y + interSect2.Y) / 2,
                (interSect1.Z + interSect2.Z) / 2);
            return interSect1.Distance(interSect2);
        }
        public static List<Vector3> GetEllipticalSegment(List<Vector3> Pts, Cylinder cyl, double epsilon, List<PolygonalFace> faces)
        {

            var ListOfPts = new List<Vector3>();
            for (int i = 0; i < Pts.Count - 1; i++)
            {
                #region Case I: a Line

                if (cyl.Axis.Dot(faces[i].Normal) == 0)
                {
                    ListOfPts.Add(Pts[i]);
                    ListOfPts.Add(Pts[i + 1]);
                }
                #endregion

                #region Case II: Circle
                else if (cyl.Axis.Cross(faces[i].Normal) == Vector3.Zero)
                {
                    var dist = (Pts[i] - Pts[i + 1]).LengthSquared();
                    if (dist > epsilon)
                    {
                        int numOfpts = (int)Math.Ceiling(epsilon / dist);
                        ListOfPts.Add(Pts[i]);

                        var pointonCyl = MiscFunctions.PointOnPlaneFromIntersectingLine(faces[i].Normal,
                            faces[i].Normal.Dot(faces[i].Vertices[0].Coordinates), cyl.Anchor, cyl.Axis, out _);
                        var V1 = Pts[i] - pointonCyl;
                        var V2 = pointonCyl - Pts[i + 1];
                        var angle = V1.SmallerAngleBetweenVectors(V2);
                        for (int j = 1; j < numOfpts + 1; j++)
                        {
                            var angle_i = angle * j / numOfpts;

                        }

                    }
                }
                #endregion

                #region Case III: Ellipse 
                else
                {
                    ListOfPts.Add(Pts[i]);
                    ListOfPts.Add(Pts[i + 1]);
                }
                #endregion
            }

            ListOfPts.Add(Pts[0]);
            return ListOfPts.ToList();
        }
        public static List<Vector3> Get3DCirclePoints(Vector3 point1, Vector3 point2, Vector3 NormalDir, PolygonalFace face, Cylinder cyl)
        {
            var res = new List<Vector3>();
            var Center = (point1 - cyl.Anchor) * cyl.Axis + cyl.Anchor;
            var R = cyl.Radius;
            var Dir = Vector3.Zero;
            var MidPoint = 0.5 * (point1 + point2);

            if (MidPoint == Center)
            {
                Dir = NormalDir.Cross(point1 - Center).Normalize();
                if (!IsVertexInsideTriangle(face, Center + Dir * R))
                {
                    Dir = (point1 - Center).Cross(NormalDir).Normalize();
                }
            }
            else
            {

            }


            return res;
        }
        #endregion

        public static List<List<Vector3>> GetSlices(TessellatedSolid ts, PrimitiveSurface surface, double thickness)
        {
            if (surface is Plane)
            {

            }
            else if (surface is Cylinder)
            {

            }

            else if (surface is Sphere)
            {


            }

            else if (surface is Cone)
            {

            }

            else
            {

            }

            var R = new List<List<Vector3>>();
            return R;

        }

        /*
        public static Dictionary<Edge, (int, Vector3, Vector3)> GetSlicesWMatrix(TessellatedSolid ts, Matrix3x3 matrix)
        {
            var Line1 = new Vector3(matrix.M11, matrix.M12, matrix.M13);
            var Line2 = new Vector3(matrix.M21, matrix.M22, matrix.M33);
            var Line3 = new Vector3(matrix.M31, matrix.M32, matrix.M33);


            var EdgeSphIntersect = new Dictionary<Edge, (int, Vector3, Vector3)>();
            foreach (var f in ts.Faces)
            {
                var p0 = f.A.Coordinates;
                var d1 = f.B.Coordinates - f.A.Coordinates;
                var d2 = f.C.Coordinates - f.A.Coordinates;



                var Qu = new Vector3(d1.Dot(Line1), d1.Dot(Line2), d1.Dot(Line3));
                var Qp = new Vector3(p0.Dot(Line1), p0.Dot(Line2), p0.Dot(Line3));
                var uQu = Qu.Dot(d1) - 1;
                var uQp = Qp.Dot(d1) - 1;
                var pQp = Qp.Dot(p0) - 1;


                var a = uQu;
                var b = uQp;
                var c = pQp;

                var SQRT = b * b - (4 * a * c);
                var NumberOfIntersection = 0;
                if (SQRT >= 0)
                {
                    var V1 = Vector3.Null; var V2 = Vector3.Null;

                    var t1 = (-b + Math.Sqrt(SQRT)) / (2 * a);
                    var t2 = (-b - Math.Sqrt(SQRT)) / (2 * a);
                    if (t1 >= 0 && t1 <= 1)
                    {
                        V1 = edge.From.Coordinates + t1 * d;
                        NumberOfIntersection = 1;
                        if (t2 >= 0 && t2 <= 1)
                        {
                            NumberOfIntersection = 2;
                            V2 = edge.From.Coordinates + t2 * d;
                        }
                    }
                    else if (t2 >= 0 && t2 <= 1)
                    {
                        NumberOfIntersection = 1;
                        V1 = edge.From.Coordinates + t2 * d;
                    }
                    if (NumberOfIntersection > 0)
                    {
                        EdgeSphIntersect.Add(edge, (NumberOfIntersection, V1, V2));
                    }
                }

            }
            return EdgeSphIntersect;

        }
        public static Matrix3x3 GetSphereMatrix(TessellatedSolid ts, Sphere sph)
        {

            Matrix3x3 Matrix1 = new Matrix3x3(sph.Radius, 0.0, 0.0,
                                                    0.0, sph.Radius, 0.0,
                                                    0.0, 0.0, sph.Radius);
            var MatrixDet = Matrix1.GetDeterminant();
            var M = sph.Radius / (MatrixDet);
            Matrix3x3 SphereMatrix = new Matrix3x3(M, 0.0, 0.0,
                                         0.0, M, 0.0,
                                         0.0, 0.0, M);
            return SphereMatrix;
        }
        public static List<Vector3> GetTriCylIntersection(Cylinder cyl, PolygonalFace tri)
        {

            var u = tri.A.Coordinates;
            var v = tri.B.Coordinates;
            var w = tri.C.Coordinates;


            return new List<Vector3>();
        }
        */
    }

}
