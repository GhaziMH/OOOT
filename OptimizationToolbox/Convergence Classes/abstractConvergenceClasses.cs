﻿using System;
using System.Collections.Generic;
using System.Xml.Serialization;

namespace OptimizationToolbox
{
    [XmlInclude(typeof(DeltaFConvergence)), XmlInclude(typeof(DeltaGradFConvergence)), XmlInclude(typeof(DeltaXConvergence)),
    XmlInclude(typeof(MaxAgeConvergence)), XmlInclude(typeof(MaxIterationsConvergence)), XmlInclude(typeof(MaxTimeConvergence)),
    XmlInclude(typeof(ToKnownBestConvergence)), XmlInclude(typeof(MultipleANDConvergenceConditions)),
    XmlInclude(typeof(MaxDistanceInPopulationConvergence))]
    public abstract class abstractConvergence
    {
        public abstract Boolean converged(long YInteger, double YDouble = double.NaN,
            IList<double> YDoubleArray1 = null, IList<double> YDoubleArray2 = null, 
            IList<double[]> YJaggedDoubleArray = null);
    }

}