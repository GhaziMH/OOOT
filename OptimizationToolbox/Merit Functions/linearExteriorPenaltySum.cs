﻿/*************************************************************************
 *     This file & class is part of the Object-Oriented Optimization
 *     Toolbox (or OOOT) Project
 *     Copyright 2010 Matthew Ira Campbell, PhD.
 *
 *     OOOT is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *  
 *     OOOT is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *  
 *     You should have received a copy of the GNU General Public License
 *     along with OOOT.  If not, see <http://www.gnu.org/licenses/>.
 *     
 *     Please find further details and contact information on OOOT
 *     at http://ooot.codeplex.com/.
 *************************************************************************/
using System;

namespace OptimizationToolbox
{
    public class linearExteriorPenaltySum : abstractMeritFunction
    {
        #region Constructor

        public linearExteriorPenaltySum(abstractOptMethod optMethod, double penaltyWeight)
            : base(optMethod, penaltyWeight)
        {
        }

        #endregion

        public override double calcPenalty(double[] point)
        {
            var sum = 0.0;
            double temp;

            foreach (var c in optMethod.h)
            {
                temp = optMethod.calculate(c, point);
                sum += Math.Abs(temp);
            }
            foreach (var c in optMethod.g)
            {
                temp = optMethod.calculate(c, point);
                if (temp > 0) sum += temp;
            }
            sum *= penaltyWeight;
            return sum;
        }

        public override double[] calcGradientOfPenalty(double[] point)
        {
            var n = point.GetLength(0);
            var grad = new double[n];
            for (var i = 0; i != n; i++)
            {
                var sum = 0.0;
                double temp;
                foreach (var c in optMethod.h)
                {
                    temp = optMethod.calculate(c, point);
                    if (temp < 0.0)
                        sum -= optMethod.deriv_wrt_xi(c,point, i);
                    if (temp > 0.0)
                        sum += optMethod.deriv_wrt_xi(c, point, i);
                }
                foreach (var c in optMethod.g)
                {
                    temp = optMethod.calculate(c, point);
                    if (temp > 0.0)
                        sum += optMethod.deriv_wrt_xi(c, point, i);
                }
                grad[i] = penaltyWeight * sum;
            }
            return grad;
        }
    }
}