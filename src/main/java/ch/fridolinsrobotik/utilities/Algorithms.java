/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.utilities;

/**
 * Collection of mathematic algortihms
 */
public class Algorithms {

    /**
     * @param x      value to be linear scaled from the original range to output
     *               range
     * @param inMin  original range minimum
     * @param inMax  original range maximum
     * @param outMin output range minimum
     * @param outMax output range maximum
     * @return scaled value from
     */
    public static double scale(double x, double inMin, double inMax, double outMin, double outMax) {
        double result = (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
        result = Math.max(result, outMin);
        result = Math.min(result, outMax);
        return result;
    }

    /**
     * Ease In and Out function used for calculation a factor. We calculate a factor
     * for the speed with a function as follow: x^a ------------- x^a + (1-x)^a
     * @see <a href=
     *      "https://www.wolframalpha.com/input/?i=plot+x%5Ea%2F(x%5Ea%2B(1%E2%88%92x)%5Ea)+where+a+%3D+1.5+from+x%3D0+to+x%3D1">function
     *      as a plot with a=1.5 in Wolframalpha.com</a>
     *
     *      This function defines an easing function between 0 and 1 where 'a'
     *      defines steepness of the slope. Therefore distanceToLimit must be scaled
     *      from cm to a percentage (0-1) if the speedDownWindow. We limit the
     *      minimum speed to 0.1 to let it reach the limit switch.
     * @param x
     * @param a steepness of the slope
     * @return scaling factor
     */
    public static double easeInOut(double x, double a) {
        return Math.pow(x, a) / (Math.pow(x, a) + Math.pow(1 - x, a));
    }

}
