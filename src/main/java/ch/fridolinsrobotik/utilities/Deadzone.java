/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.utilities;

/**
 * Add your docs here.
 */
public class Deadzone {

    public static double getAxis(double inputAxis, double inputDeadzone) {
        double outputAxis = 0;

        // Tests if the Value is positive or negative
        if (inputAxis > inputDeadzone) {
            outputAxis = Algorithms.scale(inputAxis, inputDeadzone, 1, 0, 1);
        } else if (inputAxis < -inputDeadzone) {
            outputAxis = Algorithms.scale(inputAxis, -inputDeadzone, -1, 0, -1);
        }
        return outputAxis;
    }

    public static boolean isInsideDeadzone(double xAxis, double yAxis, double deadzoneValue) {
		
		if(xAxis <= deadzoneValue && xAxis >= -deadzoneValue && yAxis <= deadzoneValue && yAxis >= -deadzoneValue) {
			return true;
		}else {
			return false;
		}
	}

}
