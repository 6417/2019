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
public class JoystickOptimizer {

    public static double getJoystickAngle(double xAxis, double yAxis, double deadzoneValue) {
		
		if(Deadzone.isInsideDeadzone(xAxis, yAxis, deadzoneValue)) {
			return 0;
		}else if(yAxis < 0) {
			if(xAxis < 0) {
				return 180 * Math.atan(xAxis/yAxis) / Math.PI - 180;
			}else {
				return 180 * Math.atan(xAxis/yAxis) / Math.PI + 180;
			}
		}else {
			return 180 * Math.atan(xAxis/yAxis) / Math.PI;
		}
    }
    
    public static double getDrivePower(double xAxis, double yAxis, double deadzoneValue) {
		
		double drivePower = 0;
		
		if(xAxis <= deadzoneValue && xAxis >= -deadzoneValue && yAxis <= deadzoneValue && yAxis >= -deadzoneValue) {
		
			return 0;
		}else {
			
			drivePower = Math.sqrt(Math.pow(xAxis, 2) + Math.pow(yAxis, 2));
			
			if(drivePower > 1) {
				return 1;
			}else {
				return drivePower;
			}
		}	
	}

}
