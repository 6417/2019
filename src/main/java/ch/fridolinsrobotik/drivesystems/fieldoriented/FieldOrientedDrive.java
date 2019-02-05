/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.drivesystems.fieldoriented;

/**
 * Add your docs here.
 */
public class FieldOrientedDrive {

    public static double resultingAngle(double joystickAngle, double fieldAngle) {
		
		if(joystickAngle + fieldAngle > 180) {
			return joystickAngle - fieldAngle - 360;
		}else if(joystickAngle + fieldAngle < -180) {
			return joystickAngle - fieldAngle + 360;
		}else {
			return joystickAngle - fieldAngle;
		}
	}

}
