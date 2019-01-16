/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.motorcontrollers;

/**
 * Interface to merge the MotorController classes given by the manufacturers.
 */
public interface IFridolinsMotors {
     
    public void setVelocity(double velocity);

    public void setPosition(double position);

}
