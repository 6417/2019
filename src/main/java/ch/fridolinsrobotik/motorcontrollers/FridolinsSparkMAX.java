/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.motorcontrollers;

import com.revrobotics.CANSparkMax;

/**
 * Add your docs here.
 */
public class FridolinsSparkMAX extends CANSparkMax implements IFridolinsMotors {

    public FridolinsSparkMAX(int deviceID, com.revrobotics.CANSparkMaxLowLevel.MotorType type) {

        super(deviceID, type);

    }

    @Override
    public void setVelocity(double velocity) {

        this.set(velocity);

    }

    @Override
    public void setPosition(double position) {

    }
}
