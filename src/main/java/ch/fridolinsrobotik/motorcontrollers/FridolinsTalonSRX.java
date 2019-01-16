/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.motorcontrollers;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */
public class FridolinsTalonSRX extends WPI_TalonSRX implements IFridolinsMotors {

    public FridolinsTalonSRX(int deviceNumber) {

        super(deviceNumber);

    }
    
	@Override
	public void setVelocity(double velocity) {
        
        this.set(velocity);

	}

	@Override
	public void setPosition(double position) {
		
	}



}