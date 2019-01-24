/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.motorcontrollers;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 * 
 * 
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

	private LimitSwitchNormal convertFridolinLimitSwitchPolarityToSparkMaxPolarity(FridolinsLimitSwitchPolarity polarity) {
        switch(polarity) {
            case kNormallyOpen:
                return LimitSwitchNormal.NormallyOpen;
            case kNormallyClosed:
                return LimitSwitchNormal.NormallyClosed;
            default:
                return LimitSwitchNormal.Disabled;
        }
    }

	@Override
	public void enableForwardLimitSwitch(FridolinsLimitSwitchPolarity polarity, boolean enable) {
		if(!enable) {
			polarity = FridolinsLimitSwitchPolarity.kDisabled;
		}
		super.configForwardLimitSwitchSource(
				LimitSwitchSource.FeedbackConnector, 
				convertFridolinLimitSwitchPolarityToSparkMaxPolarity(polarity)
			);
	}

	@Override
	public void enableReverseLimitSwitch(FridolinsLimitSwitchPolarity polarity, boolean enable) {
		if(!enable) {
			polarity = FridolinsLimitSwitchPolarity.kDisabled;
		}
		super.configReverseLimitSwitchSource(
				LimitSwitchSource.FeedbackConnector, 
				convertFridolinLimitSwitchPolarityToSparkMaxPolarity(polarity)
			);
	}

	private NeutralMode convertFridolinIdleModeType(FridolinsIdleModeType type) {
		switch(type) {
            case kBrake:
                return NeutralMode.Brake;
			default:
				return NeutralMode.Coast;
		}
	}


	@Override
	public void setIdleMode(FridolinsIdleModeType type) {

		super.setNeutralMode(convertFridolinIdleModeType(type));

	}

	@Override
	public double getEncoderTicks() {

		super.getSelectedSensorPosition();

		return getSelectedSensorPosition();

	}



}