/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.motorcontrollers;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.InvertType;
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
	public int getEncoderTicks() {
		return getSelectedSensorPosition();
	}

	@Override
	public boolean isForwardLimitSwitchActive() {
		return getSensorCollection().isFwdLimitSwitchClosed();
	}

	@Override
	public boolean isReverseLimitSwitchActive() {
		return getSensorCollection().isRevLimitSwitchClosed();
	}

	@Override
	public void setDirection(boolean forward) {
		super.setInverted(forward);
	}

	private InvertType convertFridolinDirectionType(FridolinsDirectionType type) {
		switch(type) {
            case followMaster:
                return InvertType.FollowMaster;
			default:
				return InvertType.OpposeMaster;
		}
	}

	@Override
	public void followDirection(FridolinsDirectionType type) {
		super.setInverted(convertFridolinDirectionType(type));
	}

	@Override
	public void follow(IFridolinsMotors master) {
		if(master instanceof IMotorController) {
			super.follow((IMotorController) master);
			return;
		}

		throw new Error("Not a type of IMotorController");
		
	}

	@Override
	public void factoryDefault() {
		super.configFactoryDefault();
	}

	@Override
	public void setSensorDirection(boolean forward) {
		super.setSensorPhase(forward);
	}

	private FeedbackDevice convertFridolinFeedbackDevice(FridolinsFeedbackDevice device) {
		switch(device) {
			default:
				return FeedbackDevice.QuadEncoder;
		}
	}

	@Override
	public void configSelectedFeedbackSensor(FridolinsFeedbackDevice device, int pidIdx, int timeoutMs) {
		super.configSelectedFeedbackSensor(convertFridolinFeedbackDevice(device), pidIdx, timeoutMs);
	}

	@Override
	public void configOpenLoopRamp(double rampTime, int timeoutMs) {
		super.configOpenloopRamp(rampTime, timeoutMs);
	}

	@Override
	public void setSensorPosition(int position) {
		super.setSelectedSensorPosition(0);
	}



}