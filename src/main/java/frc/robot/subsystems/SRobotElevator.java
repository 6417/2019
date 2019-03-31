/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import ch.fridolinsrobotik.utilities.Algorithms;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Motors;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class SRobotElevator extends Subsystem {
  private double liftMasterOutput = 0;
  private double elevatorRightOutput = 0;
  private double elevatorLeftOutput = 0;

  private double dLM;
  private double levelingElevator;
  private int lastPosLiftingUnit; 
  private double liftingBreak = 0; 

  private final double liftingUnitElevatorRatio = 100;
  private double normalizeFactor = 1;

  private boolean m_isHomed = false;

  public boolean isHomed() {
    return m_isHomed;
  }

  public double getDLM() {
    return dLM;
  }

  public double getLevelingElevator() {
    return levelingElevator;
  }

  public double getLiftingBreak() {
    return liftingBreak;
  }

  public double getLiftingUnitElevatorRatio() {
    return liftingUnitElevatorRatio;
  }

  public double getNormalizeFactor() {
    return normalizeFactor;
  }

  public double getLiftMasterOutput() {
    return liftMasterOutput;
  }

  public double getElevatorRightOutput() {
    return elevatorRightOutput;
  }

  public double getElevatorLeftOutput() {
    return elevatorLeftOutput;
  }

  /**Calculates the diffrence between the LiftingUnit Encoder and the Elevator
   * Master Encoderticks.
   */
  public double calculateDLM() {
    dLM = Math.tanh(
      (
          (Motors.robotElevatorRight.getSelectedSensorPosition(0) / liftingUnitElevatorRatio)
          - (8300 - Robot.liftingUnit.getPosition())
      ) / 500.0) * 0.5;
    return dLM;
  }

  /**Calculates the diffrence between the two Swervelifts in Encoderticks */
  public double calculateLevelingElevator() {
    levelingElevator = Math.tanh(
        (Motors.robotElevatorRight.getSelectedSensorPosition(0) - Motors.robotElevatorLeft.getSelectedSensorPosition(0))
         / liftingUnitElevatorRatio / 500.0) * 0.5;
    return levelingElevator;
  }

  public double calculateBreak() {
    if(Robot.liftingUnit.getPosition() <= 1000) {
      if(Robot.liftingUnit.getPosition() >= 500) {
        if(liftMasterOutput > 0) {
          liftingBreak = liftingBreak - (Math.abs(lastPosLiftingUnit) - Math.abs(Robot.liftingUnit.getPosition())) * 0.2;  
        } else {
        liftingBreak = liftingBreak + (Math.abs(lastPosLiftingUnit) - Math.abs(Robot.liftingUnit.getPosition())) * 0.2;
        }
        liftingBreak = Algorithms.limit(liftingBreak, -1, 1);
      } else {
        liftingBreak = 1;
      }
    } else {
      liftingBreak = 0;
    }
    lastPosLiftingUnit = Robot.liftingUnit.getPosition();

    return liftingBreak;
  }

  public void elevate(double value) {
    if(!m_isHomed) {
      stop();
      return;
    }
    Motors.liftMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled);

    if(Robot.liftingUnit.getPosition() <= 1000) {
      Motors.liftMaster.set(ControlMode.Position, 0, DemandType.ArbitraryFeedForward, -0.6);
      Motors.robotElevatorRight.set(ControlMode.PercentOutput, value);
      Motors.robotElevatorLeft.set(ControlMode.PercentOutput, value);
    } else {
      liftMasterOutput = - value - calculateDLM() + calculateLevelingElevator();
      elevatorRightOutput =  value - calculateDLM() - calculateLevelingElevator();
      elevatorLeftOutput = value - calculateDLM() + calculateLevelingElevator();

      Double[] outputs = {liftMasterOutput, elevatorRightOutput, elevatorLeftOutput};
      for(Double output : outputs) {
        normalizeFactor = Math.max(Math.abs(output), normalizeFactor);
      }

      Motors.liftMaster.set(ControlMode.PercentOutput, liftMasterOutput / normalizeFactor);
      Motors.robotElevatorRight.set(ControlMode.PercentOutput, elevatorRightOutput / normalizeFactor);
      Motors.robotElevatorLeft.set(ControlMode.PercentOutput, elevatorLeftOutput / normalizeFactor);
    
    }

  }

  public void moveElevators(double rightStick, double leftStick) {
    Motors.robotElevatorRight.set(ControlMode.PercentOutput, leftStick);
    Motors.robotElevatorLeft.set(ControlMode.PercentOutput, rightStick);
  }

  public void stop() {
    Motors.robotElevatorLeft.set(ControlMode.PercentOutput, 0);
    Motors.robotElevatorRight.set(ControlMode.PercentOutput, 0);
  }

  public void checkLimitSwitches() {
    if (!Motors.robotElevatorLeft.getSensorCollection().isRevLimitSwitchClosed()) {
      Motors.robotElevatorLeft.getSensorCollection().setQuadraturePosition(0,0);
      m_isHomed = true;
    }
    if (!Motors.robotElevatorRight.getSensorCollection().isFwdLimitSwitchClosed()) {
      Motors.robotElevatorRight.getSensorCollection().setQuadraturePosition(0,0);
      m_isHomed = true;
    }
  }

  
  // @Override
  // public void initSendable(SendableBuilder builder) {
  //   super.initSendable(builder);
  //   builder.addDoubleProperty("Motor Speed", Motors.liftMaster::getMotorOutputPercent, Motors.liftMaster::set);
  //   builder.addBooleanProperty("Reverse Limit", Motors.liftMaster.getSensorCollection()::isRevLimitSwitchClosed, null);
  //   builder.addBooleanProperty("Forward limit", Motors.liftMaster.getSensorCollection()::isFwdLimitSwitchClosed, null);
  //   builder.addBooleanProperty("Zeroed", this::isZeroed, null);
  //   builder.addDoubleProperty("Position (mm)", this::getPosition, null);
  //   builder.addDoubleProperty("Position raw (pulses)", Motors.liftMaster::getSelectedSensorPosition, null);
  //   // builder.addDoubleProperty("Target position (mm)", this::getTargetPosition, this::setTargetPosition);
  //   builder.addDoubleProperty("Distance per Pulse", encoderConverter::getDistancePerPulse,
  //       encoderConverter::setDistancePerPulse);
  //   builder.addBooleanProperty("Enable Motion Magic", this::isAutonomousEnabled, this::enableAutonomous);
  //  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
