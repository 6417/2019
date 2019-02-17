/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import ch.fridolinsrobotik.utilities.Deadzone;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SCargoGripper;
import frc.robot.subsystems.SHatchGripper;
import frc.robot.subsystems.SSwerve;
import frc.robot.subsystems.test.TestSCart;
import frc.robot.subsystems.test.TestSLiftingUnit;
import frc.robot.subsystems.test.TestSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI oi;
  public static AHRS ahrs;
  public static PowerDistributionPanel PDP;

  // Create Subsystems
  public static SCargoGripper cargoGripper;
  public static SHatchGripper hatchGripper;
  public static TestSCart testCart;
  public static TestSLiftingUnit testLiftingUnit;
  public static SSwerve swerveDrive;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  TestSubsystem m_testSubsystem;
  SendableChooser<TestSubsystem> m_testSubsystemChooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    Motors.initialize();

    if (RobotMap.CARGO_GRIPPER_SUBSYSTEM_IS_IN_USE) {
      cargoGripper = new SCargoGripper();
    }
    if (RobotMap.HATCH_GRIPPER_SUBSYSTEM_IS_IN_USE) {
      hatchGripper = new SHatchGripper();
    }
    if (RobotMap.SWERVE_DRIVE_SUBSYSTEM_IS_IN_USE) {
      swerveDrive = new SSwerve();
    }

    oi = OI.getInstance();

    PDP = new PowerDistributionPanel(62);

    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      System.out.println("Error instantiating navX-MXP:  " + ex.getMessage());
    }

    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);

    for (TestSubsystem val : TestSubsystem.values()) {
      m_testSubsystemChooser.addOption(val.name(), val);
    }
    m_testSubsystemChooser.setDefaultOption(TestSubsystem.None.name(), TestSubsystem.None);
    SmartDashboard.putData("Test Subsystem", m_testSubsystemChooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    if(RobotMap.SWERVE_DRIVE_SUBSYSTEM_IS_IN_USE) {

      SmartDashboard.putNumber("Joystick 1 X", -OI.JoystickMainDriver.getX());
      double joystickX = Deadzone.getAxis(OI.JoystickMainDriver.getX(), RobotMap.DEADZONE_RANGE);
      double joystickY = Deadzone.getAxis(-OI.JoystickMainDriver.getY(), RobotMap.DEADZONE_RANGE);
      double joystickZ = Deadzone.getAxis(-OI.JoystickMainDriver.getZ(), RobotMap.DEADZONE_RANGE);

      swerveDrive.manualDrive(joystickX, joystickY, joystickZ, ahrs.getYaw());
    }
  }

  @Override
  public void testInit() {
    m_testSubsystem = m_testSubsystemChooser.getSelected();
    if (m_testSubsystem == null) {
      return;
    }
    switch (m_testSubsystem) {
    case Cart: {
      testCart = new TestSCart();
    }
      break;
    case LiftingUnit: {
      testLiftingUnit = new TestSLiftingUnit();
    }
      break;

    default: {

    }
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    if (m_testSubsystem == null) {
      return;
    }
    switch (m_testSubsystem) {
    case Cart: {
      testCart.checkZeroPosition();
      testCart.drive(-OI.JoystickMainDriver.getY());
    }
      break;
    case LiftingUnit: {
      testLiftingUnit.checkZeroPosition();
      testLiftingUnit.drive(-OI.JoystickMainDriver.getY());
    }

    default: {

    }
    }
  }
}
