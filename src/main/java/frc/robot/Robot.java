/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import ch.fridolinsrobotik.drivesystems.swerve.SwerveDrive;
import ch.fridolinsrobotik.utilities.Algorithms;
import ch.fridolinsrobotik.utilities.Deadzone;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.groups.CHatchGrab;
import frc.robot.commands.groups.CHatchHandOut;
import frc.robot.commands.liftingunit.CLiftingUnitOrderdHeight;
import frc.robot.subsystems.SCargoGripper;
import frc.robot.subsystems.SCart;
import frc.robot.subsystems.SHatchGripper;
import frc.robot.subsystems.SLiftingUnit;
import frc.robot.subsystems.SSwerve;

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
  public static SCart cart;
  public static SLiftingUnit liftingUnit;
  public static SSwerve swerveDrive;
  public static SwerveDrive swerve = new SwerveDrive();
  public static CLiftingUnitOrderdHeight liftingUnitOrderHeight;
  // Shuffleboard
  public static ShuffleboardTab shuffleSettings = Shuffleboard.getTab("Settings");
  public static ShuffleboardTab shuffleSubsystems = Shuffleboard.getTab("Subsystems");
  public static NetworkTable raspberry = NetworkTableInstance.getDefault().getTable("RaspberryPIControlSystem");

  public static SwerveModule frontLeft;
  public static SwerveModule frontRight;
  public static SwerveModule backLeft;
  public static SwerveModule backRight;

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
    if (RobotMap.LIFTING_UNIT_SUBSYSTEM_IS_IN_USE) {
      liftingUnit = new SLiftingUnit();
      liftingUnitOrderHeight = new CLiftingUnitOrderdHeight();
    }
    if (RobotMap.CART_SUBSYSTEM_IS_IN_USE) {
      cart = new SCart();
    }

    oi = OI.getInstance();

    PDP = new PowerDistributionPanel(62);

    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      System.out.println("Error instantiating navX-MXP:  " + ex.getMessage());
    }
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
    if(RobotMap.LIFTING_UNIT_SUBSYSTEM_IS_IN_USE) {
      liftingUnit.checkZeroPosition();
    }
    if(RobotMap.CART_SUBSYSTEM_IS_IN_USE) {
      cart.checkLimitSwitches();
    }

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
    if(RobotMap.LIFTING_UNIT_SUBSYSTEM_IS_IN_USE) {
      liftingUnitOrderHeight.start();
    }
    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    // double t1 = Timer.getFPGATimestamp();
    // DriverStation.reportWarning(Double.valueOf(Timer.getFPGATimestamp() - t1).toString(), false);
    double joystickX = OI.JoystickMainDriver.getX();
    double joystickY = -OI.JoystickMainDriver.getY();
    double joystickZ = OI.JoystickMainDriver.getZ();

    double joystickYsupport = Deadzone.getAxis(-OI.JoystickSupportDriver.getY(Hand.kLeft), RobotMap.DEADZONE_RANGE);
    double joystickXsupport = Deadzone.getAxis(OI.JoystickSupportDriver.getX(Hand.kLeft), RobotMap.DEADZONE_RANGE);
    double joystickZrotateSupport = Deadzone.getAxis(-OI.JoystickSupportDriver.getRawAxis(5), RobotMap.DEADZONE_RANGE);

    if (RobotMap.SWERVE_DRIVE_SUBSYSTEM_IS_IN_USE) {
      swerveDrive.manualDrive(joystickX, joystickY, -joystickZ, ahrs.getYaw());
      // swerveDrive.driveCartesian();
    }

    if (RobotMap.LIFTING_UNIT_SUBSYSTEM_IS_IN_USE) {
      // if(RobotMap.HATCH_GRIPPER_SUBSYSTEM_IS_IN_USE) {
      //   if(OI.HatchGripperButtonExtend.get()) {
      //     OI.HatchGripperButtonExtend.toggleWhenPressed(new CHatchGrab());
      //   } else if(OI.HatchGripperButtonRetract.get()) {
      //     OI.HatchGripperButtonRetract.toggleWhenPressed(new CHatchHandOut());
      //   } else if(cart.isTargetPositionReached()) {
      //     liftingUnitOrderHeight.start();
      //   } else {
      //     System.out.println("Couldnt start the OrderHeiht Command");
      //   }
      // } else {
      //   liftingUnitOrderHeight.start();
      // }

      if(OI.JoystickSupportDriver.getPOV(RobotMap.SUPPORT_POV_CHANNEL_ID) == 0) {
        // liftingUnit.enableAutonomous(false);
        // liftingUnit.drive(joystickZrotateSupport);
      } else {
        // liftingUnit.enableAutonomous(true);
        // // liftingUnit.setTargetPosition(Algorithms.limit(joystickZrotateSupport, 0, 1) * 5000);
        // liftingUnit.setTargetPosition(7500);
        // liftingUnit.drive();
      }
      if (OI.JoystickSupportDriver.getRawButton(3)) {
        Motors.liftFollower.setSelectedSensorPosition(0);
      }
      // System.out.println(joystickZrotateSupport);
    }
    if (RobotMap.CART_SUBSYSTEM_IS_IN_USE) {
      if (OI.JoystickSupportDriver.getRawButton(1)) {
        cart.enableAutonomous(false);
        cart.drive(joystickYsupport);
      } else {
        // cart.setPosition(
        // Algorithms.limit(joystickYsupport * RobotMap.CART_DRIVE_LENGTH_MM, 0, RobotMap.CART_DRIVE_LENGTH_MM));
        // cart.enableAutonomous(true);
        // cart.drive();
      }
    }

    // if(RobotMap.CARGO_GRIPPER_SUBSYSTEM_IS_IN_USE) {
    // if(OI.JoystickSupportDriver.getRawAxis(2) > 0.5) {
    // cargoGripper.push(OI.JoystickSupportDriver.getRawAxis(2));
    // } else if(OI.JoystickSupportDriver.getRawAxis(3) >0.5) {
    // cargoGripper.pull(OI.JoystickSupportDriver.getRawAxis(3));
    // } else {
    // cargoGripper.stop();
    // }
    // }
  }

}
