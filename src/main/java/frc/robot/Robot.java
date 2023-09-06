// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;
  private double speed = 0.4;
  private double r_speed = speed;
  private double l_speed = speed;

  CANSparkMax FrontLeftMotor = new CANSparkMax(1, MotorType.kBrushless); 
  CANSparkMax FrontRightMotor = new CANSparkMax(2, MotorType.kBrushless); 
  CANSparkMax BackLeftMotor = new CANSparkMax(3, MotorType.kBrushless); 
  CANSparkMax BackRightMotor = new CANSparkMax(4, MotorType.kBrushless); 
  
  MotorController front_L = FrontLeftMotor;
  MotorController front_R = FrontRightMotor;
  MotorController back_L = BackLeftMotor;
  MotorController back_R = BackRightMotor;

  MotorControllerGroup m_left = new MotorControllerGroup(FrontLeftMotor, BackLeftMotor);
  MotorControllerGroup m_right = new MotorControllerGroup(FrontRightMotor, BackRightMotor);
  private final DifferentialDrive m_Drive = new DifferentialDrive(m_left, m_right);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();


    FrontLeftMotor.setInverted(false);
    FrontRightMotor.setInverted(false); 

    FrontLeftMotor.setSmartCurrentLimit(80);
    FrontRightMotor.setSmartCurrentLimit(80); 
    BackLeftMotor.setSmartCurrentLimit(80); 
    BackRightMotor.setSmartCurrentLimit(80); 
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

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

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Update_Limelight_Tracking();

    if (m_LimelightHasValidTarget)
          {
                m_Drive.tankDrive(l_speed, -r_speed);
                SmartDashboard.putString("DB/String 0", "yes" );

              // n_Drive.tankDrive(m_LimelightDriveCommand,m_LimelightSteerCommand);
          }
          else
          {
               // n_Drive.tankDrive(0.1,0.1);
                m_Drive.tankDrive(0.0, 0.0);
                SmartDashboard.putString("DB/String 0", "no" );

          }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public void Update_Limelight_Tracking()
  {
        // These numbers must be tuned for your Robot!  Be careful!
                  // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        SmartDashboard.putString("DB/String 1", String.valueOf(tv));
        SmartDashboard.putString("DB/String 2", String.valueOf(tx));
        SmartDashboard.putString("DB/String 3", String.valueOf(ty));
        SmartDashboard.putString("DB/String 4", String.valueOf(ta));
        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        if (tx > 2) {
          l_speed = speed;
          r_speed -= speed*(tx * 0.02);
          r_speed *= 0.8;
          SmartDashboard.putNumber("DB/String 6", r_speed);
          
        }
        if (tx < -2) {
          r_speed = speed;
          l_speed = speed*(tx * 0.02);
          l_speed *= 0.8;
            SmartDashboard.putNumber("DB/String 7", l_speed);
            l_speed = 0;
        }
        if (tx < 2 && tx > -2) {
          SmartDashboard.putNumber("DB/String 6", r_speed);
          SmartDashboard.putNumber("DB/String 7", l_speed);
          r_speed = speed;
          l_speed = speed;
        }

        // try to drive forward until the target area reaches our desired area
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
