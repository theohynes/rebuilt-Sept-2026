// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FuelConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.autos.AutoDrive;
import frc.robot.autos.drive_and_shoot_bump_QSec;
import frc.robot.autos.drive_and_shoot_forward_halfSec;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.autos.WayneState_Finals_auto;
/** 
import frc.robot.commands.pause;
import frc.robot.commands.SpinUp;
import frc.robot.commands.Eject;
import frc.robot.commands.Intake;
import frc.robot.commands.Launch;
import frc.robot.commands.LaunchSequence;
**/

//@TODO: create arm subsystem and import it here
//import frc.robot.subsystems.armSubsystem;
//import frc.robot.commands.POVArmMotorCommand;
//import frc.robot.commands.extendArmToBar;

//import frc.robot.autos.ExampleAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.List;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FuelConstants;
import frc.robot.Constants.OIConstants;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final FuelSubsystem ballSubsystem = new FuelSubsystem();

  //@TODO: create arm subsystem and import it here
  //public static final armSubsystem m_armSubsystem = new armSubsystem(); //arm subsystem


  // The driver's controller
   CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();



        // Create a POV trigger for your controller
    private Trigger povTrigger;

    // create instance of POVMotorCommand as a class-level variable
   
   //@TODO Will be for the arm, not sure if we need it yet
    //private POVArmMotorCommand povMotorCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
        // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption

    
    autoChooser.addOption("WAYNE STATE FINALS", new WayneState_Finals_auto(m_robotDrive, ballSubsystem));

    autoChooser.setDefaultOption("Autonomous_DRIVE_SHOOT", new drive_and_shoot_forward_halfSec(m_robotDrive, ballSubsystem));
    autoChooser.addOption("Autonomous_DRIVE_SHOOT", new drive_and_shoot_bump_QSec(m_robotDrive, ballSubsystem));
   
    autoChooser.addOption("Autonomous_DRIVE", new AutoDrive(m_robotDrive,0.5,  0.0).withTimeout(.25));

    // Configure the button bindings
    configureButtonBindings();
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    false),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
    private void configureButtonBindings() {
  

      // While the left bumper on operator controller is held, intake Fuel
    m_driverController.leftBumper()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));
// While the right bumper on the operator controller is held, spin up until 
    // the target RPM is reached, then launch fuel. When released, stop.
    m_driverController.rightBumper()
        .whileTrue(
            ballSubsystem.spinUpCommand()
                // Waits dynamically instead of using a hardcoded timeout
                .until(() -> ballSubsystem.isLauncherAtTargetSpeed())
                .andThen(ballSubsystem.launchCommand())
                .finallyDo(() -> ballSubsystem.stop())
        );

          
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    m_driverController.a()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop()));

    // Start Button -> Zero swerve heading
    m_driverController.start().onTrue(m_robotDrive.zeroHeadingCommand());

    
   // Create a POV trigger for your controller
    //povTrigger = new Trigger(() -> m_driverController.getPOV() != -1);
    //new POVMotorCommand(m_armSubsystem, m_driverController, povTrigger);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
     /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
  }



