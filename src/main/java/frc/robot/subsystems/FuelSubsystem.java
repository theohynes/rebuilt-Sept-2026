// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;


public class FuelSubsystem extends SubsystemBase {
    @SuppressWarnings("is never used")

  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;
  private final SparkClosedLoopController launcherPID;
  private final RelativeEncoder launcherEncoder;

  /** Creates a new CANBallSubsystem. */
  @SuppressWarnings("removal")
  public FuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    //OG CIMS
   // intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushed);
    //feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushed);

    intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
    launcherPID = intakeLauncherRoller.getClosedLoopController();
    launcherEncoder = intakeLauncherRoller.getEncoder();
    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  SmartDashboard.putNumber("Launcher Target RPM", 3500); // Start safe for PLA+
    SmartDashboard.putNumber("Launching feeder voltage", LAUNCHING_FEEDER_VOLTAGE);

   // --- FEEDER CONFIG ---
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.idleMode(IdleMode.kBrake);
    feederConfig.smartCurrentLimit(30); // Lower limit to prevent heat buildup
    feederConfig.inverted(true); 
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // --- LAUNCHER CONFIG (The Gear Protector) ---
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(false);
    launcherConfig.idleMode(IdleMode.kCoast);
    launcherConfig.smartCurrentLimit(50);
    launcherConfig.voltageCompensation(12.0);
    
    // RAMP RATE: This prevents the "screaming" sudden start
    launcherConfig.closedLoopRampRate(0.5); 
    launcherConfig.openLoopRampRate(0.5);

    // PID Constants for Velocity
    launcherConfig.closedLoop.p(0.0001);
    launcherConfig.closedLoop.i(0);
    launcherConfig.closedLoop.d(0);
    launcherConfig.closedLoop.velocityFF(0.000175); // Adjust based on your gear ratio

    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feederRoller.setInverted(true);
feederRoller.setVoltage(SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller
        .setVoltage(SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void yeetEject() {
            feederRoller.setInverted(true);
  feederRoller
        .setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller
        .setVoltage(-1 * SmartDashboard.getNumber("Intaking launcher roller value", INTAKING_INTAKE_VOLTAGE));
  }
    // A method to set the rollers to values for launching.
  public void yeetLaunch() {
        feederRoller.setInverted(false);

    feederRoller.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
    intakeLauncherRoller
        .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
  }
  //------------------------------------------------------------------------------------------------------------
// A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void normalEject() {
            feederRoller.setInverted(true);

    feederRoller
        .setVoltage(.85 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller
        .setVoltage(-.85 * SmartDashboard.getNumber("Intaking launcher roller value", -INTAKING_INTAKE_VOLTAGE));
  }
  // A method to set the rollers to values for launching.
  public void normalLaunch() {
    double targetRPM = SmartDashboard.getNumber("Launcher Target RPM", 3500);
    double feederVolts = SmartDashboard.getNumber("Launching feeder voltage", LAUNCHING_FEEDER_VOLTAGE);

    // Use PID to maintain steady RPM
    launcherPID.setReference(targetRPM, ControlType.kVelocity);

    // Check if we are within 150 RPM of target before feeding
    if (Math.abs(launcherEncoder.getVelocity() - targetRPM) < 150) {
        feederRoller.setVoltage(feederVolts);
    } else {
        feederRoller.setVoltage(0);
    }

}

  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
    feederRoller
        .setVoltage(SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE));
    intakeLauncherRoller
        .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    return this.run(() -> normalLaunch());
  }
public Command yeetTheBallCommand(){
  return this.run(()-> yeetLaunch());
}
//------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
 // Keep an eye on the actual RPM vs Target on the Dashboard
    SmartDashboard.putNumber("Actual Launcher RPM", launcherEncoder.getVelocity());
 
  }
}