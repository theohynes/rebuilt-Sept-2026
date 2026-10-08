// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FuelConstants.*;

public class FuelSubsystem extends SubsystemBase {

  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;
  private final SparkClosedLoopController launcherPID;
  private final RelativeEncoder launcherEncoder;
  private final SparkClosedLoopController closedLoopController;

  // Tolerance in RPM to verify flywheel is ready for fuel transfer
  private static final double LAUNCHER_RPM_TOLERANCE = 100.0;

  public FuelSubsystem() {
    intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);

    launcherPID = intakeLauncherRoller.getClosedLoopController();
    launcherEncoder = intakeLauncherRoller.getEncoder();

    // Push dashboard tuning defaults
    SmartDashboard.putNumber("Intaking feeder voltage", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake voltage", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder voltage", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder voltage", SPIN_UP_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launcher Target RPM", 3500.0);

    // --- FEEDER CONFIG ---
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.inverted(false); // Inversion fixed once at init; reverse in code via negative voltage
    feederConfig.idleMode(IdleMode.kBrake); // Instant stop prevents double-feeding
    feederConfig.smartCurrentLimit(35);
    feederConfig.voltageCompensation(12.0);

    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // --- LAUNCHER CONFIG ---
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(false);
    launcherConfig.idleMode(IdleMode.kCoast); // Preserves rotational inertia
    launcherConfig.smartCurrentLimit(40);     // 40A allows snappy recovery during ball compression
    launcherConfig.voltageCompensation(12.0);
    SmartDashboard.putNumber("Launcher Target RPM", LAUNCHER_TARGET_RPM);

    // Ramp rates smooth gear mesh shock while keeping acceleration crisp
    launcherConfig.closedLoopRampRate(0.25);
    launcherConfig.openLoopRampRate(0.25);

    // Velocity Closed-Loop Tuning (Slot 0)
    // NEO 2.0 free speed ~5676 RPM -> Theoretical kV ~ 1.0 / 5676 ≈ 0.000176
    launcherConfig.closedLoop
        .velocityFF(0.000176, ClosedLoopSlot.kSlot0)
        .p(0.00012, ClosedLoopSlot.kSlot0)
        .i(0.0, ClosedLoopSlot.kSlot0)
        .d(0.00018, ClosedLoopSlot.kSlot0);

    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  closedLoopController = intakeLauncherRoller.getClosedLoopController();
  }

  // --- Flywheel Speed Checker ---
  public boolean isLauncherAtTargetSpeed() {
double targetRPM = SmartDashboard.getNumber("Launcher Target RPM", LAUNCHER_TARGET_RPM);    return Math.abs(launcherEncoder.getVelocity() - targetRPM) <= LAUNCHER_RPM_TOLERANCE;
  }

public void setPercentageOutput(double Speed){
  intakeLauncherRoller.set(Speed);
}

public void setTargetRPM(double rpm){
  closedLoopController.setReference(rpm, SparkMax.ControlType.kVelocity);
}

public double getRPM(){
return launcherEncoder.getVelocity();
}

  // --- Subsystem Actions ---

  public void intake() {
    feederRoller.setVoltage(SmartDashboard.getNumber("Intaking feeder voltage", INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(SmartDashboard.getNumber("Intaking intake voltage", INTAKING_INTAKE_VOLTAGE));
  }

  public void eject() {
    feederRoller.setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder voltage", INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller.setVoltage(-1 * SmartDashboard.getNumber("Intaking intake voltage", INTAKING_INTAKE_VOLTAGE));
  }

  // Uses closed-loop RPM for the launcher, voltage for the feeder
  public void launch() {
    double targetRPM = SmartDashboard.getNumber("Launcher Target RPM", LAUNCHER_TARGET_RPM);
    feederRoller.setVoltage(SmartDashboard.getNumber("Launching feeder voltage", LAUNCHING_FEEDER_VOLTAGE));
    
    // Actually use your PID controller and target RPM!
    setTargetRPM(targetRPM); 
  }

  public void stop() {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
  }

  public void spinUp() {
    double targetRPM = SmartDashboard.getNumber("Launcher Target RPM", LAUNCHER_TARGET_RPM);
    feederRoller.setVoltage(SmartDashboard.getNumber("Spin-up feeder voltage", SPIN_UP_FEEDER_VOLTAGE));
    
    // Spin up to the target RPM
    setTargetRPM(targetRPM); 
  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    return this.run(() -> launch());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
