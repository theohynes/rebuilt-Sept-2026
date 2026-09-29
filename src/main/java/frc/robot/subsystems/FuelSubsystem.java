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

  // Tolerance in RPM to verify flywheel is ready for fuel transfer
  private static final double LAUNCHER_RPM_TOLERANCE = 150.0;

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
    launcherConfig.smartCurrentLimit(50);     // 50A allows snappy recovery during ball compression
    launcherConfig.voltageCompensation(12.0);

    // Ramp rates smooth gear mesh shock while keeping acceleration crisp
    launcherConfig.closedLoopRampRate(0.25);
    launcherConfig.openLoopRampRate(0.25);

    // Velocity Closed-Loop Tuning (Slot 0)
    // NEO 2.0 free speed ~5676 RPM -> Theoretical kV ~ 1.0 / 5676 ≈ 0.000176
    launcherConfig.closedLoop
        .velocityFF(0.000176, ClosedLoopSlot.kSlot0)
        .p(0.00012, ClosedLoopSlot.kSlot0)
        .i(0.0, ClosedLoopSlot.kSlot0)
        .d(0.0005, ClosedLoopSlot.kSlot0);

    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // --- Flywheel Speed Checker ---
  public boolean isLauncherAtTargetSpeed() {
    double targetRPM = SmartDashboard.getNumber("Launcher Target RPM", 3500.0);
    return Math.abs(launcherEncoder.getVelocity() - targetRPM) <= LAUNCHER_RPM_TOLERANCE;
  }

  // --- Subsystem Actions ---

  /** Spools launcher up to target RPM using closed-loop velocity control */
  public void runLauncherClosedLoop() {
    double targetRPM = SmartDashboard.getNumber("Launcher Target RPM", 3500.0);
    launcherPID.setReference(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  /** Feeds fuel only if the launcher is within tolerance */
  public void runFeederInterlocked() {
    double feederVolts = SmartDashboard.getNumber("Launching feeder voltage", LAUNCHING_FEEDER_VOLTAGE);
    if (isLauncherAtTargetSpeed()) {
      feederRoller.setVoltage(feederVolts);
    } else {
      feederRoller.setVoltage(0.0);
    }
  }

  /** Intakes fuel from the floor/terminal */
  public void intake() {
    double feederVolts = SmartDashboard.getNumber("Intaking feeder voltage", INTAKING_FEEDER_VOLTAGE);
    double intakeVolts = SmartDashboard.getNumber("Intaking intake voltage", INTAKING_INTAKE_VOLTAGE);
    feederRoller.setVoltage(-feederVolts);
    intakeLauncherRoller.setVoltage(intakeVolts);
  }

  /** Ejects stuck fuel out through the intake */
  public void yeetEject() {
    double feederVolts = SmartDashboard.getNumber("Intaking feeder voltage", INTAKING_FEEDER_VOLTAGE);
    double intakeVolts = SmartDashboard.getNumber("Intaking intake voltage", INTAKING_INTAKE_VOLTAGE);
    feederRoller.setVoltage(feederVolts);
    intakeLauncherRoller.setVoltage(-intakeVolts);
  }

  /** Open-loop full-voltage launch mode (bypasses PID) */
  public void yeetLaunch() {
    double feederVolts = SmartDashboard.getNumber("Launching feeder voltage", LAUNCHING_FEEDER_VOLTAGE);
    feederRoller.setVoltage(feederVolts);
    intakeLauncherRoller.setVoltage(12.0);
  }

  /** Holds the ball back by backing up the feeder while spinning the flywheel */
  public void spinUp() {
    runLauncherClosedLoop();
    double spinUpFeederVolts = SmartDashboard.getNumber("Spin-up feeder voltage", SPIN_UP_FEEDER_VOLTAGE);
    feederRoller.setVoltage(-spinUpFeederVolts);
  }

  public void stop() {
    feederRoller.setVoltage(0.0);
    intakeLauncherRoller.setVoltage(0.0);
  }

  // --- Command Factories ---

  /** Pre-spins flywheel while keeping fuel back */
  public Command spinUpCommand() {
    return this.run(this::spinUp);
  }

  /** Spins flywheel and feeds automatically when at target velocity */
  public Command launchCommand() {
    return this.run(() -> {
      runLauncherClosedLoop();
      runFeederInterlocked();
    });
  }

  public Command intakeCommand() {
    return this.run(this::intake);
  }

  public Command ejectCommand() {
    return this.run(this::yeetEject);
  }

  public Command yeetTheBallCommand() {
    return this.run(this::yeetLaunch);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Actual Launcher RPM", launcherEncoder.getVelocity());
    SmartDashboard.putBoolean("Launcher At Speed", isLauncherAtTargetSpeed());
  }
}
