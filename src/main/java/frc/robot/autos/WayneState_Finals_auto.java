// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;

public class WayneState_Finals_auto extends SequentialCommandGroup {

  public WayneState_Finals_auto(DriveSubsystem driveSubsystem1, FuelSubsystem ballSubsystem) {
    
    addCommands(
        // 1. Start spinning up the launcher AT THE SAME TIME as driving backward.
        // .deadlineWith means the spinUpCommand will stop as soon as the AutoDrive finishes.
        new AutoDrive(driveSubsystem1, -.9, 0.0)
            .withTimeout(0.5)
            .deadlineWith(ballSubsystem.spinUpCommand()),

        // 2. Ensure we are exactly at 4,000 RPM before feeding.
        // Because we started revving while driving, this might finish instantly!
        ballSubsystem.spinUpCommand()
            .until(() -> ballSubsystem.isLauncherAtTargetSpeed()),

        // 3. Fire the Fuel! 
        // We use a timeout here (e.g., 3 seconds) so the command eventually finishes, 
        // and then we safely shut off all motors using finallyDo.
        ballSubsystem.launchCommand()
            .withTimeout(3.0)
            .finallyDo(() -> ballSubsystem.stop())
    );
  }
}
