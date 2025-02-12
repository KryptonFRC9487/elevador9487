// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ElevatorConstants.ElevatorPose;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {

  private final SubsystemTracker subsystemSupplier = new SubsystemTracker();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(subsystemSupplier);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

     // Elevator Commands
    new POVButton(p2Controller, POV.DOWN)
        .onTrue(new InstantCommand(() -> elevatorSubsystem.setTarget(ElevatorPose.INITAL)));

    new POVButton(p2Controller, POV.LEFT)
      .onTrue(new InstantCommand(() -> elevatorSubsystem.setTarget(ElevatorPose.L3)));

    new POVButton(p2Controller, POV.UP)
        .onTrue(new InstantCommand(() -> elevatorSubsystem.setTarget(ElevatorPose.L4)));

    new POVButton(p2Controller, POV.RIGHT)
        .onTrue(new InstantCommand(() -> elevatorSubsystem.setTarget(ElevatorPose.L2)));
    }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
