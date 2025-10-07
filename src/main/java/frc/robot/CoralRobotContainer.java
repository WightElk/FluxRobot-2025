package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TraySubsystem;

/**
 * Robot with Coral Elevator
 */
public class CoralRobotContainer extends RobotContainer {
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final TraySubsystem tray = new TraySubsystem();

  public CoralRobotContainer() {
    super();
  }

  @Override
  protected void configureBindings() {
    super.configureBindings();

    //Elevator bindings
    driverController.povDown().whileTrue(new RunCommand(() -> elevator.jogUp(), elevator));
    driverController.povUp().whileTrue(new RunCommand(() -> elevator.jogDown(), elevator));
    driverController.povLeft().or(driverController.povRight()).whileTrue(new RunCommand(() -> elevator.stop(), elevator));

    driverController.y().onTrue(new RunCommand(() -> elevator.moveToLevel2(), elevator));
    driverController.b().onTrue(new RunCommand(() -> elevator.moveToLevel1(), elevator));
    driverController.a().onTrue(new RunCommand(() -> elevator.moveToBottom(), elevator));
  }
}
