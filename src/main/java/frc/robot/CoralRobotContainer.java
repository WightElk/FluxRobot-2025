package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TraySubsystem;
import frc.robot.commands.TrayInOutCommand;

/**
 * Robot with Coral Elevator
 */
public class CoralRobotContainer extends RobotContainer {
  private final CANBus canBus = new CANBus(RobotConfig.CoralRobot.systemCANBus);
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final TraySubsystem tray = new TraySubsystem(canBus);

  public CoralRobotContainer() {
    super(RobotConfig.CoralRobot.driveCANBus, RobotConfig.CoralRobot.pigeonId, RobotConfig.CoralRobot.pigeonConfigs,
      RobotConfig.CoralRobot.frontLeft, RobotConfig.CoralRobot.frontRight,
      RobotConfig.CoralRobot.backLeft, RobotConfig.CoralRobot.backRight);
  }

  @Override
  protected void configureBindings() {
    super.configureBindings();

    // Elevator control bindings
    driverController.povDown().whileTrue(new RunCommand(() -> elevator.jogUp(), elevator));
    driverController.povUp().whileTrue(new RunCommand(() -> elevator.jogDown(), elevator));
    driverController.povLeft().or(driverController.povRight()).whileTrue(new RunCommand(() -> elevator.stop(), elevator));

    driverController.y().onTrue(new RunCommand(() -> elevator.moveToLevel2(), elevator));
    driverController.b().onTrue(new RunCommand(() -> elevator.moveToLevel1(), elevator));
    driverController.a().onTrue(new RunCommand(() -> elevator.moveToBottom(), elevator));

    // Tray control bindings
    driverController.rightTrigger(OperatorConstants.TriggerThreshold).whileTrue(new TrayInOutCommand(tray, () -> driverController.getRightTriggerAxis()));
    driverController.leftTrigger(OperatorConstants.TriggerThreshold).whileTrue(new TrayInOutCommand(tray, () -> driverController.getLeftTriggerAxis()));
  }
}
