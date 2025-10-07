package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Pneumatics;

/**
 * Robot with Algae Intake
 */
public class AlgaeRobotContainer extends RobotContainer {
    private final Pneumatics pneumatics = new Pneumatics();

  AlgaeRobotContainer() {
    super();
  }

  @Override
  protected void configureBindings() {
    super.configureBindings();

    // Pneumatics bindings
    driverController.a().onTrue(new InstantCommand(() -> pneumatics.setForward()));
    driverController.b().onTrue(new InstantCommand(() -> pneumatics.setReverse()));
    driverController.x().onTrue(new InstantCommand(() -> pneumatics.setOff()));
    driverController.start().onTrue(new InstantCommand(() -> pneumatics.toggleCompressor()));
  }
}
