// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DriveConstants {
    public static final double AutoModeSpeed = -0.8;  // Move backward to driver station
    public static final double AutoModeDriveTime = 2.2;  // In seconds
  }

  public static final class ElevatorConstants {
    public static final int LeaderId = 1;
    public static final int FollowerId = 2;

    public static final double BottomPos = 0.05;
    public static final double Level1pos = -7;
    public static final double Level2pos = -19.6;

    public static final double JogStep = 0.2;

    public static final double kP = 0.05;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double OutputLimit = 0.3;
  }

  public static final class TrayConstants {
    public static final int MotorId = 3;
    public static final int FeedMotorId = 4;
    public static final int CanDiId = 7;
  
    public static final double InSpeed = 0.5;
    public static final double OutSpeed = 1.0;
    public static final double Speed = 0.8;
    public static final double BackwardSpeed = 0.5;
  }

  public static final class PneumoConstants {
    public static final int HubId = 3;
    public static final int ForwardLeftSolenoidId = 6;
    public static final int ReverseLeftSolenoidId = 7;
    public static final int ForwardRightSolenoidId = 0;
    public static final int ReverseRightSolenoidId = 1;

    public static final double MinPressure = 20;
    public static final double MaxPressure = 60;
  }

  public static final class IntakeConstants {
    public static final int LeaderId = 14;
    public static final int FollowerId = 13;
    public static final double InSpeed = 0.7;
    public static final double OutSpeed = 1.0;
  }

  public static class OperatorConstants {
    public static final int DriverControllerPort = 0;
    public static final int OperatorControllerPort = 1;

    public static final boolean UseTwoControllers = true;

    public static final double TriggerThreshold = 0.2;

    public static final double LinCoef = 0.15;
    public static final double Threshold = 0.0;
    public static final double CuspX = 0.9;
    //TODO: Tune before competition!
    public static final double SpeedLimitX = 0.5;
    public static final double MinLimit = 0.4;

    public static final double RotLinCoef = 0.2;
    public static final double RotThreshold = 0.0;
    public static final double RotCuspX = 0.5;
    //TODO: Tune before competition!
    public static final double SpeedLimitRot = 0.8;
  }

  /**
   * Vision system constants (Coral robot only - single camera).
   */
  public static final class VisionConstants {
    /** PhotonVision camera name (must match name in PhotonVision UI) */
    public static final String CAMERA_NAME = "Logitech_Webcam_C930e";

    // Distance to AprilTag in inches
    public static final double TargetDistance = 27;

    /** P-controller gain for rotation alignment */
    public static final double ROTATION_P = 0.07;

    /** P-controller gain for forward drive control */
    public static final double DRIVE_P = 0.15;//0.1

    /** Angle tolerance for alignment completion (degrees) */
    public static final double ANGLE_TOLERANCE = 2.0;

    /** Target area percentage for desired distance (~1.5-2m away) */
    public static final double AREA_TARGET = 8.0;

    /** Area tolerance for distance completion */
    public static final double AREA_TOLERANCE = 1.0;

    /** Minimum rotation speed (rad/s) */
    public static final double MIN_ROTATION_SPEED = 0.1;

    /** Maximum rotation speed (rad/s) */
    public static final double MAX_ROTATION_SPEED = 2.0;

    /** Minimum drive speed (m/s) */
    public static final double MIN_DRIVE_SPEED = 0.2;

    /** Maximum drive speed (m/s) */
    public static final double MAX_DRIVE_SPEED = 1.5;

    /** Maximum yaw error before stopping forward drive (degrees) */
    public static final double MAX_YAW_ERROR_FOR_DRIVE = 15.0;
  }

  public static class LightConstants {
    public static final int CanId = 6;
  }
}
