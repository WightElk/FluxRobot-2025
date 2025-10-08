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

  public static final class ElevatorConstants {
    public static final int LeaderId = 1;
    public static final int FollowerId = 2;

    public static final double BottomPos = -0.5;
    public static final double Level1pos = -7;
    public static final double Level2pos = -19.5;

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
  
    public static final double Speed = 1.0;
    public static final double InSpeed = 0.5;
    public static final double OutSpeed = 1.0;
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

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final double TriggerThreshold = 0.1;

    public static final double LinCoef = 0.2;
    public static final double Threshold = 0.0;
    public static final double CuspX = 0.9;
    public static final double SpeedLimitX = 0.4;
    public static final double SpeedLimitRot = 0.4;
  }
}
