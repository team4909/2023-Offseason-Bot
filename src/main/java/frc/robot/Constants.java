package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final Mode kCurrentMode = Mode.kSim;

  public static enum Mode {
    kReal, kSim, kReplay
  }

  public static final String kCANFDBus = "CANivore1";
  public static final boolean kUseFOC = true;

  public static final class DriveConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2.0;
    public static final double kTrackwidthMeters = Units.inchesToMeters(26.0);
    public static final double kWheelbaseMeters = Units.inchesToMeters(26.0);
    public static final double kDriveRatio = 6.75;
    public static final double kSteerRatio = 150.0 / 7.0;
    // These are surely different for FOC, calculate at some point
    public static final double kMaxLinearSpeedMetersPerSec = Units.feetToMeters(16.3);
    public static final double kMaxAngularSpeedRadPerSec = 10.0;

    // This should be tuned at some point
    public static final double kDrivekP = 0.17;
    public static final double kDrivekS = 0.26015;
    public static final double kDrivekV = 2.5039;
    public static final double kDrivekA = 0.99695;
    public static final double kDrivekF = 0.12; // This should be tuned at some point

    public static final double kSteerkP = 9.61;
  }

  public static final class DeviceIDs {
    public static final int kFrontRightModuleSteerMotor = 1;
    public static final int kFrontRightModuleDriveMotor = 2;
    public static final int kBackRightModuleSteerMotor = 3;
    public static final int kBackRightModuleDriveMotor = 4;
    public static final int kBackLeftModuleSteerMotor = 5;
    public static final int kBackLeftModuleDriveMotor = 6;
    public static final int kFrontLeftModuleSteerMotor = 7;
    public static final int kFrontLeftModuleDriveMotor = 8;
    public static final int kFrontRightModuleAzimuthEncoder = 9;
    public static final int kBackRightModuleAzimuthEncoder = 10;
    public static final int kBackLeftModuleAzimuthEncoder = 11;
    public static final int kFrontLeftModuleAzimuthEncoder = 12;
    public static final int kImu = 13;
  }
}
