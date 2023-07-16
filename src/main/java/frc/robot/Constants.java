package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final Mode kCurrentMode = Mode.kSim;

  public static enum Mode {
    kReal,
    kSim,
    kReplay
  }

  public static final String kCANFDBus = "CANivore1";
  public static final boolean kUseFOC = true;

  public static final class DriveConstants {
    public static final double kWheelDiameter = Units.inchesToMeters(4.0);
    public static final double kWheelRadius = kWheelDiameter / 2.0;
    public static final double kWheelCircumference = kWheelDiameter * Math.PI;
    public static final double kTrackwidthMeters = Units.inchesToMeters(26.0);
    public static final double kWheelbaseMeters = Units.inchesToMeters(26.0);
    public static final double kDriveRatio = 6.75;
    public static final double kTurnRatio = 150.0 / 7.0;
    public static final double kDriveMetersPerRotation = (1.0 / kDriveRatio) * kWheelCircumference;

    public static final double kDrivekP = 0.05;
    public static final double kDrivekS = 0.0;
    public static final double kDrivekV = 2.5039;
    public static final double kDrivekA = 0.99695;

    public static final double kSteerkP = 0.3;
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
    public static final int kIMU = 13;
  }
}
