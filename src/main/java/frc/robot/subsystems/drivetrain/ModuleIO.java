package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

  @AutoLog
  public static class ModuleIOInputs {
    public double drivePositionRotations;
    public double driveVelocityRPS;
    public double driveClosedLoopReference;
    public double driveClosedLoopError;
    public double driveTorqueCurrentAmps;
    public double driveStatorCurrentAmps;
    public double driveSupplyCurrentAmps;
    public double driveSupplyVoltage;
    public double driveTempCelsius;

    public double steerPositionRotations;
    public double steerVelocityRPS;
    public double steerClosedLoopReference;
    public double steerClosedLoopError;
    public double steerTorqueCurrentAmps;
    public double steerStatorCurrentAmps;
    public double steerSupplyCurrentAmps;
    public double steerSupplyVoltage;
    public double steerTempCelsius;

    public double azimuthPositionRotations;
    public double azimuthVelocityRPS;
    public double azimuthAbsolutePositionRotations;
    public double azimuthUnfilteredVelocityRPS;
    public double azimuthPositionSinceBoot;

    public double latencyCompensatedPositionRotations;
  }

  public void updateInputs(ModuleIOInputs inputs);

  public void updateSim();

  void setDriveRPS(double speedRPS);

  void setSteerRotations(double angleRotations);

  void setDriveVoltage(double volts);

  void setSteerVoltage(double volts);

}
