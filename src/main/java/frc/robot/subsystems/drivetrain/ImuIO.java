package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public class ImuIO {
  @AutoLog
  public static class ImuIOInputs {
    public double yawPositionRad;
    public double pitchPositionRad;
    public double rollPositionRad;
    public double yawVelocityRadPerSec;
    public double pitchVelocityRadPerSec;
    public double rollVelocityRadPerSec;
  }
}
