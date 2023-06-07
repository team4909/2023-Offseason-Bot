package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.FaultManager;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final Drivetrain m_drivetrain = new Drivetrain();

  @Override
  public void robotInit() {
    configureBindings();
    enableLiveWindowInTest(false);
    addPeriodic(() -> FaultManager.getInstance().poll(), 4);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationPeriodic() {
    m_drivetrain.updateModulesSim();
    m_drivetrain.updateGyroSim();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No auto configured");
  }

  private void configureBindings() {
  }
}
