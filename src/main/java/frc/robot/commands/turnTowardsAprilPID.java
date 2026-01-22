package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Drivetrain;
import frc.robot.Robot;

public class turnTowardsAprilPID extends Command {
private Drivetrain lDrivetrain;
private Robot lRobot;
private double m_goalPos = 0;
private double wrappedAngle = 0;
private double m_initialPos = 0;
private double m_currentPos = 0;
private double m_Period = Constants.kDefaultPeriod;
private final ProfiledPIDController m_PIDController;
private int[] l_tagsToCheck;
private Timer m_timer = new Timer();

  /**
   *Creates a new command which will spin the robot in place a given angle.  This function optimizes to minimize the
   * angle travelled to between - pi and pi
   * @param angle in Radians, positive is counter clockwise; 
   * @param drivetrain
*/
  public turnTowardsAprilPID(int[] tagsToCheck, double p_Period, Drivetrain driveTrain, Robot robot) {
    l_tagsToCheck = tagsToCheck;
    lDrivetrain = driveTrain;
    lRobot = robot;
    m_Period = p_Period;
    addRequirements(lDrivetrain);
    

    m_PIDController =
    new ProfiledPIDController(
      .1, 
      0,
      0, 
      new TrapezoidProfile.Constraints(
                  9,
                    48));
  m_PIDController.setTolerance(.1);
  }

  @Override
  public void initialize() {
    //Run once, at the start of the command
    m_initialPos = lDrivetrain.m_odometry.getPoseMeters().getRotation().getRadians();
    
    wrappedAngle = MathUtil.angleModulus(lRobot.getAprilTx(l_tagsToCheck)); //Wrap the angle to be between -pi and pi
    
    
    System.out.print("wrappedAngle = ");
    System.out.println(wrappedAngle);
    

    System.out.print("InitPos = ");
    System.out.println(m_initialPos);
    
    
    m_goalPos = m_initialPos + wrappedAngle;

    System.out.print("GoalPos = ");
    System.out.println(m_goalPos);

  }

  @Override
  public void execute() {
    //run repeatedly, until isFinished() returns true
    m_currentPos = lDrivetrain.m_odometry.getPoseMeters().getRotation().getRadians();
    //m_currDistance = Math.abs(m_currentPos - m_initialPos);
    System.out.print("Current Pos = ");
    System.out.println(m_currentPos);
    /*
    System.out.print("Current Dist = ");
    System.out.println(m_currDistance);*/
    double aprilRot = MathUtil.clamp(m_PIDController.calculate(m_currentPos, m_goalPos), 
        -Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, 
        Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond);

    lDrivetrain.drive(0,0,
      MathUtil.clamp(m_PIDController.calculate(m_currentPos, m_goalPos), 
        -Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, 
        Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond), 
      false, 
      m_Period);

      SmartDashboard.putNumber("CurrentPosAPril", m_currentPos);
      SmartDashboard.putNumber("GoalPos", m_goalPos);
      SmartDashboard.putNumber("AprilError", m_PIDController.getPositionError());
      SmartDashboard.putNumber("AprilRotCommand", aprilRot);
  }

  @Override
  public void end(boolean interrupted) {
    //Run once, at the end of the command
    lDrivetrain.drive(0, 0, 0, false, m_Period);

  }

  @Override
  public boolean isFinished() {
    // Determines when to finish the command
    //return m_PIDController.atGoal();
    boolean rv = false;
    if (m_PIDController.atGoal()) {
      m_timer.start();
    } else {
      m_timer.reset();
    }
    System.out.println("Timer value");
    System.out.print(m_timer.get());
    if (m_timer.hasElapsed(0.5)) {
      rv = true;
    }
    return rv;
    //return true;    //for simulation
   
  }





   
}


