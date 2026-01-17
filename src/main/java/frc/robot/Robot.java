// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.commands.driveSidewaysPID;
import frc.robot.commands.driveSpinwaysPID;
import frc.robot.commands.driveStraightPID;
import frc.robot.commands.driveToPositionPID;
import frc.robot.commands.turnTowardsAprilPID;
import frc.robot.subsystems.LEDSubsystem;
import java.util.AbstractMap;
import java.util.HashMap;
import java.util.Map;

public class Robot extends TimedRobot {
  private final CommandXboxController m_controller = new CommandXboxController(0);
  private Trigger yButton     = m_controller.y(); 
  private Trigger xButton     = m_controller.x(); 
  private Trigger aButton     = m_controller.a(); 
  private Trigger bButton     = m_controller.b(); 
  private Trigger startButton = m_controller.start(); 
  private Trigger backButton  = m_controller.back();
  private Trigger lbButton     = m_controller.leftBumper();
  private Trigger rbButton    = m_controller.rightBumper(); 
  private Trigger lBTrigger   = m_controller.leftTrigger(0.1); 
  private Trigger rBTrigger   = m_controller.rightTrigger(.1); 
  private Trigger povUp       = m_controller.povUp();
  private Trigger povDown     = m_controller.povDown();
  private Trigger povLeft     = m_controller.povLeft();
  private Trigger povRight    = m_controller.povRight();
  
  private boolean isHighGear = false;
  private boolean isFieldRelative = false;
  
  private final Drivetrain m_swerve = new Drivetrain();
  private final Field2d m_field = new Field2d();

  private RawFiducial[] fiducials;

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();

  private HashMap<Integer, RawFiducial> aprilTags = new HashMap<Integer, RawFiducial>();
  // Slew rate limiters to make joystick inputs more gentle; Passing in "3" means 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private Command m_autonomousCommand;

  private String m_autoSelected;
  private final SendableChooser<String> m_AutoChooser = new SendableChooser<>();
  private final SendableChooser<String> m_AprilTagSelected = new SendableChooser<>();

  private final LEDSubsystem ledSystem = new LEDSubsystem();

  private LimelightHelpers limelight = new LimelightHelpers();
  int id;                  // Tag ID
  double txnc;             // X offset (no crosshair)
  double tync;             // Y offset (no crosshair)
  double ta;               // Target area
  double distToCamera;     // Distance to camera
  double distToRobot;      // Distance to robot
  double ambiguity;        // Tag pose ambiguity
  int closestAprilTagID = 0;  //Tag ID with the greatest area

  private int tagID = 3;
  private double txToTurn = 0;
  private double angleToTurn = 0;

public Robot() {
  //CameraServer.startAutomaticCapture();
}

  @Override
  public void robotInit() {
    m_AutoChooser.setDefaultOption("None", Constants.AutoConstants.kAutoProgram[0]);
    m_AutoChooser.addOption("Auto 1", Constants.AutoConstants.kAutoProgram[1]);
    m_AutoChooser.addOption("Auto 2", Constants.AutoConstants.kAutoProgram[2]);
    m_AutoChooser.addOption("Auto 3", Constants.AutoConstants.kAutoProgram[3]);
    SmartDashboard.putData("Auto Choices", m_AutoChooser);  //Sync the Autochooser

    m_AprilTagSelected.setDefaultOption("None", "None");
    m_AprilTagSelected.addOption("1", "1");
    m_AprilTagSelected.addOption("2","2");
    m_AprilTagSelected.addOption("3","3");
  
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    SmartDashboard.putData("Auto Choices", m_AutoChooser);  //Sync the Autochooser
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

    //AprilTagSelection
    SmartDashboard.putData("April Tag Target", m_AprilTagSelected);  //Sync the Autochooser
    
    publisher.set(m_swerve.m_odometry.getPoseMeters());
    


    fiducials = LimelightHelpers.getRawFiducials("");
    closestAprilTagID = 0;  //reset the closest tag ID each time
    double closestAprilTagArea = 0;
    for (RawFiducial fiducial : fiducials) {
      /*if (!aprilTags.containsKey(fiducial.id))
      {
        aprilTags.put(fiducial.id, fiducial);
      }else{
        aprilTags.replace(fiducial.id, fiducial);
      }*/
        id = fiducial.id;                    // Tag ID
        txnc = fiducial.txnc;             // X offset (no crosshair)
        tync = fiducial.tync;             // Y offset (no crosshair)
        ta = fiducial.ta;                 // Target area
        distToCamera = fiducial.distToCamera;  // Distance to camera
        distToRobot = fiducial.distToRobot;    // Distance to robot
        ambiguity = fiducial.ambiguity;   // Tag pose ambiguity
      if(ta > closestAprilTagArea) {
        closestAprilTagArea = ta;
        closestAprilTagID = id;
      }
    }

    SmartDashboard.putNumber("distToCamera", distToCamera);
    SmartDashboard.putNumber("Txnc", txnc);
    SmartDashboard.putNumber("closestAprilTag", closestAprilTagID);

    
  }

  

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    SmartDashboard.putData("Field", m_field);

    m_autoSelected = m_AutoChooser.getSelected();
    m_autonomousCommand = getAutonomousCommand();
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }    
  }

  @Override
  public void teleopInit() {
    // Do this in either robot or subsystem init
    SmartDashboard.putData("Field", m_field);
    publisher.set(m_swerve.m_odometry.getPoseMeters());
    
    // This makes sure that the autonomous stops running when teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    /* Button Triggers */
    backButton.onTrue(shiftGears()); 
    startButton.onTrue(changeIsFieldRelative());
    
    aButton.onTrue(turnTowardAprilTag(3)); 
    //aButton.onTrue(turnTorwardAprilTag(m_AprilTagSelected.getSelected()));   //turn toward the closest AprilTag 
  //turn toward the closest AprilTag 
    bButton.onTrue(new driveSpinwaysPID(0, getPeriod(), m_swerve));
  //bButton.onTrue(new setCranePosition(Constants.Position.keProcessor, m_AlgaeGrabber));
    //yButton.onTrue(new setCranePosition(Constants.Position.keReef3, m_AlgaeGrabber));
    //xButton.onTrue(new setCranePosition(Constants.Position.keReef2, m_AlgaeGrabber));
    
    //lBTrigger.whileTrue(new setClawSpeed(0.5, m_AlgaeGrabber))
    //          .onFalse(new setClawSpeed(0, m_AlgaeGrabber));
    //rBTrigger.whileTrue(new setClawSpeed(-m_controller.getRightTriggerAxis(), m_AlgaeGrabber))
    //          .onFalse(new setClawSpeed(0, m_AlgaeGrabber));    //check - out is negative
    //lbButton.whileTrue(new setClawSpeed(0.5, m_AlgaeGrabber))
    //          .onFalse(new setClawSpeed(0, m_AlgaeGrabber));
    //rbButton.whileTrue(new setClawSpeed(-0.5, m_AlgaeGrabber))
    //          .onFalse(new setClawSpeed(0, m_AlgaeGrabber));
    //rbButton.onTrue(changeIsAlgaeRelative());

    //povUp.onTrue(new InstantCommand(() -> m_AlgaeGrabber.IncCraneAngle()));
    //povDown.onTrue(new InstantCommand(() -> m_AlgaeGrabber.DecCraneAngle()));
    //povRight.onTrue(new InstantCommand(() -> m_AlgaeGrabber.IncWristAngle()));
    //povLeft.onTrue(new InstantCommand(() -> m_AlgaeGrabber.DecWristAngle()));
    
   
  }
  
  @Override
  public void autonomousPeriodic() {
    //driveWithJoystick(false);
    publishToDashboard();
    m_swerve.publishToDashboard();
    m_swerve.updateOdometry();

    // Do this in either robot periodic or subsystem periodic
    m_field.setRobotPose(m_swerve.m_odometry.getPoseMeters());
  }

  @Override
  public void teleopPeriodic() {
    publishToDashboard();
    //m_swerve.publishToDashboard();
    //switchGears(false);
    m_swerve.updateOdometry();

    // Do this in either robot periodic or subsystem periodic
    m_field.setRobotPose(m_swerve.m_odometry.getPoseMeters());
 
    //Set the max speed constant to use high (regular) or low speed based on isHighGear
    double l_MaxSpeed = isHighGear?Constants.kMaxRobotSpeed:Constants.kMaxRobotSpeedLowGear;
    double l_MaxAngSpeed = isHighGear?Constants.kMaxRobotAngularSpeed:Constants.kMaxRobotAngularSpeedLowGear;
    
    
    final var xSpeed =
      -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.1))
      * l_MaxSpeed;
    SmartDashboard.putNumber("xSpeed", xSpeed);

    final var ySpeed =
      -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.1))
        * l_MaxSpeed;
    SmartDashboard.putNumber("ySpeed", ySpeed);

    final var rot =
      -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.1))
        * l_MaxAngSpeed;
    SmartDashboard.putNumber("rot", rot);

    m_swerve.drive(xSpeed, ySpeed, rot, isFieldRelative, getPeriod()); 
  

    if (tagID != 0){
      System.out.print(tagID);
      System.out.print("Something");
      for(RawFiducial fiducial : fiducials)
        if (fiducial.id == tagID) {
          txToTurn = fiducial.txnc;
        } else {
          txToTurn = 0;
        }
      angleToTurn = -(txToTurn)*(Math.PI/180); 
      SmartDashboard.putNumber("TXTesting", txToTurn);
      SmartDashboard.putNumber("angleToTurn", angleToTurn);
      }
  }

  


  public void publishToDashboard()
  {
    SmartDashboard.putNumber("Controller Left X", m_controller.getLeftX());
    SmartDashboard.putNumber("Controller Left Y", m_controller.getLeftY());
    SmartDashboard.putNumber("Controller Right X", m_controller.getRightX());
    SmartDashboard.putNumber("Gyro Angle", m_swerve.m_gyro.getRotation2d().getDegrees());
    SmartDashboard.putBoolean("High Gear Enabled", isHighGear);
    SmartDashboard.putBoolean("isFieldRelative", isFieldRelative);

    SmartDashboard.putNumber("RightTrigger", m_controller.getRightTriggerAxis());    
  }


  /* AUTO Stuff below here */
  public Command getAutonomousCommand() {

      Command temp = new Command() {};
    // Grabs the choser Auto from Shuffleboard
    switch (m_autoSelected) {
      case "None":
      temp = m_swerve.getPathPlannerCommand();
        break;
      case "Auto 1":
        temp = Commands.sequence(
          new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
          driveStraight(1));
        break;
      case "Auto 2":
        temp = Commands.sequence(
          new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
          new InstantCommand(() -> System.out.println("Command 1:")),
          driveStraight(1),
          new InstantCommand(() -> System.out.println("Stop & wait  .5 seconds")),
          new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(.5),
          new InstantCommand(() -> System.out.println("Done !")));
        break;
      case "Auto 3":
        temp = Commands.sequence(
        new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
        new InstantCommand(() -> System.out.println("Command 1:")),
        driveToPosition(new Pose2d(0, 0, new Rotation2d(Math.PI/2))),
        new InstantCommand(() -> System.out.println("Stop & wait  .5 seconds")),
        new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(.5),
        new InstantCommand(() -> System.out.println("Done !")));
        break;
      
      default:
        break;
    }
    return temp;
  }
 
  public InstantCommand resetOdoCommand() {
    return new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0))));
  }
  public Command driveStraight(double dist) {
    return new driveStraightPID(dist, getPeriod(), m_swerve);
  }
    
  public Command driveSideways(double dist) {
    return new driveSidewaysPID(dist, getPeriod(), m_swerve);
  }

  public Command driveSpinways(double angle) {
    return new driveSpinwaysPID(angle, getPeriod(), m_swerve);
  }

  public Command driveToPosition(Pose2d position) {
    return new driveToPositionPID(position, getPeriod(), m_swerve);
  }

    public Command shiftGears() {
    return Commands.sequence(
        new InstantCommand(() -> isHighGear=!isHighGear)
    );
  }

  public Command changeIsFieldRelative() {
    return Commands.sequence(
        new InstantCommand(() -> isFieldRelative=!isFieldRelative)
    );
  }

  public Command turnTowardAprilTag(int tagID) {
    //double angle = 0;
    return Commands.sequence(
      //new InstantCommand(() -> ang = angleToTurn),  //getAprilTx(3)
      new turnTowardsAprilPID(angleToTurn, getPeriod(), m_swerve)
    );
  }

  /*public double getAprilTx (int tagID) {
     System.out.print(tagID);
        System.out.print("Something");
            double txToTurn = 0;

    if (tagID != 0){
      
      for(RawFiducial fiducial : fiducials)
        if (fiducial.id == tagID) {
          txToTurn = fiducial.txnc;
        } else {
          txToTurn = 0;
        }
      }
      double angleToTurn = -(txToTurn)*(Math.PI/180); 
      SmartDashboard.putNumber("TXTesting", txToTurn);
      return angleToTurn;
  }*/


/**
 * diagScoreAuto starts in front of the cage closest to the wall facing towards the driver stations
 * Drives towards the reef, scores on Reef 1, then backs up
 * @param isOnBlueSide Set to TRUE if the Robot starts on the BlueSide (vs. false for RedSide)
 * @return
 */
public Command diagScoreAuto(boolean isOnBlueSide) {
  return Commands.sequence(
    new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
    new InstantCommand(() -> System.out.println("Drive Forward")),
    driveStraight(1.25), 
    new InstantCommand(() -> System.out.println("Turn 60 Degrees CCW")),
    driveSpinways(Math.PI/3),
    new InstantCommand(() -> System.out.println("Drive Forward to Reef")),
    driveStraight(3.3), //calculate the diagnoal distance to the reef
    new InstantCommand(() -> System.out.println("Set Coral Height - Reef 1")),
    //setCoralHeight(Constants.Position.keReef1),
    new InstantCommand(() -> System.out.println("Set Coral Tilt - Score")),
    //scoreCoralReef1(),
    new InstantCommand(() -> System.out.println("Wait")),
    new WaitCommand(3.5),
    new InstantCommand(() -> System.out.println("Set Coral Height - Stow")),
    //stowCoral(),
    new InstantCommand(() -> System.out.println("Stop & wait  .5 seconds")),
    new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(.5),
    new InstantCommand(() -> System.out.println("Done !")));
}
 

}