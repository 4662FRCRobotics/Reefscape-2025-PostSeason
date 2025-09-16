// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;


public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private static final AprilTagFieldLayout m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  private static final Transform3d kRobotToCam = 
    new Transform3d(new Translation3d(0.185, 0.22, 0.3), new Rotation3d(0, 0, 0));
  private PhotonPoseEstimator photonEstimator = 
    new PhotonPoseEstimator(m_fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU(ADIS16470_IMU.IMUAxis.kY, ADIS16470_IMU.IMUAxis.kX, ADIS16470_IMU.IMUAxis.kZ);

  // Odometry class for tracking robot pose
 /*  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });*/

          // Pose estimation class for tracking robot pose
    SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            getHeading(),
            getModulePositions(),
            new Pose2d());
    private final Field2d m_field = new Field2d();

    PhotonCamera m_aprilCameraOne;
    PhotonCamera m_driverCameraTwo;
    boolean m_hasTargets = false;
    PhotonTrackedTarget m_target = new PhotonTrackedTarget();

private BranchSide m_side = BranchSide.MIDDLE;

  public enum BranchSide{
    LEFT(new Translation2d(-0.153209, 0.5406845 + 0.02)),
    RIGHT(new Translation2d(0.2032, 0.5408565 + 0.02)),
    MIDDLE(new Translation2d(0.064853, 0.5408565 + 0.02));
    
    public Translation2d tagOffset;
    private BranchSide(Translation2d offsets) {
      tagOffset = offsets;
    }

    public BranchSide mirror() {
      switch (this) {
        case LEFT: return RIGHT;
        case MIDDLE: return MIDDLE;
        default: return LEFT;
      }
    }
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
   
    RobotConfig config = null;
    try{
        config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
    }
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new PPHolonomicDriveController( // HolonomicPathFollowerConfig
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config,
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );

    SmartDashboard.putData("Field", m_field);

    m_aprilCameraOne = new PhotonCamera(DriveConstants.kCameraOne);
    m_aprilCameraOne.setDriverMode(false);
    m_driverCameraTwo = new PhotonCamera(DriveConstants.kCameraTwo);
    m_driverCameraTwo.setDriverMode(true);

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    /*m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });*/
    double yaw = 0;
    double pitch = 0;
    double area = 0;
    double skew = 0;
    int targetID = 0;
    Optional<Pose3d> target3d;
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    var april1Result = m_aprilCameraOne.getLatestResult();
    m_hasTargets = april1Result.hasTargets();
    if (m_hasTargets) {
      m_target = april1Result.getBestTarget();
      visionEst = photonEstimator.update(april1Result);
      yaw = m_target.getYaw();
      pitch = m_target.getPitch();
      area = m_target.getArea();
      skew = m_target.getSkew();
      targetID = m_target.getFiducialId();
      target3d = m_fieldLayout.getTagPose(targetID);
    /*  if (visionEst.isPresent()) {
        target3d = Optional.of(visionEst.get().estimatedPose);
      }*/
    } else { 
      target3d = Optional.empty();
      visionEst = Optional.empty();
    }
    
    SmartDashboard.putString("Branch side", m_side.name());
    SmartDashboard.putNumber("Yaw" , yaw);
    SmartDashboard.putNumber("Pitch" , pitch);
    SmartDashboard.putNumber("Area" , area);
    SmartDashboard.putNumber("Skew" , skew);
    SmartDashboard.putNumber("Target ID" , targetID);
    SmartDashboard.putNumber("Gyro" , getHeading().getDegrees());
    SmartDashboard.putBoolean("Has Target", m_hasTargets);

    m_poseEstimator.update(
      getHeading(),
      getModulePositions()
      );

    if (!visionEst.isEmpty()) {
      m_poseEstimator.resetPose(visionEst.get().estimatedPose.toPose2d());
    } 
    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
  }

  void setSide(BranchSide side){
    m_side = side;
  }

  public Command cmdSetBranchSide(BranchSide side){
    return Commands.runOnce(()-> setSide(side),this);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
       getHeading(),
       getModulePositions(),
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean halfThrottle) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double throttle = 1.0;

    if (halfThrottle){
      throttle = 0.5;
    }

    xSpeed = squareAxis(xSpeed)*throttle;
    ySpeed = squareAxis(ySpeed)*throttle;
    rot = squareAxis(rot)*throttle;
    
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  private double squareAxis(double axis) {
    return Math.copySign(axis * axis, axis);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

      //@AutoLogOutput(key = "Chassis/ModulePositions")
  private SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
}


  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kY));
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  private void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds, false);
  }

  private void drive(ChassisSpeeds speeds, boolean fieldRelative) {
      if (fieldRelative)
          speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
      //speeds = ChassisSpeeds.discretize(speeds, LoggedRobot.defaultPeriodSecs);
      var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
      setModuleStates(swerveModuleStates);
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
    };
  }

  //-------PATH PLANNER---------

  //auto prebuild path
  public Command getPathStep(String pathName) {

      return new PathPlannerAuto(pathName);
  }

  //returns command to drive path to a specified branch
  public Command driveToBranch (BranchSide side) {
    return Commands.defer(() -> {
    if (m_hasTargets){
       var branch = getBranchFromTag(m_fieldLayout.getTagPose(m_target.getFiducialId()).get().toPose2d(), side);
       //return Commands.print("Has target " + m_target.getFiducialId() + side.name());
       return getPathFromWaypoint(getWaypointFromBranch(branch));
    } else {return Commands.print("No target");}
    }, Set.of(this));
  }

  private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs){
    return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
  }

  private Command getPathFromWaypoint(Pose2d waypoint) {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(getPose().getTranslation(), getPose().getRotation()), waypoint);
    
    PathPlannerPath path = new PathPlannerPath(
      waypoints,
      DriveConstants.kTeleopPathConstraints,
      new IdealStartingState(getVelocityMagnitude(getRobotRelativeSpeeds()), getHeading()),
      new GoalEndState(0.0, waypoint.getRotation()));

    path.preventFlipping = true;

    return (AutoBuilder.followPath(path))
    .finallyDo((interupt) -> {
      if (interupt) {
        drive(new ChassisSpeeds(0, 0, 0), false);
      }
    });
  }

  //returns pathplanner waypoint with direction of travel away from associated reefside
  private Pose2d getWaypointFromBranch(Pose2d branch){
    return new Pose2d(
      branch.getTranslation(),
      branch.getRotation().rotateBy(Rotation2d.k180deg)
    );
  }

  //returns target branch pos/coords
  private static Pose2d getBranchFromTag(Pose2d tag, BranchSide side) {
    var translation = tag.getTranslation().plus(
      new Translation2d(
        side.tagOffset.getY(),
        side.tagOffset.getX()
      ).rotateBy(tag.getRotation())
    );

    return new Pose2d(
      translation.getX(),
      translation.getY(),
      tag.getRotation()
    );
  }



}
