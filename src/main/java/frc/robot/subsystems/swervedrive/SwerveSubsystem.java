package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

import frc.robot.Constants;
import frc.robot.Constants.AutonConstants;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  /**
   * Swerve drive object.
   */
  private final SwerveDrive m_swerveDrive;

  /**
   * Maximum speed of the robot in meters per second, used to limit acceleration.
   */
  public double maximumSpeed = Units.feetToMeters(17); //
                                                       // Linear Speed (ft/sec) =
                                                       // motor speed (RPM) / gear ratio * pi * wheel diameter [inch] /
                                                       // 12 / 60
  public DynamicObstacles obstacles = new DynamicObstacles(new ArrayList<>(
      List.of(new Pair<>(new Translation2d(0, 2.20), new Translation2d(4.47, 6.52)))), this::getTranslation2d);

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(
      File directory) {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      m_swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    m_swerveDrive.setHeadingCorrection(false); // * Should only be used while controlling the robot via angle.

    // Disables cosine compensation for
    // simulations since it causes discrepancies, not seen in real life
    m_swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);

    setupPathPlanner();

    // #region Draw Path
    ArrayList<Pose2d> obstacle = new ArrayList<>();
    obstacles.getObstacles().forEach((o) -> {
      obstacle.add(new Pose2d(o.getFirst().getX(), o.getFirst().getY(), new Rotation2d()));
      obstacle.add(new Pose2d(o.getSecond().getX(), o.getFirst().getY(), new Rotation2d()));
      obstacle.add(new Pose2d(o.getSecond().getX(), o.getSecond().getY(), new Rotation2d()));
      obstacle.add(new Pose2d(o.getFirst().getX(), o.getSecond().getY(), new Rotation2d()));
      obstacle.add(new Pose2d(o.getFirst().getX(), o.getFirst().getY(), new Rotation2d()));
    });

    m_swerveDrive.field.getObject("path")
        .setPoses(obstacle);
    // #endregion
  }

  public void drawTrajectory(Trajectory trajectory) {
    m_swerveDrive.field.getObject("Trajectory").setTrajectory(trajectory);
  }

  public void clearTrajectory() {
    m_swerveDrive.field.getObject("Trajectory").setTrajectory(new Trajectory());
  }

  /**
   * Initialize {@link SwerveDrive} with the configuration provided.
   *
   * @param configuration Swerve configuration.
   */
  public SwerveSubsystem(SwerveConfiguration configuration) {
    this(configuration.getFile());
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    m_swerveDrive = new SwerveDrive(driveCfg, controllerCfg, maximumSpeed);
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner() {
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        this::getRobotVelocity,
        this::setChassisSpeeds,
        new HolonomicPathFollowerConfig(
            AutonConstants.kTranslationPID,
            AutonConstants.kAnglePID,
            4.5,
            m_swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
            new ReplanningConfig()),
        () -> {
          // controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // ! THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
    );

    Pathfinding.setPathfinder(new LocalADStar());
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose) {
    PathConstraints constraints = new PathConstraints(
        m_swerveDrive.getMaximumVelocity(), 4.0,
        m_swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(720));

    Pathfinding.setDynamicObstacles(getObstacles(), getPose().getTranslation());

    Command cmd = AutoBuilder.pathfindToPose(
        pose,
        constraints,
        0.0, // [meters/sec]
        0.0 // [meters]
    ).andThen(() -> {

    });

    return cmd;
  }

  /**
   * Retrieves the list of obstacles on the field.
   * 
   * This method returns an ArrayList of pairs of {@link Translation2d} objects,
   * where each pair represents the start and end points of an obstacle.
   *
   * The obstacles are used by the pathfinding system to avoid collisions and
   * navigate around these obstacles.
   *
   * @return An ArrayList of {@link Pair} objects, where each pair contains two
   *         {@link Translation2d} objects representing the start and end points
   *         of an obstacle.
   */
  public ArrayList<Pair<Translation2d, Translation2d>> getObstacles() {
    return obstacles.getObstacles();
  }

  /**
   * Retrieves the current translation (position) of the robot on the field.
   * 
   * This method returns the translation component of the robot's pose, which
   * includes its position in the X and Y plane, but not its orientation.
   *
   * The pose is obtained from the swerve drive's odometry system, which tracks
   * the robot's position and orientation over time.
   *
   * @return The current {@link Translation2d} object representing the robot's
   *         position on the field.
   */
  public Translation2d getTranslation2d() {
    return getPose().getTranslation();
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother
   *                     controls.
   * @param translationY Translation in the Y direction. Cubed for smoother
   *                     controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    return run(() -> {
      double xInput = smooth(translationX.getAsDouble()); // Smooth controll out
      double yInput = smooth(translationY.getAsDouble()); // Smooth controll out

      driveFieldOriented(m_swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          m_swerveDrive.getOdometryHeading().getRadians(),
          m_swerveDrive.getMaximumVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param rotation     Rotation as a value between [-1, 1] converted to radians.
   * @return Drive command.
   */
  public Command driveFieldOrientedCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier rotation) {
    return run(() -> {
      driveFieldOriented(m_swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
          translationY.getAsDouble(),
          (m_swerveDrive.getOdometryHeading().getDegrees()
              + rotation.getAsDouble() * Constants.OperatorConstants.kTurnSpeed) * Math.PI / 180,
          m_swerveDrive.getOdometryHeading().getRadians(),
          m_swerveDrive.getMaximumVelocity()));
    });
  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, m_swerveDrive, 12),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, m_swerveDrive),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to drive the robot using translative values and heading as angular
   * velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother
   *                         controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother
   *                         controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for
   *                         smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> {
      // TODO check if smoothing by 3 is good or change to 2
      m_swerveDrive.drive(
          new Translation2d(
              smooth(translationX.getAsDouble()) * m_swerveDrive.getMaximumVelocity(),
              smooth(translationY.getAsDouble()) * m_swerveDrive.getMaximumVelocity()),
          smooth(angularRotationX.getAsDouble()) * m_swerveDrive.getMaximumAngularVelocity(),
          true,
          false);
    });
  }

  /**
   * The primary method for controlling the drivebase. Takes a
   * {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly. Can use either open-loop
   * or closed-loop velocity control for
   * the wheel velocities. Also has field- and robot-relative modes, which affect
   * how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear
   *                      velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards
   *                      the bow (front) and positive y is
   *                      torwards port (left). In field-relative mode, positive x
   *                      is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall
   *                      when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.
   *                      Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for
   *                      robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    m_swerveDrive.drive(translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    m_swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    m_swerveDrive.drive(velocity);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return m_swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not
   * need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must
   * be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    m_swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by
   * odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return m_swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    m_swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    m_swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but
   * facing toward 0.
   */
  public void zeroGyro() {
    m_swerveDrive.zeroGyro();
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    m_swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose
   * estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to
   * resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for
   * speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    xInput = smooth(xInput);
    yInput = smooth(yInput);
    return m_swerveDrive.swerveController.getTargetSpeeds(xInput,
        yInput,
        headingX,
        headingY,
        getHeading().getRadians(),
        maximumSpeed);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   * Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    xInput = smooth(xInput);
    yInput = smooth(yInput);
    return m_swerveDrive.swerveController.getTargetSpeeds(xInput,
        yInput,
        angle.getRadians(),
        getHeading().getRadians(),
        maximumSpeed);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return m_swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return m_swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return m_swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return m_swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock() {
    m_swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return m_swerveDrive.getPitch();
  }

  private double smooth(double v) {
    final double smoothFactor = 2;
    return Math.pow(v, smoothFactor) * Math.signum(v);
  }
}
