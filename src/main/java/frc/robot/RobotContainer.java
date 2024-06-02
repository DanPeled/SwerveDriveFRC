package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveConfiguration;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer {
        private final SwerveSubsystem drivebase = new SwerveSubsystem(new SwerveConfiguration("neo"));

        final CommandXboxController driverXbox = new CommandXboxController(0);

        public RobotContainer() {
                // Configure the trigger bindings
                configureBindings();

                // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
                // () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                // OperatorConstants.LEFT_Y_DEADBAND),
                // () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                // OperatorConstants.LEFT_X_DEADBAND),
                // () -> driverXbox.getRightX());

                // TODO: check if driveFieldOriented command also works on physical hardware
                Command driveFieldOriented;
                if (Robot.isReal()) {
                        driveFieldOriented = drivebase.driveFieldOrientedCommand(
                                        () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                        OperatorConstants.kLeftYDeadband),
                                        () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                        OperatorConstants.kLeftXDeadband),
                                        () -> MathUtil.applyDeadband(driverXbox.getRawAxis(3),
                                                        OperatorConstants.kLeftXDeadband));
                } else {
                        driveFieldOriented = drivebase.driveFieldOrientedCommand(
                                        () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                        OperatorConstants.kLeftYDeadband),
                                        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(),
                                                        OperatorConstants.kLeftXDeadband),
                                        () -> MathUtil.applyDeadband(-driverXbox.getRawAxis(3),
                                                        OperatorConstants.kLeftXDeadband));
                }

                drivebase.setDefaultCommand(driveFieldOriented);
        }

        private boolean isIdle() {
                return Math.abs(driverXbox.getLeftY()) <= 0.3
                                && Math.abs(driverXbox.getLeftX()) <= 0.3;
        }

        private Command getDriveToCommand() {
                if (isIdle()) {
                        return drivebase.driveToPose(
                                        new Pose2d(new Translation2d(1.9, 7.64),
                                                        Rotation2d.fromDegrees(
                                                                        90)));
                }
                return new Command() {
                        @Override
                        public void initialize() {

                        }
                };
        }

        private void configureBindings() {
                driverXbox.a().whileTrue(new Command() {
                        @Override
                        public void initialize() {
                                drivebase.obstacles.clearObstacles();
                        }
                });

                driverXbox.x().whileTrue(new Command() {
                        @Override
                        public void initialize() {
                                drivebase.obstacles.reset();
                        }
                });
                driverXbox.b().whileTrue(getDriveToCommand());
        }

        public Command getAutonomousCommand() {
                return drivebase.getAutonomousCommand("New Auto");
        }

        public void setDriveMode() {
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }
}
