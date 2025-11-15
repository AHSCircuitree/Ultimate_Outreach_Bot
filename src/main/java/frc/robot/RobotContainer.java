// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants$AutoConstants;
import frc.robot.Constants$DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

public class RobotContainer {
   private final DriveSubsystem m_robotDrive = new DriveSubsystem();
   XboxController m_driverController = new XboxController(0);

   public RobotContainer() {
      this.configureButtonBindings();
      this.m_robotDrive.setDefaultCommand(new RunCommand(() -> {
         this.m_robotDrive.drive(-MathUtil.applyDeadband(this.m_driverController.getLeftY(), 0.05), -MathUtil.applyDeadband(this.m_driverController.getLeftX(), 0.05), -MathUtil.applyDeadband(this.m_driverController.getRightX(), 0.05), true);
      }, new Subsystem[]{this.m_robotDrive}));
   }

   private void configureButtonBindings() {
      (new JoystickButton(this.m_driverController, Button.kR1.value)).whileTrue(new RunCommand(() -> {
         this.m_robotDrive.setX();
      }, new Subsystem[]{this.m_robotDrive}));
   }

   public Command getAutonomousCommand() {
      TrajectoryConfig config = (new TrajectoryConfig(3.0, 3.0)).setKinematics(Constants$DriveConstants.kDriveKinematics);
      Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0.0, 0.0, new Rotation2d(0.0)), List.of(new Translation2d(1.0, 1.0), new Translation2d(2.0, -1.0)), new Pose2d(3.0, 0.0, new Rotation2d(0.0)), config);
      ProfiledPIDController thetaController = new ProfiledPIDController(1.0, 0.0, 0.0, Constants$AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-3.141592653589793, Math.PI);
      DriveSubsystem var10003 = this.m_robotDrive;
      Objects.requireNonNull(var10003);
      Supplier var5 = var10003::getPose;
      SwerveDriveKinematics var10004 = Constants$DriveConstants.kDriveKinematics;
      PIDController var10005 = new PIDController(1.0, 0.0, 0.0);
      PIDController var10006 = new PIDController(1.0, 0.0, 0.0);
      DriveSubsystem var10008 = this.m_robotDrive;
      Objects.requireNonNull(var10008);
      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory, var5, var10004, var10005, var10006, thetaController, var10008::setModuleStates, new Subsystem[]{this.m_robotDrive});
      this.m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
      return swerveControllerCommand.andThen(() -> {
         this.m_robotDrive.drive(0.0, 0.0, 0.0, false);
      }, new Subsystem[0]);
   }
}
