// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Turrent;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final CommandXboxController driver = new CommandXboxController(0);
    final CommandXboxController gunner = new CommandXboxController(1);
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));

    Shooter shooter = new Shooter();
    Turrent turrent = new Turrent();
    Intake intake = new Intake();
    limelight scope = new limelight();
    Climber climber = new Climber();
    // Establish a Sendable Chooser that will be able to be sent to the
    // SmartDashboard, allowing selection of desired auto
    private final SendableChooser<Command> autoChooser;

    public boolean aimAtAliance = false;
    public boolean aimAtHub = false;
    public boolean manualTower = false;
    public boolean manualShuttel = false;
    public boolean aimAtTarget = true;

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> driver.getLeftY() * -1,
            () -> driver.getLeftX() * -1)
            .withControllerRotationAxis(driver::getRightX)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative
     * input stream.
     */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driver::getRightX,
            driver::getRightY)
            .headingWhile(true);

    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative
     * input stream.
     */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
            .allianceRelativeControl(false);

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX())
            .withControllerRotationAxis(() -> driver.getRawAxis(
                    2))
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
            .withControllerHeadingAxis(() -> Math.sin(
                    driver.getRawAxis(2) *Math.PI)*(Math.PI *2),
                    () -> Math.cos(driver.getRawAxis(2) *Math.PI)*(Math.PI * 2))
            .headingWhile(true)
            .translationHeadingOffset(true)
            .translationHeadingOffset(Rotation2d.fromDegrees(0));

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        DriverStation.silenceJoystickConnectionWarning(true);

        // Create the NamedCommands that will be used in PathPlanner
        NamedCommands.registerCommand("test", Commands.print("I EXIST"));

        // Have the autoChooser pull in all PathPlanner autos as options
        autoChooser = AutoBuilder.buildAutoChooser();

        // Set the default auto (do nothing)
        autoChooser.setDefaultOption("Do Nothing", Commands.none());

        // Add a simple auto option to have the robot drive forward for 1 second then
        // stop
        autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

        // Put the autoChooser on the SmartDashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary predicate, or via the
     * named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
     * for
     * {@link CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
     * Flight joysticks}.
     */
    private void configureBindings() {
        intake.setDefaultCommand(intake.Hold());

        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
        Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
                driveDirectAngle);
        Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
        Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
        Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
                driveDirectAngleKeyboard);

      

        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        // turrent.setDefaultCommand(turrent.AimAtYaw(()->scope.getTx(),
        // ()->!intake.isUp()&&climber.isRetracted())); // IS up is inverted since we
        // can not move with intake up
        // driver.leftBumper().whileTrue(Commands.runOnce(drivebase::lock,
        // drivebase).repeatedly());

        driver.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
 
        driver.rightBumper().whileTrue(intake.DeployCommand().andThen(intake.intakeCommand()));
        driver.leftBumper().and(gunner.a().negate()).whileTrue(turrent.Stow().andThen(intake.RetractCommand()));

     
        //driver.y().whileTrue(shooter.tuningCommand());

        driver.rightTrigger(0.2).and(() -> aimAtTarget)
                .whileTrue(shooter
                        .distanceShot(() -> scope.getDistance(), () -> scope.getTv(), gunner.rightBumper(),
                                gunner.leftBumper())
                        .alongWith(turrent.AimAtYaw(() -> scope.getTx(), () -> intake.isUp() && climber.isRetracted(),
                                gunner.rightTrigger(0.2), gunner.leftTrigger(0.2))));
        driver.rightTrigger(0.2).and(() -> manualTower).whileTrue(shooter.manualTower().alongWith(turrent.Stow()));
        driver.leftTrigger(0.2).whileTrue(turrent.Stow().andThen(intake.RetractCommand()).andThen(climber.manual(() -> gunner.getLeftY(), () -> gunner.getRightY())));

        gunner.a().onTrue(Commands.runOnce(() -> {
            aimAtAliance = false;
            aimAtHub = false;
            manualTower = false;
            manualShuttel = false;
            aimAtTarget = true;

        }));
        gunner.b().onTrue(Commands.runOnce(() -> {
            aimAtAliance = false;
            aimAtHub = false;
            manualTower = true;
            manualShuttel = false;
            aimAtTarget = false;

        }));
        // gunner.leftBumper().and(driver.b().negate()).and(driver.a().negate()).whileTrue(turrent.Stow().andThen(intake.RetractCommand()).andThen(climber.DepolyCommand()));
        // gunner.rightBumper().and(driver.b().negate()).and(driver.a().negate()).whileTrue(turrent.Stow().andThen(climber.RetractCommand()));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Pass in the selected auto from the SmartDashboard as our desired autnomous
        // commmand
        return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
