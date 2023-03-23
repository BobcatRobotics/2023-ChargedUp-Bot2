package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import io.github.oblarg.oblog.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(2);
    private final Joystick rotate = new Joystick(0);
    private final Joystick strafe = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton ruffy0 = new JoystickButton(rotate, 1);
    private final JoystickButton ruffy1 = new JoystickButton(strafe, 1);

    /* Subsystems */
    private static final Limelight m_Limelight = new Limelight();
    private static final Swerve s_Swerve = new Swerve(m_Limelight);

    /* Autonomous */
    private static SwerveAutoBuilder swerveAutoBuilder;
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * Sets Autonomous SendableChooser up on SmartDashboard
     */
    public void putAutoChooserUp() {
        SmartDashboard.putData(autoChooser);
    }

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure Oblog
        Logger.configureLoggingAndConfig(this, false);

        // Configure the button bindings
        configureButtonBindings();

        // Initialize SwerveAutoBuilder
        swerveAutoBuilder = new SwerveAutoBuilder(
            s_Swerve::getPose,
            s_Swerve::resetOdometry,
            Constants.Swerve.swerveKinematics,
            new PIDConstants(Constants.AutoConstants.kPXController, 0, 0),
            new PIDConstants(Constants.AutoConstants.kPThetaController, 0, 0),
            s_Swerve::setModuleStates,
            Constants.AutoConstants.eventMap,
            true,
            s_Swerve
        );
    }

    /**
     * Schedules default commands during teleopInit
     */
    public void scheduleDefaultTeleop() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -strafe.getRawAxis(Joystick.AxisType.kY.value)*Math.abs(strafe.getRawAxis(Joystick.AxisType.kY.value)), 
                () -> -strafe.getRawAxis(Joystick.AxisType.kX.value)*Math.abs(strafe.getRawAxis(Joystick.AxisType.kX.value)), 
                () -> -rotate.getRawAxis(Joystick.AxisType.kX.value), 
                () -> false // always field centric
            )
        );
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        ruffy0.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        ruffy1.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }

    /**
     * 
     * @param trajs - List<PathPlannerTrajectory> from PathPlanner.loadPathGroup()
     * @return command generated from SwerveAutoBuilder
     */
    public static Command buildAuto(List<PathPlannerTrajectory> trajs) {
        return swerveAutoBuilder.fullAuto(trajs);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}
