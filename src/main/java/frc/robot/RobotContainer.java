package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Constants.OIConstants;

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;

    // The robot's subsystems
    private final Drivetrain drivetrain = new Drivetrain(); // Drivetrain Subsystem
    // private final Vision vision = new Vision(drivetrain); // Vision Subsystem

    // The driver's controller
    public static final CommandXboxController primary = new CommandXboxController(OIConstants.primaryPort);

    // Speed factor to reduce drive speed (e.g., 50% speed)
    private static final double slowFactor = 0.5;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        registerNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        drivetrain.setDefaultCommand(
            new RunCommand(
                () -> {
                    // Get the controller inputs
                    double ySpeed = -MathUtil.applyDeadband(primary.getLeftY(), OIConstants.driveDeadband);
                    double xSpeed = -MathUtil.applyDeadband(primary.getLeftX(), OIConstants.driveDeadband);
                    double rot = -MathUtil.applyDeadband(primary.getRightX(), OIConstants.driveDeadband);

                    // Apply slow factor if B button is pressed
                    if (primary.b().getAsBoolean()) {  // Fixed: Use b().getAsBoolean() instead of getBButton()
                        ySpeed *= slowFactor;
                        xSpeed *= slowFactor;
                        rot *= slowFactor;
                    }

                    drivetrain.drive(ySpeed, xSpeed, rot, true);
                },
                drivetrain));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link Trigger} class.
     */
    private void configureButtonBindings() {
        // Example of how to bind a button to a command using the new command-based framework
        //primary.a()
        //     .whileTrue(new ExampleCommand());
    }

    private void registerNamedCommands() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * This method is called when the autonomous period begins.
     * Start flashing the lights blue here.
     */
    public void autonomousInit() {
        // Start flashing blue during autonomous
    }

    /**
     * This method is called periodically during the autonomous period.
     * Keep calling the flashing method to update the lights.
     */
    public void autonomousPeriodic() {
        // Ensure flashing is updated periodically during autonomous
    }

    /**
     * Stop the flashing or lights when teleop starts.
     */
    public void teleopInit() {
    }
}