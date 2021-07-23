package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpiutil.net.PortForwarder;
import frc.modeling.FieldPositions;
import frc.robot.commands.DriverControl;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.util.Geometry;

public class Robot extends TimedRobot {
    // Declare named commands for use later.
    private Command m_autonomousCommand;

    // Declare Controllers for Robot
    private XboxController driver;
    private XboxController operator;

    // Declare Subsystems
    public static Climber climber;
    private Drivetrain drivetrain;
    private Intake intake;
    public static Spindexer spindexer;
    private Shooter shooter;

    // Declare Pre-Allocated Buttons on Controllers
    private Button shooterButton;
    private Button unjamButton;
    private Button intakeButton;
    private Button coolMotorsButton, liftIntakeButton;
    private Button extendRetractClimberButton;

    // Declare some multi-use variables for Robot.java functions
    private boolean POVDebounce = false;



    // Activate Autonomous Chooser
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /*
        Declare NetworkTableEntry variables
        (to change the values assosciated with Widgets)
    */

    /*
        Robot Constructor. Use this function to initialize subsystems,
        attatch buttons to commands, set default commands, and any other
        function that needs to happen once, on Robot Startup.
    */
    @Override
    public void robotInit() {
        // Declare Subsystems
        drivetrain = new Drivetrain();
        shooter = new Shooter(drivetrain.model.center);
        intake = new Intake();
        spindexer = new Spindexer();
        climber = new Climber();

        driver = new XboxController(Constants.IO.DRIVER_CONTROLLER);
        operator = new XboxController(Constants.IO.OPERATOR_CONTROLLER);

        // Critical Function Buttons
        intakeButton = new Button(() -> (driver.getTriggerAxis(Hand.kLeft) > 0.1));
        shooterButton = new Button(() -> operator.getXButton());

        // Buttons for Non-Critical Functions
        unjamButton = new Button(() -> operator.getStartButton());
        liftIntakeButton = new Button(() -> driver.getBackButton());
        coolMotorsButton = new Button(() -> driver.getStartButton());
        extendRetractClimberButton = new Button(() -> driver.getYButton());

        // Set up Port Forwarding so we can access Limelight over USB tether to robot.
        PortForwarder.add(5800, "limelight.local", 5800);
        PortForwarder.add(5801, "limelight.local", 5801);
        PortForwarder.add(5805, "limelight.local", 5805);

        // Set Default Commands
        drivetrain.setDefaultCommand(
                new DriverControl(drivetrain, () -> driver.getY(Hand.kLeft), () -> driver.getX(Hand.kRight)));
        // shooter.setDefaultCommand(new RunCommand(() -> shooter.disable(), shooter));

        // Critical Function Button Definitions
        intakeButton.whenPressed(new IntakeCommand(intake, spindexer).withInterrupt(() -> !intakeButton.get()));
        shooterButton.whenPressed(new ShootCommand(shooter, spindexer, () -> shooterButton.get(), operator, driver));
        extendRetractClimberButton.whenPressed(new ConditionalCommand(
                                                new InstantCommand(() -> climber.RetractClimber(), climber),
                                                new InstantCommand(() -> climber.ExtendClimber(), climber),
                                                climber::isClimberExtended));


        // Non-Critical Function Button Defintions
        unjamButton.whenPressed(new ConditionalCommand(
                                    new InstantCommand(() -> spindexer.clearedJam()),
                                    new InstantCommand(() -> spindexer.startJamClear()),
                                    spindexer::clearingJam));

        liftIntakeButton.whenPressed(new InstantCommand(() -> intake.foldIntake()));
        coolMotorsButton.whenPressed(new InstantCommand(() -> drivetrain.coolFalcons()));

        // Put the chooser on the dashboard
        SmartDashboard.putData(m_chooser);
        LiveWindow.disableAllTelemetry();
    }


    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // mainPressure.setDouble(250 * (transducer.getVoltage() / 5) - 25);
    }


    @Override
    public void disabledInit() {
        drivetrain.disable();
        shooter.disable();
    }


    @Override
    public void disabledPeriodic() {
    } // Limelight Output small


    @Override
    public void autonomousInit() {
        drivetrain.zeroSensors();
        // shooter.zeroSensors();

        // m_autonomousCommand = new OurTrench(drivetrain, intake, shooter, spindexer);

        // ** Pull the autonomous command choice from the Sendable Chooser **
        m_autonomousCommand = m_chooser.getSelected();

        // schedule the autonomous command selected
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }


    @Override
    public void autonomousPeriodic() {
    }


    @Override
    public void teleopInit() {
        // stop running autonomous command
        if (m_autonomousCommand != null) m_autonomousCommand.cancel();
        drivetrain.setShiftMode(false);
        shooter.enable();
    }


    @Override
    public void teleopPeriodic() {

            /* ********* Drivetrain ********* */

        // If driver is holding the left bumper, shift into high gear. Else, stay in low.
        if (driver.getBumperPressed(Hand.kLeft))
            drivetrain.setShiftMode(true);
        else if (driver.getBumperReleased(Hand.kLeft))
            drivetrain.setShiftMode(false);

        if (driver.getStickButton(Hand.kLeft) && driver.getStickButton(Hand.kRight)) {
            drivetrain.zeroSensors();
        }

            /* ********* Climber ********* */
        if (driver.getAButton() && driver.getBButton()){
            climber.EngagePTO();
        }
        else climber.DisengagePTO();

            /* ********* Shooter ********* */

        // Manual Hood Control Logic
        if (operator.getPOV(0) == -1) POVDebounce = false; // If the D-Pad isn't touched, reset.
        else if (POVDebounce == false){ // If the D-Pad is engaged and it wasn't engaged last cycle...
            POVDebounce = true; // set the flag to prevent multi-press.
            if (operator.getPOV(0) == 0) shooter.trimHood(5.0); // If Up...+5 degrees
            else if (operator.getPOV(0) == 90) shooter.trimHood(2.5); // if Right, +2.5 Degrees
            else if (operator.getPOV(0) == 180) shooter.trimHood(-5.0); // if Down, -5 Degrees
            else if (operator.getPOV(0) == 270) shooter.trimHood(-2.5); // if Left, -2.5 Degrees
            else POVDebounce = false; // If we're hitting a combo of D-Pad buttons, reset.
        }

        // Manual Turret Control
        // TODO This is probably better once the limiting is set up properly (check)
        double angle = Math.toDegrees(Math.atan2(operator.getRawAxis(0), -operator.getRawAxis(1)));
        double mag = Geometry.distance(0, operator.getRawAxis(1), 0, operator.getRawAxis(0));
        if (mag > 0.8) {
            shooter.manualTurret(angle);
        }

        // Override
        if (operator.getBackButtonPressed()) shooter.toggleVisionOverride();
    }


    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        drivetrain.model.setPosition(FieldPositions.RIGHT.START);
        // drivetrain.runTests();
        shooter.enable();
    }


    @Override
    public void testPeriodic() {
    }
}
