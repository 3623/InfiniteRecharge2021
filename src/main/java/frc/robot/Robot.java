package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpiutil.net.PortForwarder;
import frc.modeling.FieldPositions;
import frc.robot.commands.*;
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
    public static XboxController driver;
    public static XboxController operator;

    // Declare Subsystems
    public static Climber climber;
    private Drivetrain drivetrain;
    private Intake intake;
    public static Spindexer spindexer;
    public static Shooter shooter;

    public AnalogInput MainPressure;

    // Declare Pre-Allocated Buttons on Controllers
    private Button shiftButton;
    private Button shooterButton;
    private Button visionOverrideButton;
    private Button fireOverrideButton;
    private Button unjamButton;
    private Button intakeButton, liftIntakeButton;
    private Button climberButton;
    private Button ptoButton;
    private Button coolMotorsButton;
    private Button drivetrainZeroButton;

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
        /* Initialize Subsystems */
        drivetrain = new Drivetrain();
        spindexer = new Spindexer();
        shooter = new Shooter(drivetrain.model.center, spindexer);
        intake = new Intake();
        climber = new Climber();

        MainPressure = new AnalogInput(0);

        driver = new XboxController(Constants.IO.DRIVER_CONTROLLER);
        operator = new XboxController(Constants.IO.OPERATOR_CONTROLLER);

        // Critical Function Buttons
        shiftButton = new Button(() -> driver.getBumper(Hand.kRight));

        intakeButton = new Button(() -> (operator.getAButton() || driver.getTriggerAxis(Hand.kRight) > 0.2));
        liftIntakeButton = new Button(() -> (driver.getTriggerAxis(Hand.kLeft) > 0.5));

        shooterButton = new Button(() -> (operator.getXButton() || driver.getXButton()));
        visionOverrideButton = new Button(() -> operator.getBackButton());
        fireOverrideButton = new Button(() -> (operator.getTriggerAxis(Hand.kRight) > 0.5));

        unjamButton = new Button(() -> operator.getBButton());

        climberButton = new Button(() -> driver.getYButton());
        ptoButton = new Button(() -> (driver.getAButton() && driver.getBButton()));

        // Buttons for Non-Critical Functions
        coolMotorsButton = new Button(() -> driver.getStartButton());
        drivetrainZeroButton = new Button(() -> driver.getBackButton());

        /* Default Commands */
        drivetrain.setDefaultCommand(
                new DriverControl(drivetrain, climber, () -> -driver.getY(Hand.kLeft), () -> driver.getX(Hand.kRight)));
        shiftButton.whileActiveOnce(new StartEndCommand(() -> drivetrain.setShiftMode(true),
                                                        () -> drivetrain.setShiftMode(false)));

        /* Button Definitions */
        intakeButton.whileActiveOnce(new IntakeCommand(intake, spindexer));
        liftIntakeButton.whenPressed(new InstantCommand(() -> intake.foldIntake()));

        shooterButton.whenPressed(new ShootCommand(shooter, spindexer, () -> shooterButton.get(), operator, driver));

        visionOverrideButton.whenPressed(new InstantCommand(() -> shooter.toggleVisionOverride()));
        fireOverrideButton.whileActiveOnce(new StartEndCommand(() -> shooter.setFireOverride(true),
                                                               () -> shooter.setFireOverride(false)));

        climberButton.whenPressed(new ConditionalCommand(
                                                new InstantCommand(() -> climber.RetractClimber(), climber),
                                                new InstantCommand(() -> climber.ExtendClimber(), climber),
                                                climber::isClimberExtended));
        ptoButton.whileActiveOnce(new StartEndCommand(() -> climber.EngagePTO(),
                                                      () -> climber.DisengagePTO()));

        unjamButton.whileActiveOnce(new StartEndCommand(() -> spindexer.toggleJamClear(),
                                                        () -> spindexer.toggleJamClear(),
                                                        spindexer));

        coolMotorsButton.whenPressed(new InstantCommand(() -> drivetrain.toggleCoolFalcons()));

        drivetrainZeroButton.whenPressed(new InstantCommand(() -> drivetrain.zeroSensors()));

        /* Declare Autonomous Command Options */
        final Command m_SimpleShootAuto = new SimpleShoot(drivetrain, intake, shooter, spindexer);
        final Command m_ThierTrenchAuto = new TheirTrench(drivetrain, intake, shooter, spindexer);
        final Command m_BlockTrenchAuto = new BlockTrench(drivetrain, intake, shooter, spindexer);
        final Command m_OurTrenchAuto = new OurTrench(drivetrain, intake, shooter, spindexer);
        final Command m_MiddleAuto = new Middle(drivetrain, intake, shooter, spindexer);
        // Add the Autonomous Commands to the Chooser
        m_chooser.setDefaultOption("Simple Shoot", m_SimpleShootAuto);
        m_chooser.addOption("Their Trench", m_ThierTrenchAuto);
        m_chooser.addOption("Block Trench", m_BlockTrenchAuto);
        m_chooser.addOption("Normal Our Trench", m_OurTrenchAuto);
        m_chooser.addOption("Middle", m_MiddleAuto);
        // Put the chooser on the dashboard
        SmartDashboard.putData(m_chooser);

        // DO NOT DELETE! Without this loops run for too long
        LiveWindow.disableAllTelemetry();

        // Set up Port Forwarding so we can access Limelight over USB tether to robot.
        PortForwarder.add(5800, "limelight.local", 5800);
        PortForwarder.add(5801, "limelight.local", 5801);
        PortForwarder.add(5805, "limelight.local", 5805);

    }


    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Main Pressure", 250 * (MainPressure.getVoltage() / 5) - 25);
    }


    @Override
    public void disabledInit() {
        drivetrain.disable();
        shooter.disable();
        spindexer.stopSpinning();
    }


    @Override
    public void disabledPeriodic() {
    } // Limelight Output small


    @Override
    public void autonomousInit() {
        drivetrain.zeroSensors();
        drivetrain.setShiftMode(false);
        // shooter.zeroSensors();

        // ** Pull the autonomous command choice from the Sendable Chooser **
        m_autonomousCommand = m_chooser.getSelected();

        // schedule the autonomous command selected
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule(false);
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
        intake.setIntaking(true); // Hopefully this will deploy intake but not run rollers
        intake.setIntaking(false);
        spindexer.setStandby();
    }


    @Override
    public void teleopPeriodic() {

        // Manual Hood Control Logic
        if (operator.getPOV(0) == -1) POVDebounce = false; // If the D-Pad isn't touched, reset.
        else if (POVDebounce == false){ // If the D-Pad is engaged and it wasn't engaged last cycle...
            POVDebounce = true; // set the flag to prevent multi-press.
            if (operator.getPOV(0) == 0) shooter.trimHood(1.0); // If Up...+5 degrees
            else if (operator.getPOV(0) == 90) shooter.trimHood(0.5); // if Right, +2.5 Degrees
            else if (operator.getPOV(0) == 180) shooter.trimHood(-1.0); // if Down, -5 Degrees
            else if (operator.getPOV(0) == 270) shooter.trimHood(-0.5); // if Left, -2.5 Degrees
            else POVDebounce = false; // If we're hitting a combo of D-Pad buttons, reset.
        }

        // Manual Turret Control
        double angle = Math.toDegrees(Math.atan2(operator.getX(Hand.kLeft), -operator.getY(Hand.kLeft)));
        double mag = Geometry.distance(0, operator.getY(Hand.kLeft), 0, operator.getX(Hand.kLeft));
        if (mag > 0.8) {
            shooter.manualTurret(angle);
        }
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
        // drivetrain.periodic();
        // drivetrain.setSpeed(1.0, 1.0);
    }
}
