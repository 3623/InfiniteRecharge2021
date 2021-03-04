package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.AnalogInput;
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
import frc.robot.commands.AssistedTrenchDrive;
import frc.robot.commands.BarrelAuto;
import frc.robot.commands.BounceAuto;
import frc.robot.commands.DriverControl;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OurTrench;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.util.Geometry;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private XboxController driver;
    private XboxController operator;
    private Button intakeButton;
    private Button shooterButton;
    private Button trenchDriveButton;
    private Button indexEngageButton;
    private Button feedEngageButton;
    // private Climber climber;
    private Drivetrain drivetrain;
    private Intake intake;
    public static Spindexer spindexer;
    private Feeder feed;
    private Shooter shooter;

    AnalogInput transducer = new AnalogInput(0);

    @Override
    public void robotInit() {
        driver = new XboxController(Constants.IO.DRIVER_CONTROLLER);
        operator = new XboxController(Constants.IO.OPERATOR_CONTROLLER);
        intakeButton = new Button(() -> (driver.getTriggerAxis(Hand.kLeft) > 0.1));
        trenchDriveButton = new Button(() -> (driver.getTriggerAxis(Hand.kRight) > 0.1));
        shooterButton = new Button(() -> operator.getXButton());
        feedEngageButton = new Button(() -> operator.getAButton());
        indexEngageButton = new Button(() -> operator.getYButton());
        drivetrain = new Drivetrain();
        shooter = new Shooter(drivetrain.model.center);
        intake = new Intake();
        spindexer = new Spindexer();
        feed = new Feeder();
        // climber = new Climber();

        // Set up Port Forwarding so we can access Limelight over USB tether to robot.
        PortForwarder.add(5800, "limelight.local", 5800);
        PortForwarder.add(5801, "limelight.local", 5801);
        PortForwarder.add(5805, "limelight.local", 5805);

        drivetrain.setDefaultCommand(
                new DriverControl(drivetrain, () -> driver.getY(Hand.kLeft), () -> driver.getX(Hand.kRight)));
        // feed.setDefaultCommand(new RunCommand(() -> feed.stop(), feed));
        // spindexer.setDefaultCommand(new RunCommand(() -> spindexer.stopSpinning(), spindexer));
        shooter.setDefaultCommand(new RunCommand(() -> shooter.disable(), shooter));

        intakeButton.whenPressed(new IntakeCommand(intake, spindexer).withInterrupt(() -> !intakeButton.get()));
        shooterButton.whenPressed(new ShootCommand(shooter, spindexer));
        trenchDriveButton
                .whileActiveOnce(new AssistedTrenchDrive(drivetrain, () -> driver.getTriggerAxis(Hand.kRight)));
        // TODO Remove below testing Commands 
        indexEngageButton.toggleWhenPressed(new RunCommand(() -> spindexer.setShooting(true),spindexer));
        feedEngageButton.toggleWhenPressed(new RunCommand(() -> feed.run(0.9),feed));   
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
        m_autonomousCommand = new BounceAuto(drivetrain);

        // schedule the autonomous command (example)
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
        drivetrain.setShiftMode(true);
        shooter.enable();
        shooter.setDistance(0.0);

    }


    @Override
    public void teleopPeriodic() {
        if (driver.getBumperPressed(Hand.kRight)) {
            drivetrain.setShiftMode(false);
        } else if (driver.getBumperPressed(Hand.kLeft)) {
            drivetrain.setShiftMode(true);
        }

        if (driver.getStartButtonPressed()) {
            drivetrain.zeroSensors();
        }

        // double angle = Math.toDegrees(Math.atan2(operator.getRawAxis(0), -operator.getRawAxis(1)));
        // double mag = Geometry.distance(0, operator.getRawAxis(1), 0, operator.getRawAxis(0));
        // if (mag > 0.8) { 
        //     shooter.setAngle(angle);
        //     System.out.println(angle + " " + mag);
        // }
    }


    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        drivetrain.model.setPosition(FieldPositions.RIGHT.START);
        // drivetrain.runTests();
        // shooter.enable();
    }


    @Override
    public void testPeriodic() {
    }
}
