package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
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
import frc.robot.commands.AccuracyShootCommand;
import frc.robot.commands.AssistedTrenchDrive;
import frc.robot.commands.BarrelAuto;
import frc.robot.commands.BounceAuto;
import frc.robot.commands.DriverControl;
import frc.robot.commands.GalacticSearchABLUE;
import frc.robot.commands.GalacticSearchARED;
import frc.robot.commands.GalacticSearchBBLUE;
import frc.robot.commands.GalacticSearchBRED;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OurTrench;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SlalomAuto;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private Command barrel, slalom, bounce;
    private Command A_RED, A_BLUE, B_RED, B_BLUE;

    private XboxController driver;
    private XboxController operator;
    // private Button shooterButton; // Disabled for Accuracy Challenge
    // private Button trenchDriveButton; // Disabled for Challenges
    // private Climber climber;
    private Drivetrain drivetrain;
    private Intake intake;
    public static Spindexer spindexer;
    private Shooter shooter;

    private boolean POVDebounce = false;

    AnalogInput transducer = new AnalogInput(1);

    // private Button indexButton;
    // private Button readyButton;
    // private Button shootButton;
    // private Button stopSpinningButton;
    private Button unjamButton;
    private Button intakeButton;
    private Button coolMotorsButton, liftIntakeButton;

    private Button greenZone, blueZone, redZone, yellowZone;
    public static char zoneConfirm = 'z';

    private ShuffleboardTab AccuracyTab = Shuffleboard.getTab("Accuracy");
    private SimpleWidget ZoneWidget = AccuracyTab.add("Zone", false)
                                                .withWidget(BuiltInWidgets.kBooleanBox)
                                                .withProperties(Map.of("colorWhenFalse", "black"));
    private NetworkTableEntry AccuracyZone = ZoneWidget.getEntry();

    // Activate Autonomous Chooser
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    @Override
    public void robotInit() {
        // Declare Subsystems
        drivetrain = new Drivetrain();
        shooter = new Shooter(drivetrain.model.center);
        intake = new Intake();
        spindexer = new Spindexer();
        // climber = new Climber();

        // Declare Critical Function Buttons
        driver = new XboxController(Constants.IO.DRIVER_CONTROLLER);
        operator = new XboxController(Constants.IO.OPERATOR_CONTROLLER);
        intakeButton = new Button(() -> (driver.getTriggerAxis(Hand.kLeft) > 0.1));
        //trenchDriveButton = new Button(() -> (driver.getTriggerAxis(Hand.kRight) > 0.1));
        // shooterButton = new Button(() -> operator.getXButton());

        // Accuracy Challenge Shooter Buttons
        greenZone = new Button(() -> operator.getAButton());
        redZone = new Button(() -> operator.getBButton());
        blueZone = new Button(() -> operator.getXButton());
        yellowZone = new Button(() -> operator.getYButton());

        // Buttons for Non-Critical Functions
        unjamButton = new Button(() -> operator.getStartButton());
        liftIntakeButton = new Button(() -> driver.getBackButton());
        coolMotorsButton = new Button(() -> driver.getStartButton());
        
        // Set up Port Forwarding so we can access Limelight over USB tether to robot.
        PortForwarder.add(5800, "limelight.local", 5800);
        PortForwarder.add(5801, "limelight.local", 5801);
        PortForwarder.add(5805, "limelight.local", 5805);

        // Set Default Commands
        drivetrain.setDefaultCommand(
                new DriverControl(drivetrain, () -> driver.getY(Hand.kLeft), () -> driver.getX(Hand.kRight)));
        shooter.setDefaultCommand(new RunCommand(() -> shooter.disable(), shooter));

        // Critical Function Button Defitions
        intakeButton.whenPressed(new IntakeCommand(intake, spindexer).withInterrupt(() -> !intakeButton.get()));
        //shooterButton.whenPressed(new ShootCommand(shooter, spindexer)); // Disabled for Accuracy Challenge
        
        //trenchDriveButton // Disabled for Challenges
        //        .whileActiveOnce(new AssistedTrenchDrive(drivetrain, () -> driver.getTriggerAxis(Hand.kRight)));
        
        
        // Accuracy Challenge Button Definitions
        greenZone.whenPressed(new ConditionalCommand(
                                new AccuracyShootCommand(shooter, spindexer, zoneConfirm),
                                new InstantCommand(() -> Robot.setZone('g')), 
                                Robot::isGreenZone));

        yellowZone.whenPressed(new ConditionalCommand(
                                new AccuracyShootCommand(shooter, spindexer, zoneConfirm),
                                new InstantCommand(() -> Robot.setZone('y')), 
                                Robot::isYellowZone));

        blueZone.whenPressed(new ConditionalCommand(
                                new AccuracyShootCommand(shooter, spindexer, zoneConfirm),
                                new InstantCommand(() -> Robot.setZone('b')), 
                                Robot::isBlueZone));

        redZone.whenPressed(new ConditionalCommand(
                                new AccuracyShootCommand(shooter, spindexer, zoneConfirm),
                                new InstantCommand(() -> Robot.setZone('r')), 
                                Robot::isRedZone));

        // Non-Critical Function Button Defintions
        unjamButton.whenPressed(new ConditionalCommand(
                                    new InstantCommand(() -> spindexer.clearedJam()),
                                    new InstantCommand(() -> spindexer.startJamClear()), 
                                    spindexer::clearingJam));

        liftIntakeButton.whenPressed(new InstantCommand(() -> intake.foldIntake()));
        coolMotorsButton.whenPressed(new InstantCommand(() -> drivetrain.coolFalcons()));

        barrel = new BarrelAuto(drivetrain);
        slalom = new SlalomAuto(drivetrain);
        bounce = new BounceAuto(drivetrain);

        A_RED = new GalacticSearchARED(drivetrain, intake);
        A_BLUE = new GalacticSearchABLUE(drivetrain, intake);
        B_RED = new GalacticSearchBRED(drivetrain, intake);
        B_BLUE = new GalacticSearchBBLUE(drivetrain, intake);

        // Add commands to the autonomous command chooser 
        // AutoNav Challenge autos
        m_chooser.setDefaultOption("Barrel Course", barrel);
        m_chooser.addOption("Slalom Course", slalom);
        m_chooser.addOption("Bounce Course", bounce);

        // Galactic Search (Ball Pickup) Autos FOR TESTING ONLY
        m_chooser.addOption("A RED", A_RED);
        m_chooser.addOption("B RED", B_RED);
        m_chooser.addOption("A BLUE", A_BLUE);
        m_chooser.addOption("B BLUE", B_BLUE);


        // Put the chooser on the dashboard
        SmartDashboard.putData(m_chooser);
    }

    private void zoneColorIndicator(){
        switch (zoneConfirm){
            case 'g':
                ZoneWidget.withProperties(Map.of("colorWhenTrue", "green"));
                break;
            case 'y':
                ZoneWidget.withProperties(Map.of("colorWhenTrue", "yellow"));
                break;
            case 'b':
                ZoneWidget.withProperties(Map.of("colorWhenTrue", "blue"));
                break;
            case 'r':
                ZoneWidget.withProperties(Map.of("colorWhenTrue", "red"));
                break;
        }
        if (!(zoneConfirm == 'z')) AccuracyZone.setBoolean(true);
        else AccuracyZone.setBoolean(false);

    }

    public static void setZone(char zone) {
        zoneConfirm = zone;
    }

    public static char getZone(){
        return zoneConfirm;
    }

    public static boolean isGreenZone(){
        return zoneConfirm=='g';
    }

    public static boolean isBlueZone(){
        return zoneConfirm=='b';
    }

    public static boolean isYellowZone(){
        return zoneConfirm=='y';
    }

    public static boolean isRedZone(){
        return zoneConfirm=='r';
    }


    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        zoneColorIndicator();
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
        m_autonomousCommand = m_chooser.getSelected();

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

        if (operator.getPOV(0) == -1) POVDebounce = false;
        else if (POVDebounce == false){
            POVDebounce = true;
            if (operator.getPOV(0) == 0) shooter.modifyHoodSet(5.0);
            else if (operator.getPOV(0) == 90) shooter.modifyHoodSet(2.5);
            else if (operator.getPOV(0) == 180) shooter.modifyHoodSet(-5.0);
            else if (operator.getPOV(0) == 270) shooter.modifyHoodSet(-2.5);
            else POVDebounce = false;
        }

        shooter.moveTurret(-operator.getY(Hand.kLeft));        

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
        shooter.enable();
    }


    @Override
    public void testPeriodic() {
    }
}
