package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;
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
import frc.controls.pixy.Pixy2;
import frc.controls.pixy.Pixy2CCC;
import frc.controls.pixy.Pixy2.LinkType;
import frc.controls.pixy.Pixy2CCC.Block;
import frc.controls.pixy.links.SPILink;
import frc.controls.pixy.links.UARTLink;
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
    // Declare named commands for use later.
    private Command m_autonomousCommand;
    private Command barrel, slalom, bounce;
    private Command A_RED, A_BLUE, B_RED, B_BLUE;

    // Declare Controllers for Robot
    private XboxController driver;
    private XboxController operator;

    // Declare Subsystems
    // private Climber climber;
    private Drivetrain drivetrain;
    private Intake intake;
    public static Spindexer spindexer;
    private Shooter shooter;
    private static Pixy2 ballDetectorPixy;

    // Declare Pre-Allocated Buttons on Controllers
    // private Button shooterButton; // Disabled for Accuracy Challenge
    // private Button trenchDriveButton; // Disabled for Challenges
    // private Button indexButton;
    // private Button readyButton;
    // private Button shootButton;
    // private Button stopSpinningButton;
    private Button unjamButton;
    private Button intakeButton;
    private Button coolMotorsButton, liftIntakeButton;
    private Button greenZone, blueZone, redZone, yellowZone;

    // Declare some multi-use variables for Robot.java functions
    private boolean POVDebounce = false;
    public static char zoneConfirm = 'z';

    

    // Activate Autonomous Chooser
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    // Declare Code Generated Shuffleboard Tabs
    private ShuffleboardTab AccuracyTab = Shuffleboard.getTab("Accuracy");

    // Declare Code Generated/Set Widgets
    private SimpleWidget ZoneWidget = AccuracyTab.add("Zone", false).withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenFalse", "#000000"));

    /* 
        Declare NetworkTableEntry variables 
        (to change the values assosciated with Widgets)
    */
    private NetworkTableEntry AccuracyZone = ZoneWidget.getEntry();

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
        // climber = new Climber();

        ballDetectorPixy = Pixy2.createInstance(new SPILink());
        ballDetectorPixy.init(0);
        ballDetectorPixy.setLamp((byte) 1, (byte) 1); // Turns the LEDs on
		ballDetectorPixy.setLED(255, 255, 255); // Sets the RGB LED to full white

        
        driver = new XboxController(Constants.IO.DRIVER_CONTROLLER);
        operator = new XboxController(Constants.IO.OPERATOR_CONTROLLER);
        
        // Critical Function Buttons
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

        // Critical Function Button Definitions
        intakeButton.whenPressed(new IntakeCommand(intake, spindexer).withInterrupt(() -> !intakeButton.get()));
        //shooterButton.whenPressed(new ShootCommand(shooter, spindexer)); // Disabled for Accuracy Challenge
        
        //trenchDriveButton // Disabled for Challenges
        //        .whileActiveOnce(new AssistedTrenchDrive(drivetrain, () -> driver.getTriggerAxis(Hand.kRight)));
        
        
        // Accuracy Challenge Button Definitions
        greenZone.whenPressed(new ConditionalCommand(
                                new AccuracyShootCommand(shooter, spindexer, 'g'),
                                new InstantCommand(() -> Robot.setZone('g')), 
                                Robot::isGreenZone));

        yellowZone.whenPressed(new ConditionalCommand(
                                new AccuracyShootCommand(shooter, spindexer, 'y'),
                                new InstantCommand(() -> Robot.setZone('y')), 
                                Robot::isYellowZone));

        blueZone.whenPressed(new ConditionalCommand(
                                new AccuracyShootCommand(shooter, spindexer, 'b'),
                                new InstantCommand(() -> Robot.setZone('b')), 
                                Robot::isBlueZone));

        redZone.whenPressed(new ConditionalCommand(
                                new AccuracyShootCommand(shooter, spindexer, 'r'),
                                new InstantCommand(() -> Robot.setZone('r')), 
                                Robot::isRedZone));

        // Non-Critical Function Button Defintions
        unjamButton.whenPressed(new ConditionalCommand(
                                    new InstantCommand(() -> spindexer.clearedJam()),
                                    new InstantCommand(() -> spindexer.startJamClear()), 
                                    spindexer::clearingJam));

        liftIntakeButton.whenPressed(new InstantCommand(() -> intake.foldIntake()));
        coolMotorsButton.whenPressed(new InstantCommand(() -> drivetrain.coolFalcons()));

        // Attach Auto Command files to their allocated memory space
        barrel = new BarrelAuto(drivetrain);
        slalom = new SlalomAuto(drivetrain);
        bounce = new BounceAuto(drivetrain);

        A_RED = new GalacticSearchARED(drivetrain, intake, spindexer);
        A_BLUE = new GalacticSearchABLUE(drivetrain, intake, spindexer);
        B_RED = new GalacticSearchBRED(drivetrain, intake, spindexer);
        B_BLUE = new GalacticSearchBBLUE(drivetrain, intake, spindexer);

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

    public static Block getBiggestBlock() {
		// Gets the number of "blocks", identified targets, that match signature 1 on the Pixy2,
		// does not wait for new data if none is available,
        // and limits the number of returned blocks to 25, for a slight increase in efficiency
        final int blockSignature = 1;
		int blockCount = ballDetectorPixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
		System.out.println("Found " + blockCount + " blocks!"); // Reports number of blocks found
		if (blockCount <= 0) {
			return null; // If blocks were not found, stop processing
		}
		ArrayList<Block> blocks = ballDetectorPixy.getCCC().getBlockCache(); // Gets a list of all blocks found by the Pixy2
		Block largestBlock = null;
		for (Block block : blocks) {
			if (block.getSignature() == blockSignature) {
				if (largestBlock == null) {
					largestBlock = block;
				} else if (block.getWidth() > largestBlock.getWidth()) {
					largestBlock = block;
				}
			}
		}
		return largestBlock;
	}

    private void zoneColorIndicator(){
        /*
            This function serves to change the color of the AccuracyZone BooleanBox widget to match
            the color of the Accuracy Challenge Zone we are shooting from.
        */
        if (isGreenZone()) ZoneWidget.withProperties(Map.of("colorWhenTrue", "#62C334"));
        if (isYellowZone()) ZoneWidget.withProperties(Map.of("colorWhenTrue", "#F0FF00"));
        if (isBlueZone()) ZoneWidget.withProperties(Map.of("colorWhenTrue", "#0078FF"));
        if (isRedZone()) ZoneWidget.withProperties(Map.of("colorWhenTrue", "#FF0000"));

        if (isGreenZone() || isYellowZone() || isBlueZone() || isRedZone()) AccuracyZone.setBoolean(true);
        else AccuracyZone.setBoolean(false);

        SmartDashboard.putString("Zone", Character.toString(zoneConfirm));
    }

    public static void setZone(char zone) {
        /* Set the Zone Character variable to lock in a zone choice for firing.
            Accuracy Challenge exclusive function */
        zoneConfirm = zone;
        System.out.println("Zone Changed to " + zone);
        System.out.println("ZoneConfirm var is now " + zoneConfirm);
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
        Block biggest = getBiggestBlock();
        if (!(biggest == null)){
            SmartDashboard.putNumber("Pixy Biggest Block X", biggest.getX());
            SmartDashboard.putNumber("Pixy Biggest Block Y", biggest.getY());
        }
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
        // If driver is holding the left bumper, shift into high gear. Else, stay in low.
        if (driver.getBumper(Hand.kLeft)) {
            drivetrain.setShiftMode(true);
        }
        else drivetrain.setShiftMode(false);

        if (driver.getStickButton(Hand.kLeft) && driver.getStickButton(Hand.kRight)) {
            drivetrain.zeroSensors();
        }

        // Manual Hood Control Logic
        if (operator.getPOV(0) == -1) POVDebounce = false; // If the D-Pad isn't touched, reset.
        else if (POVDebounce == false){ // If the D-Pad is engaged and it wasn't engaged last cycle...
            POVDebounce = true; // set the flag to prevent multi-press.
            if (operator.getPOV(0) == 0) shooter.modifyHoodSet(5.0); // If Up...+5 degrees
            else if (operator.getPOV(0) == 90) shooter.modifyHoodSet(2.5); // if Right, +2.5 Degrees
            else if (operator.getPOV(0) == 180) shooter.modifyHoodSet(-5.0); // if Down, -5 Degrees
            else if (operator.getPOV(0) == 270) shooter.modifyHoodSet(-2.5); // if Left, -2.5 Degrees
            else POVDebounce = false; // If we're hitting a combo of D-Pad buttons, reset.
        }

        // Manual Turret Control
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
