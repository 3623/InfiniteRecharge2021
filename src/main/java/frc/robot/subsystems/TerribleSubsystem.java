package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TerribleSubsystem extends SubsystemBase{

    protected static final int UPDATE_RATE = 200;

    protected void updateThreadStart() {
        Thread t = new Thread(() -> {
            while (!Thread.interrupted()) {
                this.update();
                try {
                    Thread.sleep(1000 / UPDATE_RATE);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
        t.start();
    }

    /**
     * Override this to implement a test for subsystem function
     */
    protected boolean test() {
        return true;
    }

    protected boolean checkValue(String name, double value, double goal, double percThreshold) {
        return true;
    }

    public void runTests() {
        Thread t = new Thread(() -> {
            this.test();
        });
        t.start();
    }

    protected void update(){    }

    @Override
    public void periodic() {
        SmartDashboard.putData(this);
    }

    protected void display(String key, double value) {
        SmartDashboard.putNumber(getName() + "/" + key, value);
    }
}
