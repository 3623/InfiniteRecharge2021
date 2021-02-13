package frc.modeling.motors;

/**
 * CIM Motor model, implements Motor abstract class
 *
 * @author eric
 *
 */
public class Falcon extends Motor{
	public Falcon(){
		STALL_TORQUE = 4.69; // N. m
		FREE_SPEED = 6380.0; // rpm
		STALL_CURRENT = 257.0;
		FREE_CURRENT = 1.5;
		kSlopeTorque = -STALL_TORQUE / FREE_SPEED;
		kSlopeCurrent = -(STALL_CURRENT - FREE_CURRENT) / FREE_SPEED;
	}

	// Testing calculations
	public static void main ( String[] args ) {
			new Falcon();
			System.out.println(Falcon.outputTorque(12, 5330));
			System.out.println(Falcon.outputTorque(12, 0));
			System.out.println(Falcon.outputTorque(12, 2000));
			System.out.println(Falcon.outputTorque(12, -5330));
	  }
}