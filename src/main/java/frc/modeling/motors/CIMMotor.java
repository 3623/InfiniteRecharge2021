package frc.modeling.motors;

/**
 * CIM Motor model, implements Motor abstract class 
 * 
 * @author eric
 *
 */
public class CIMMotor extends Motor{
	public CIMMotor(){
		STALL_TORQUE = 2.41; // N. m
		FREE_SPEED = 5330.0; // rpm
		STALL_CURRENT = 130.1;
		FREE_CURRENT = 3.8;
		kSlopeTorque = -STALL_TORQUE / FREE_SPEED;
		kSlopeCurrent = -(STALL_CURRENT - FREE_CURRENT) / FREE_SPEED;
	}
	
	// Testing calculations
	public static void main ( String[] args ) {
			new CIMMotor();
			System.out.println(CIMMotor.outputTorque(12, 5330));
			System.out.println(CIMMotor.outputTorque(12, 0));
			System.out.println(CIMMotor.outputTorque(12, 2000));
			System.out.println(CIMMotor.outputTorque(12, -5330));
	  }
}