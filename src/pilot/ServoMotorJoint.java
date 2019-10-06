package pilot;

import javax.vecmath.Vector3f;

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.constraintsolver.HingeConstraint;

public class ServoMotorJoint extends HingeConstraint {

	private float targetAngle;

	public ServoMotorJoint(RigidBody rbA, Vector3f pivotInA, Vector3f axisInA) {
		super(rbA, pivotInA, axisInA);
		setTargetAngle(getHingeAngle());
	}
	
	public ServoMotorJoint(RigidBody rbA, RigidBody rbB, Vector3f pivotInA, Vector3f pivotInB, Vector3f axisInA, Vector3f axisInB) {
		super(rbA, rbB, pivotInA, pivotInB, axisInA, axisInB);
		setTargetAngle(getHingeAngle());
	}
	
	@Override
	public void solveConstraint(float timeStep) {
		super.solveConstraint(timeStep);
		
		float maxSpeed = 10;
		float maxImpulse = 40;
		float angle = getHingeAngle();
		float diff = (float) Math.atan2(Math.sin(getTargetAngle()-angle), Math.cos(getTargetAngle()-angle));
		//float softness = .9f;
		float softness = 1f;
		//float bias = .3f;
		float bias = .0f;
		float relax = 1f;
		//float relax = .01f;
		//setLimit((float)(-2*Math.PI), (float)(2*Math.PI), softness, bias, relax);
		
		float v = 0;
		if (Math.abs(diff) > 0) {
			v = (float) diff/Math.abs(diff);
			v *= Math.abs(diff)*maxSpeed;
		}

		//getRigidBodyB().applyTorqueImpulse(new Vector3f(0, 0, v));
		//getRigidBodyB().applyTorque(new Vector3f(0, 0, v));
		//getRigidBodyB().setAngularVelocity(new Vector3f(0, 0, v));
		
		enableAngularMotor(true, v, maxImpulse);
	}

	public float getTargetAngle() {
		return targetAngle;
	}

	public void setTargetAngle(float targetAngle) {
		this.targetAngle = targetAngle;
	}
	
	public float getAngle() {
		return getHingeAngle();
	}
}
