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
		
		float maxSpeed = 4;
		float maxImpulse = 1.5f;
		float angle = getHingeAngle();
		float diff = (float) Math.atan2(Math.sin(getTargetAngle()-angle), Math.cos(getTargetAngle()-angle));
		float v = 0;
		if (Math.abs(diff) > 0) {
			v = (float) diff/Math.abs(diff);
			v *= Math.abs(diff)*maxSpeed;
		}

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
