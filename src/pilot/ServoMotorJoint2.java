package pilot;

import javax.vecmath.Vector3f;

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.constraintsolver.Generic6DofConstraint;
import com.bulletphysics.dynamics.constraintsolver.RotationalLimitMotor;
import com.bulletphysics.linearmath.Transform;

public class ServoMotorJoint2 extends Generic6DofConstraint {
	
	private float targetAngle;

	public ServoMotorJoint2(RigidBody rbA, RigidBody rbB, Transform frameInA, Transform frameInB, boolean useLinearReferenceFrameA) {
		super(rbA, rbB, frameInA, frameInB, useLinearReferenceFrameA);
	}
	
	@Override
	public void solveConstraint(float timeStep) {
		super.solveConstraint(timeStep);
		
		//System.out.println(df.format(joint0.getAngle(0))+"\t"+df.format(joint0.getAngle(1))+"\t"+df.format(joint0.getAngle(2)));
		
		float maxSpeed = 5f;
		float angle = getAngle(2);
		float diff = (float) Math.atan2(Math.sin(getTargetAngle()-angle), Math.cos(getTargetAngle()-angle));
		RotationalLimitMotor motor = getRotationalLimitMotor(2);
		//System.out.println(df.format(diff));
		float v = 0;
		if (Math.abs(diff) > 0) {
			v = (float) diff/Math.abs(diff);
			v *= Math.abs(diff)*maxSpeed;
			v = Math.min(v, maxSpeed);
		}
		
		motor.enableMotor = true;
		motor.targetVelocity = v;
		motor.maxMotorForce = 2;
		/*
		setLimit(0, 0, 0);
		setLimit(1, 0, 0);
		setLimit(2, 0, 0);
		setLimit(3, 0, 0);
		setLimit(4, 0, 0);
		setLimit(5, 1, -1);
		*/
		//getRigidBodyA().setAngularVelocity(new Vector3f(0, 0, -v));
	}

	public float getTargetAngle() {
		return targetAngle;
	}

	public void setTargetAngle(float targetAngle) {
		this.targetAngle = targetAngle;
	}
}
