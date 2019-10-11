package pilot;

import java.util.Objects;
import java.util.UUID;

import javax.vecmath.Vector3f;

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.MotionState;

public class MyRigidBody extends RigidBody {

	private String id;
	
	public MyRigidBody(String id, float mass, MotionState motionState, CollisionShape collisionShape) {
		super(mass, motionState, collisionShape);
		setId(id);
	}
	
	public MyRigidBody(float mass, MotionState motionState, CollisionShape collisionShape) {
		this(null, mass, motionState, collisionShape);
	}
	
	public MyRigidBody(String id, RigidBodyConstructionInfo rigidBodyCI) {
		super(rigidBodyCI);
		setId(id);
	}
	
	public MyRigidBody(RigidBodyConstructionInfo rigidBodyCI) {
		this(null, rigidBodyCI);
	}

	private void setId(String id) {
		if (id == null) {
			this.id = UUID.randomUUID().toString();
		} else {
			this.id = id;
		}
	}
	
	public String getId() {
		return id;
	}
	
	@Override
	public boolean equals(Object obj) {
		if (this == obj) {
			return true;
		}
		if (obj == null) {
			return false;
		}
		if (getClass() != obj.getClass()) {
			return false;
		}
		MyRigidBody other = (MyRigidBody) obj;
		return Objects.equals(getId(), other.getId());
	}

	@Override
	public int hashCode() {
		return Objects.hash(getId());
	}
		
	public static MyRigidBody createRigidBody(float x, float y, float a, float m, CollisionShape shape) {

		if (shape != null) {
			MotionState motionState = MotionStateUtil.createDefaultMotionState(x, y, a);
			Vector3f inertia = CollisionShapesUtil.createIntertia(shape, m);
			RigidBodyConstructionInfo rigidBodyCI = new RigidBodyConstructionInfo((float) m, motionState, shape, inertia);
			MyRigidBody body = new MyRigidBody(rigidBodyCI);
			body.setUserPointer(null);

			return body;
		}

		return new MyRigidBody(0, null, null);
	}
	
}
