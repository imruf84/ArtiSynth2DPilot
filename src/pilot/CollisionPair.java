package pilot;

import java.util.Objects;

public class CollisionPair {

	private final MyRigidBody body1;
	private final MyRigidBody body2;

	public CollisionPair(MyRigidBody body1, MyRigidBody body2) {
		super();
		this.body1 = body1;
		this.body2 = body2;
	}

	public MyRigidBody getBody1() {
		return body1;
	}

	public MyRigidBody getBody2() {
		return body2;
	}

	public boolean needCollide() {
		return false;
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
		CollisionPair other = (CollisionPair) obj;
		return (Objects.equals(getBody1(), other.getBody1()) && Objects.equals(getBody2(), other.getBody2()))
				|| (Objects.equals(getBody1(), other.getBody2()) && Objects.equals(getBody2(), other.getBody1()));
	}

	@Override
	public int hashCode() {
		return Objects.hash(getBody1(), getBody2());
	}
}
