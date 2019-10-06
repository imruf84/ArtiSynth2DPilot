package pilot;

import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

import com.bulletphysics.collision.dispatch.CollisionConfiguration;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.CollisionObject;

public class MyCollisionDispatcher extends CollisionDispatcher {

	private final Set<CollisionPair> collisionPairs = new HashSet<>();
	
	public MyCollisionDispatcher(CollisionConfiguration c) {
		super(c);
	}
	
	@Override
	public boolean needsCollision(CollisionObject body0, CollisionObject body1) {
		
		CollisionPair key = new CollisionPair((MyRigidBody)body0, (MyRigidBody)body1);
		Iterator<CollisionPair> iterator = getCollisionPairs().iterator();
		while(iterator.hasNext()) {
			CollisionPair next = iterator.next();
			if (next.equals(key)) {
				return next.needCollide();
			}
		}
		
		return super.needsCollision(body0, body1);
	}

	public Set<CollisionPair> getCollisionPairs() {
		return collisionPairs;
	}
	
	public CollisionPair addCollisionPair(CollisionPair p) {
		getCollisionPairs().add(p);
		return p;
	}
	
	public CollisionPair removeCollisionPair(CollisionPair p) {
		getCollisionPairs().remove(p);
		return p;
	}

}
