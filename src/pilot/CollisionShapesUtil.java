package pilot;

import java.util.Arrays;
import java.util.concurrent.ThreadLocalRandom;

import javax.vecmath.Vector3f;

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.ConvexHullShape;
import com.bulletphysics.util.ObjectArrayList;

public class CollisionShapesUtil {
	
	private static final float WY = 5f;
	
	private CollisionShapesUtil() {
	}
	
	public static Vector3f createIntertia(CollisionShape shape, float m) {
		Vector3f inertia = new Vector3f(0, 0, 0);
		shape.calculateLocalInertia((float) m , inertia);
		
		return inertia;
	}
	
	public static CollisionShape createBoxShape(double w, double h) {
		float wx = (float) (w / 2d);
		float wz = (float) (h / 2d);

		ObjectArrayList<Vector3f> points = new ObjectArrayList<>();

		points.add(new Vector3f(wx, WY, wz));
		points.add(new Vector3f(-wx, WY, wz));
		points.add(new Vector3f(-wx, WY, -wz));
		points.add(new Vector3f(wx, WY, -wz));

		points.add(new Vector3f(wx, -WY, wz));
		points.add(new Vector3f(-wx, -WY, wz));
		points.add(new Vector3f(-wx, -WY, -wz));
		points.add(new Vector3f(wx, -WY, -wz));

		for (Vector3f p : points) {
			float id = -p.y;
			p.y = p.z;
			p.z = id;
		}

		return new ConvexHullShape(points);
	}

	public static CollisionShape createRandomPolyShape(int n, double r) {

		ObjectArrayList<Vector3f> points = new ObjectArrayList<>();

		float[] ang = new float[n];
		for (int i = 0; i < n; i++) {
			ang[i] = (float) ThreadLocalRandom.current().nextDouble(0, Math.PI * 2);
		}
		Arrays.sort(ang);
		for (int i = 0; i < n; i++) {
			points.add(new Vector3f((float) (r * Math.cos(ang[i])), WY, (float) (r * Math.sin(ang[i]))));
		}
		for (int i = 0; i < n; i++) {
			points.add(new Vector3f(points.get(i).x, -WY, points.get(i).z));
		}

		for (Vector3f p : points) {
			float id = -p.y;
			p.y = p.z;
			p.z = id;
		}

		return new ConvexHullShape(points);
	}
}
