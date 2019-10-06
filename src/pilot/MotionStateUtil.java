package pilot;

import javax.vecmath.Matrix4f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.MotionState;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.Transform;

public class MotionStateUtil {
	
	private MotionStateUtil() {
	}
	
	public static MotionState createDefaultMotionState(float x, float y, float a) {
		Quat4f rot = new Quat4f();
		QuaternionUtil.setRotation(rot, new Vector3f(0, 0, 1), (float) a);

		return new DefaultMotionState(new Transform(new Matrix4f(rot, new Vector3f((float) x, (float) y, 0), 1.0f)));
	}
}
