package pilot;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.RenderingHints;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.concurrent.ThreadLocalRandom;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.vecmath.Matrix4f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import org.joml.Matrix4d;
import org.joml.Vector2d;
import org.joml.Vector3d;

import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.CollisionFilterGroups;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.CollisionWorld;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.ConvexHullShape;
import com.bulletphysics.collision.shapes.PolyhedralConvexShape;
import com.bulletphysics.dynamics.ActionInterface;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.Generic6DofConstraint;
import com.bulletphysics.dynamics.constraintsolver.HingeConstraint;
import com.bulletphysics.dynamics.constraintsolver.RotationalLimitMotor;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.TranslationalLimitMotor;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.IDebugDraw;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;

// https://github.com/normen/jbullet/tree/master/src/com/bulletphysics
public class JBulletTest extends JPanel
		implements MouseWheelListener, MouseListener, MouseMotionListener, ComponentListener, KeyListener {

	private static final long serialVersionUID = -730734892177645353L;
	private Point dragFrom = null;
	private static Vector3d drag = new Vector3d();
	private static Matrix4d viewMatrix = new Matrix4d();
	//private static double cameraZoom = 1 / 10d;
	private static double cameraZoom = 1 / 4d;
	//private static Vector2d cameraPosition = new Vector2d(0, -14);
	private static Vector2d cameraPosition = new Vector2d(0, 0);
	private static double viewportWidth = 1200;
	private static double viewportHeight = 800;

	private static double renderFPS = 30;
	private static float physicsFPS = 100;

	private static float wy = 50f;
	
	private static Object worldLock = new Object();
	private static DynamicsWorld world = null;
	private static RigidBody zeroBody = null;
	
	private static int sceneIndex = 2;

	public JBulletTest() {
		addMouseWheelListener(this);
		addMouseListener(this);
		addMouseMotionListener(this);
		addComponentListener(this);
	}

	private static void initScene() {
		initScene(sceneIndex);
	}
	
	private static void initScene(int si) {
		sceneIndex = si;
		synchronized (worldLock) {
			clearScene();
			switch (sceneIndex) {
			case 1:
				initScene1();
				break;
			case 2:
				initScene2();
				break;
			default:
				break;
			}
		}
	}

	private static void clearScene() {
		BroadphaseInterface broadphase = new DbvtBroadphase();
		DefaultCollisionConfiguration collisionConfiguration = new DefaultCollisionConfiguration();
		CollisionDispatcher dispatcher = new CollisionDispatcher(collisionConfiguration);
		SequentialImpulseConstraintSolver solver = new SequentialImpulseConstraintSolver();
		world = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
		world.setGravity(new Vector3f(0, -10, 0));
		zeroBody = new RigidBody(0, null, null);
		world.addRigidBody(zeroBody);
	}

	private static void initScene2() {
		
		RigidBody floor = addBox(20, .4, 0, 0, -.5, 0, 0, 0, 0);
		
		float dl = .02f;
		float l1 = 2f;
		RigidBody box1 = addBox(l1, .02, 0.1, l1/2, 0, 0, 0, 0, 0);
		//ServoMotorJoint joint1 = new ServoMotorJoint(box1, new Vector3f(-(l1/2+dl), 0, 0), new Vector3f(0, 0, 1));
		ServoMotorJoint joint1 = new ServoMotorJoint(addBox(.01, .01, 0.1, 0, 0, 0, 0, 0, 0), box1, new Vector3f(0, 0, 0), new Vector3f(-(l1/2+dl), 0, 0), new Vector3f(0, 0, 1), new Vector3f(0, 0, 1));
		joint1.setTargetAngle((float) (joint1.getAngle()+Math.PI/4));
		world.addConstraint(joint1);
		
		float l2 = 1.5f;
		RigidBody box2 = addBox(l2, .02, 0.1, l1+dl+l2/2+dl, 0, 0, 0, 0, 0);
		ServoMotorJoint joint2 = new ServoMotorJoint(box1, box2, new Vector3f(l1/2+dl, 0, 0), new Vector3f(-l2/2-dl, 0, 0), new Vector3f(0, 0, 1), new Vector3f(0, 0, 1));
		joint2.setTargetAngle((float) (joint2.getAngle()+Math.PI/5));
		world.addConstraint(joint2);
		
		float l3 = 1f;
		RigidBody box3 = addBox(l3, .02, .1, l1+dl+l2+dl+l3/2+dl, 0, 0, 0, 0, 0);
		ServoMotorJoint joint3 = new ServoMotorJoint(box2, box3, new Vector3f(l2/2+dl, 0, 0), new Vector3f(-l3/2-dl, 0, 0), new Vector3f(0, 0, 1), new Vector3f(0, 0, 1));
		joint3.setTargetAngle((float) (joint3.getAngle()+Math.PI/4));
		world.addConstraint(joint3);
		
		world.addCollisionObject(box3, (short)10, (short)CollisionFlags.KINEMATIC_OBJECT);
		world.addCollisionObject(floor, (short)11, (short)CollisionFlags.STATIC_OBJECT);
		
		/*
		RigidBody box0 = new RigidBody(0, null, null);
		world.addRigidBody(box0);
		
		float de = .04f;
		
		float l1 = 2;
		RigidBody box1 = addBox(l1, .1, 0.1, l1/2+de, 0, 0, 0, 0, 0);
		ServoMotorJoint2 joint0 = new ServoMotorJoint2(
				//box0,
				addBox(.1, .1, 0.1, 0, 0, 0, 0, 0, 0),
				box1, 
				new Transform(new Matrix4f(new Quat4f(), new Vector3f(0, 0, 0), 1)), 
				new Transform(new Matrix4f(new Quat4f(), new Vector3f(-(l1/2+de), 0, 0), 1)), 
				false);
		joint0.setTargetAngle(-3.14f/3f);
		world.addConstraint(joint0);
		
		float l2 = 1.5f;
		RigidBody box2 = addBox(l2, .1, 0.1, l1+l2/2+de, 0, 0, 0, 0, 0);
		ServoMotorJoint2 joint1 = new ServoMotorJoint2(
				box1, 
				box2, 
				new Transform(new Matrix4f(new Quat4f(), new Vector3f(l1/2+de, 0, 0), 1)), 
				new Transform(new Matrix4f(new Quat4f(), new Vector3f(-(l2/2+de), 0, 0), 1)), 
				false);
		joint1.setTargetAngle(3.14f/3f);
		world.addConstraint(joint1);
		
		float l3 = 1f;
		RigidBody box3 = addBox(l3, .1, .1, l1+l2+l3/2+de, 0, 0, 0, 0, 0);
		ServoMotorJoint2 joint2 = new ServoMotorJoint2(
				box2, 
				box3, 
				new Transform(new Matrix4f(new Quat4f(), new Vector3f(l2/2+de, 0, 0), 1)), 
				new Transform(new Matrix4f(new Quat4f(), new Vector3f(-(l3/2+de), 0, 0), 1)), 
				false);
		joint2.setTargetAngle(3.14f/5f);
		world.addConstraint(joint2);
		*/
		
	}
	
	private static void initScene1() {
		addBox(100, 10, 0, 0, -10f / 2f, 0, 0, 0, 0);
		addBox(100, 10, 0, 0, 100f, 0, 0, 0, 0);
		addBox(10, 100, 0, -100f / 2f, 100f / 2f, 0, 0, 0, 0);
		addBox(10, 100, 0, 100f / 2f, 100f / 2f, 0, 0, 0, 0);
	}

	private static CollisionShape createBoxShape(double w, double h) {
		float wx = (float) (w / 2d);
		float wz = (float) (h / 2d);

		ObjectArrayList<Vector3f> points = new ObjectArrayList<>();

		points.add(new Vector3f(wx, wy, wz));
		points.add(new Vector3f(-wx, wy, wz));
		points.add(new Vector3f(-wx, wy, -wz));
		points.add(new Vector3f(wx, wy, -wz));

		points.add(new Vector3f(wx, -wy, wz));
		points.add(new Vector3f(-wx, -wy, wz));
		points.add(new Vector3f(-wx, -wy, -wz));
		points.add(new Vector3f(wx, -wy, -wz));

		for (Vector3f p : points) {
			float id = -p.y;
			p.y = p.z;
			p.z = id;
		}

		return new ConvexHullShape(points);
	}

	private static CollisionShape createRandomPolyShape(int n, double r) {

		ObjectArrayList<Vector3f> points = new ObjectArrayList<>();

		float[] ang = new float[n];
		for (int i = 0; i < n; i++) {
			ang[i] = (float) ThreadLocalRandom.current().nextDouble(0, Math.PI * 2);
		}
		Arrays.sort(ang);
		for (int i = 0; i < n; i++) {
			points.add(new Vector3f((float) (r * Math.cos(ang[i])), wy, (float) (r * Math.sin(ang[i]))));
		}
		for (int i = 0; i < n; i++) {
			points.add(new Vector3f(points.get(i).x, -wy, points.get(i).z));
		}

		for (Vector3f p : points) {
			float id = -p.y;
			p.y = p.z;
			p.z = id;
		}

		return new ConvexHullShape(points);
	}

	private static RigidBody addPoly(int n, double r, double m, double x, double y, double a, double vx, double vy,
			double va) {
		CollisionShape shape = createRandomPolyShape(n, r);
		return createRigidBody(shape, m, x, y, a, vx, vy, va);
	}

	private static RigidBody addBox(double w, double h, double m, double x, double y, double a, double vx, double vy,
			double va) {

		CollisionShape shape = createBoxShape(w, h);
		return createRigidBody(shape, m, x, y, a, vx, vy, va);
	}

	private static RigidBody createRigidBody(CollisionShape shape, double m, double x, double y, double a, double vx,
			double vy, double va) {
		Quat4f rot = new Quat4f();
		QuaternionUtil.setRotation(rot, new Vector3f(0, 0, 1), (float) a);

		DefaultMotionState motionState = new DefaultMotionState(
				new Transform(new Matrix4f(rot, new Vector3f((float) x, (float) y, 0), 1.0f)));

		Vector3f inertia = new Vector3f(0, 0, 0);
		shape.calculateLocalInertia((float) m, inertia);

		RigidBodyConstructionInfo rigidBodyCI = new RigidBodyConstructionInfo((float) m, motionState, shape, inertia);
		RigidBody body = new RigidBody(rigidBodyCI);
		body.setUserPointer(null);
		body.applyCentralImpulse(new Vector3f((float) vx, (float) vy, 0));
		body.applyTorqueImpulse(new Vector3f(0, 0, (float) va));

		body.setRestitution(.1f);
		body.setSleepingThresholds(.1f, .1f);
		
		Transform tr1 = new Transform();
		tr1.setIdentity();
		Transform tr2 = new Transform();
		tr2.setIdentity();
		Generic6DofConstraint contr2D = new Generic6DofConstraint(body, zeroBody, tr1, tr2, false);
		contr2D.setLimit(0, 1, 0);
		contr2D.setLimit(1, 1, 0);
		contr2D.setLimit(2, 0, 0);
		contr2D.setLimit(3, 0, 0);
		contr2D.setLimit(4, 0, 0);
		contr2D.setLimit(5, 1, 0);

		synchronized (worldLock) {
			world.addConstraint(contr2D);
			world.addRigidBody(body);
		}

		return body;
	}

	private static RigidBody addRandomPoly() {
		int n = ThreadLocalRandom.current().nextInt(3, 8);
		double r = ThreadLocalRandom.current().nextDouble(.1, 4);
		double m = ThreadLocalRandom.current().nextDouble(.01, 10);
		double x = ThreadLocalRandom.current().nextDouble(-10, 10);
		double y = ThreadLocalRandom.current().nextDouble(10, 40);
		double a = ThreadLocalRandom.current().nextDouble(-Math.PI, Math.PI);
		double vx = ThreadLocalRandom.current().nextDouble(-50, 50);
		double vy = ThreadLocalRandom.current().nextDouble(-100, 100);
		double va = ThreadLocalRandom.current().nextDouble(-40, 40);

		return addPoly(n, r, m, x, y, a, vx, vy, va);
	}

	private static RigidBody addRandomBox() {
		double w = ThreadLocalRandom.current().nextDouble(.1, 4);
		double h = ThreadLocalRandom.current().nextDouble(.1, 4);
		double m = ThreadLocalRandom.current().nextDouble(.01, 10);
		double x = ThreadLocalRandom.current().nextDouble(-10, 10);
		double y = ThreadLocalRandom.current().nextDouble(10, 40);
		double a = ThreadLocalRandom.current().nextDouble(-Math.PI, Math.PI);
		double vx = ThreadLocalRandom.current().nextDouble(-50, 50);
		double vy = ThreadLocalRandom.current().nextDouble(-100, 100);
		double va = ThreadLocalRandom.current().nextDouble(-40, 40);

		return addBox(w, h, m, x, y, a, vx, vy, va);
	}

	public static void main(String[] args) {

		JFrame frame = new JFrame("Physics");
		frame.setSize((int) viewportWidth, (int) viewportHeight);
		frame.setLocationRelativeTo(null);
		frame.setLayout(new BorderLayout());
		JBulletTest main = new JBulletTest();
		frame.add(main, BorderLayout.CENTER);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.addWindowListener(new WindowAdapter() {
			@Override
			public void windowOpened(WindowEvent e) {
				updateViewMatrix();
			}
		});
		frame.addKeyListener(main);
		frame.setVisible(true);

		initScene();

		// Szimul�ci� futtat�sa.
		new Thread(() -> {

			long prevTime = System.nanoTime();
			double renderAcc = 0;
			double updateAcc = 0;

			try {

				for (;;) {

					long currentTime = System.nanoTime();
					double dt = (double) (System.nanoTime() - prevTime) / 1000000000d;
					prevTime = currentTime;

					updateAcc += dt;
					if (updateAcc >= 1d / physicsFPS) {
						updateAcc = 0;
						synchronized (worldLock) {
							world.stepSimulation(1 / physicsFPS, 2);
						}
					}

					renderAcc += dt;
					if (renderAcc >= 1d / renderFPS) {
						renderAcc = 0;
						frame.repaint();
					}

					Thread.sleep(1);

				}

			} catch (InterruptedException ex) {
				Thread.currentThread().interrupt();
			}

		}).start();
	}

	private static void updateViewMatrix() {
		viewMatrix.identity().scale(1, -1, 1).translate(viewportWidth / 2, -viewportHeight / 2, 0)
				.scale(Math.min(viewportWidth, viewportHeight) / 4);
		viewMatrix.scale(cameraZoom);
		viewMatrix.translate(cameraPosition.x + drag.x, cameraPosition.y + drag.y, 0);
	}

	private void renderScene(Graphics g2) {

		if (world == null) {
			return;
		}

		synchronized (worldLock) {
			for (CollisionObject o : world.getCollisionObjectArray()) {
				drawRigidBody(g2, (RigidBody) o);
			}
		}
	}

	private void drawRigidBody(Graphics g2, RigidBody body) {
		Transform trans = new Transform();
		body.getMotionState().getWorldTransform(trans);

		PolyhedralConvexShape shape = (PolyhedralConvexShape) body.getCollisionShape();
		g2.setColor(Color.black);

		int n = shape.getNumVertices() / 2;
		Vector3d[] pts = new Vector3d[n];
		for (int i = 0; i < n; i++) {
			Vector3f v0 = new Vector3f();
			shape.getVertex(i, v0);
			trans.transform(v0);
			pts[i] = viewMatrix.transformPosition(new Vector3d(v0.x, v0.y, -v0.z));
		}
		for (int i = 0; i < n - 1; i++) {
			g2.drawLine((int) pts[i].x, (int) pts[i].y, (int) pts[i + 1].x, (int) pts[i + 1].y);
		}
		g2.drawLine((int) pts[0].x, (int) pts[0].y, (int) pts[n - 1].x, (int) pts[n - 1].y);

		g2.setColor(Color.red);
		Vector3f v0 = new Vector3f();
		trans.transform(v0);
		Vector3d m = viewMatrix.transformPosition(new Vector3d(v0.x, v0.y, -v0.z));
		g2.drawLine((int) m.x - 2, (int) m.y, (int) m.x + 2, (int) m.y);
		g2.drawLine((int) m.x, (int) m.y - 2, (int) m.x, (int) m.y + 2);
	}

	@Override
	protected void paintComponent(Graphics g) {

		super.paintComponent(g);
		Graphics2D g2 = (Graphics2D) g.create();
		g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

		// Jelenet kirajzol�sa.
		renderScene(g2);

		// Tengelyek.
		Vector3d origo = viewMatrix.transformPosition(new Vector3d(0, 0, 0));
		Vector3d unitX = viewMatrix.transformPosition(new Vector3d(1, 0, 0));
		Vector3d unitY = viewMatrix.transformPosition(new Vector3d(0, 1, 0));
		g2.setColor(Color.red);
		g2.drawLine((int) origo.x, (int) origo.y, (int) unitX.x, (int) unitX.y);
		g2.setColor(Color.green);
		g2.drawLine((int) origo.x, (int) origo.y, (int) unitY.x, (int) unitY.y);
	}

	@Override
	public void mouseWheelMoved(MouseWheelEvent e) {
		double scroll = 1 - 2 * e.getPreciseWheelRotation() / 10d;
		cameraZoom *= scroll;

		Point p = e.getPoint();

		Vector3d v0 = new Vector3d(p.x - viewportWidth / 2, p.y - viewportHeight / 2, 0);
		Matrix4d m = viewMatrix.invert(new Matrix4d());
		Vector3d t0 = m.transformDirection(v0);

		updateViewMatrix();

		m = viewMatrix.invert(new Matrix4d());
		Vector3d v1 = new Vector3d(p.x - viewportWidth / 2, p.y - viewportHeight / 2, 0);
		Vector3d t1 = m.transformDirection(v1);

		t1.sub(t0);
		cameraPosition.x += t1.x;
		cameraPosition.y += t1.y;

		updateViewMatrix();
	}

	@Override
	public void mousePressed(MouseEvent e) {
		if (SwingUtilities.isMiddleMouseButton(e)) {
			dragFrom = e.getPoint();
		}
	}

	@Override
	public void mouseReleased(MouseEvent e) {
		if (SwingUtilities.isMiddleMouseButton(e)) {
			dragFrom = null;
			cameraPosition.x += drag.x;
			cameraPosition.y += drag.y;
			drag.set(0, 0, 0);
		}
	}

	@Override
	public void mouseDragged(MouseEvent e) {
		if (SwingUtilities.isMiddleMouseButton(e)) {
			Point point = e.getPoint();
			drag.set(point.getX() - dragFrom.getX(), -(point.getY() - dragFrom.getY()), 0);
			double unitLengthScreen = viewMatrix.transformDirection(new Vector3d(1, 0, 0)).length();
			drag.div(unitLengthScreen);

			updateViewMatrix();
		}
	}

	@Override
	public void componentResized(ComponentEvent e) {
		viewportWidth = getWidth();
		viewportHeight = getHeight();
		updateViewMatrix();
	}

	@Override
	public void mouseClicked(MouseEvent e) {
		
		float m = 1;
		if (SwingUtilities.isRightMouseButton(e)) {
			Vector3d p = viewMatrix.invert(new Matrix4d()).transformPosition(new Vector3d(e.getPoint().x, e.getPoint().y, 0));
			addBox(.10, .10, m, p.x, p.y, 0, 0, 0, 0);
		}
	}

	@Override
	public void mouseEntered(MouseEvent e) {
		// Not implemented.
	}

	@Override
	public void mouseExited(MouseEvent e) {
		// Not implemented.
	}

	@Override
	public void mouseMoved(MouseEvent e) {
		// Not implemented.
	}

	@Override
	public void componentHidden(ComponentEvent e) {
		// Not implemented.
	}

	@Override
	public void componentMoved(ComponentEvent e) {
		// Not implemented.
	}

	@Override
	public void componentShown(ComponentEvent e) {
		// Not implemented.
	}

	@Override
	public void keyPressed(KeyEvent e) {
		// Not implemented.
	}

	@Override
	public void keyReleased(KeyEvent e) {

		switch (e.getKeyCode()) {
		case KeyEvent.VK_ESCAPE:
			System.exit(0);
			break;
		case KeyEvent.VK_R:
			initScene();
			break;
		case KeyEvent.VK_A:
			if (e.isShiftDown()) {
				addRandomBox();
			} else {
				addRandomPoly();
			}
			break;
		case KeyEvent.VK_Q:
			for (int i = 0; i < 400; i++) {
				if (e.isShiftDown()) {
					addRandomBox();
				} else {
					addRandomPoly();
				}
			}
			break;
		case KeyEvent.VK_1:
			initScene(1);
			break;
		case KeyEvent.VK_2:
			initScene(2);
			break;
		default:
			break;
		}
	}

	@Override
	public void keyTyped(KeyEvent e) {
		// Not implemented.
	}
}
