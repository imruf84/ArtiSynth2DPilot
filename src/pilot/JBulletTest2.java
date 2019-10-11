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
import java.util.concurrent.ThreadLocalRandom;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.vecmath.Vector3f;

import org.joml.Matrix4d;
import org.joml.Vector2d;
import org.joml.Vector3d;

import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.broadphase.Dispatcher;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.CollisionWorld;
import com.bulletphysics.collision.dispatch.CollisionWorld.ClosestRayResultCallback;
import com.bulletphysics.collision.dispatch.CollisionWorld.LocalRayResult;
import com.bulletphysics.collision.dispatch.CollisionWorld.RayResultCallback;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.PolyhedralConvexShape;
import com.bulletphysics.dynamics.ActionInterface;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.constraintsolver.Generic6DofConstraint;
import com.bulletphysics.dynamics.constraintsolver.HingeConstraint;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.IDebugDraw;
import com.bulletphysics.linearmath.MotionState;
import com.bulletphysics.linearmath.Transform;

// https://github.com/normen/jbullet/tree/master/src/com/bulletphysics
public class JBulletTest2 extends JPanel
		implements MouseWheelListener, MouseListener, MouseMotionListener, ComponentListener, KeyListener {

	private static final long serialVersionUID = -730734892177645353L;
	private Point dragFrom = null;
	private static Vector3d drag = new Vector3d();
	private static Matrix4d viewMatrix = new Matrix4d();
	private static double cameraZoom = 1 / 6d;
	private static Vector2d cameraPosition = new Vector2d(0, -5);
	private static double viewportWidth = 1200;
	private static double viewportHeight = 800;

	private static double renderFPS = 30;
	private static float physicsFPS = 60;
	
	private static Object worldLock = new Object();
	private static DynamicsWorld world = null;
	private static RigidBody zeroBody = null;
	
	private static int sceneIndex = 2;
	private static Generic6DofConstraint grabConstraint = null;
	private static MyRigidBody grabBody;

	public JBulletTest2() {
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
		MyCollisionDispatcher dispatcher = new MyCollisionDispatcher(collisionConfiguration);
		SequentialImpulseConstraintSolver solver = new SequentialImpulseConstraintSolver();
		world = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
		world.setGravity(new Vector3f(0, -10, 0));
		zeroBody = new RigidBody(0, null, null);
		world.addRigidBody(zeroBody);
	}
	
	private static void initScene1() {
		MyRigidBody floor = addRigidBody(MyRigidBody.createRigidBody(0, 0, 0, 0, CollisionShapesUtil.createBoxShape(100, 1)));
		floor.setRestitution(.1f);
		floor.setFriction(.9f);
		
		MyRigidBody leftWall = addRigidBody(MyRigidBody.createRigidBody(-50, 50, 0, 0, CollisionShapesUtil.createBoxShape(1, 100)));
		leftWall.setRestitution(.1f);
		leftWall.setFriction(.9f);
		
		MyRigidBody rightWall = addRigidBody(MyRigidBody.createRigidBody(50, 50, 0, 0, CollisionShapesUtil.createBoxShape(1, 100)));
		rightWall.setRestitution(.1f);
		rightWall.setFriction(.9f);
		
		MyRigidBody body1 = addRigidBody(MyRigidBody.createRigidBody(1, 10, 1, .2f, CollisionShapesUtil.createBoxShape(1.4, 1.4)));
		body1.setRestitution(.1f);
		body1.setFriction(.9f);
		
		MyRigidBody body2 = addRigidBody(MyRigidBody.createRigidBody(1, 14, 4, .2f, CollisionShapesUtil.createBoxShape(1.4, 1.4)));
		body2.setRestitution(.1f);
		body2.setFriction(.9f);
		
		MyCollisionDispatcher dispatcher = (MyCollisionDispatcher) world.getDispatcher();
		dispatcher.addCollisionPair(new CollisionPair(body1, body2));
		
		MyRigidBody body3 = addRigidBody(MyRigidBody.createRigidBody(1, 20, -2, .2f, CollisionShapesUtil.createBoxShape(1.4, 1.4)));
		body3.setRestitution(.1f);
		body3.setFriction(.9f);
	}
	
	private static void initScene2() {
		
		MyCollisionDispatcher dispatcher = (MyCollisionDispatcher) world.getDispatcher();
		
		MyRigidBody floor = addRigidBody(MyRigidBody.createRigidBody(0, -1.5f, 0, 0, CollisionShapesUtil.createBoxShape(100, 1)));
		MyRigidBody leftWall = addRigidBody(MyRigidBody.createRigidBody(-50, 50, 0, 0, CollisionShapesUtil.createBoxShape(1, 100)));
		MyRigidBody rightWall = addRigidBody(MyRigidBody.createRigidBody(50, 50, 0, 0, CollisionShapesUtil.createBoxShape(1, 100)));
		
		float l0 = 10;
		MyRigidBody box0 = addRigidBody(MyRigidBody.createRigidBody(0, 5, 0, 1, CollisionShapesUtil.createBoxShape(l0, 1)));
		MyRigidBody box1 = addRigidBody(MyRigidBody.createRigidBody(0, 5, 0, 1, CollisionShapesUtil.createBoxShape(l0, 1)));
		MyRigidBody box2 = addRigidBody(MyRigidBody.createRigidBody(0, 5, 0, 1, CollisionShapesUtil.createBoxShape(l0, 1)));
		ServoMotorJoint joint0 = new ServoMotorJoint(box0, box1, new Vector3f(l0/2, 0, 0), new Vector3f(-l0/2, 0, 0), new Vector3f(0, 0, 1), new Vector3f(0, 0, 1));
		joint0.setTargetAngle(joint0.getAngle()+(float)Math.PI/3f);
		ServoMotorJoint joint1 = new ServoMotorJoint(box1, box2, new Vector3f(l0/2, 0, 0), new Vector3f(-l0/2, 0, 0), new Vector3f(0, 0, 1), new Vector3f(0, 0, 1));
		joint1.setTargetAngle(joint1.getAngle()+(float)Math.PI/3f);
		dispatcher.addCollisionPair(new CollisionPair(box0, box1));
		dispatcher.addCollisionPair(new CollisionPair(box1, box2));
		dispatcher.addCollisionPair(new CollisionPair(box0, box2));
		world.addConstraint(joint0);
		world.addConstraint(joint1);
	}
	
	private static MyRigidBody addRigidBody(MyRigidBody body) {
		synchronized (worldLock) {
			world.addRigidBody(body);
			fixTo2DPlane(body);
		}
		
		return body;
	}
	
	private static Generic6DofConstraint fixTo2DPlane(MyRigidBody body) {
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
		}
		
		return contr2D;
	}

	public static void main(String[] args) {

		JFrame frame = new JFrame("Physics");
		frame.setSize((int) viewportWidth, (int) viewportHeight);
		frame.setLocationRelativeTo(null);
		frame.setLayout(new BorderLayout());
		JBulletTest2 main = new JBulletTest2();
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
		frame.setExtendedState(frame.getExtendedState() | JFrame.MAXIMIZED_BOTH);

		initScene();

		// Szimuláció futtatása.
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
							world.stepSimulation(1 / physicsFPS, 10);
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

		// Jelenet kirajzolása.
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
		
		if (SwingUtilities.isLeftMouseButton(e)) {
			//System.out.println("left press");
			Vector3d p = viewMatrix.invert(new Matrix4d()).transformPosition(new Vector3d(e.getPoint().x, e.getPoint().y, 0));

			RayResultCallback result = new RayResultCallback() {
				@Override
				public float addSingleResult(LocalRayResult rayResult, boolean normalInWorldSpace) {
					MyRigidBody body = (MyRigidBody) rayResult.collisionObject;
					Transform tr = new Transform();
					body.getCenterOfMassTransform(tr);
					tr.inverse();
					Vector3f localPivot = new Vector3f((float)p.x, (float)p.y, 0);
					tr.transform(localPivot);
					//System.out.println(localPivot);
					
					if (grabConstraint == null) {
						
						body.setActivationState(RigidBody.DISABLE_DEACTIVATION);
						
						grabBody = addRigidBody(MyRigidBody.createRigidBody(0, 0, 0, 0, null));
						fixTo2DPlane(grabBody);
						world.addRigidBody(grabBody);
						MotionState motionState = MotionStateUtil.createDefaultMotionState((float)p.x, (float)p.y, 0);
						grabBody.setMotionState(motionState);
						
						Transform grabBodyTr = new Transform();
						grabBodyTr.setIdentity();
						Transform bodyTr = new Transform();
						bodyTr.setIdentity();
						bodyTr.origin.x = localPivot.x;
						bodyTr.origin.y = localPivot.y;
						grabConstraint = new Generic6DofConstraint(grabBody, body, grabBodyTr, bodyTr, false);
						world.addConstraint(grabConstraint);
					}

					return 0f;
				}
			};
			
			world.rayTest(new Vector3f((float)p.x, (float)p.y, -1000), new Vector3f((float)p.x, (float)p.y, 1000), result);
		}
	}

	@Override
	public void mouseReleased(MouseEvent e) {
		
		if (SwingUtilities.isRightMouseButton(e)) {
			float m = 10;
			Vector3d p = viewMatrix.invert(new Matrix4d()).transformPosition(new Vector3d(e.getPoint().x, e.getPoint().y, 0));
			MyRigidBody body = addRigidBody(MyRigidBody.createRigidBody((float)p.x, (float)p.y, (float) ThreadLocalRandom.current().nextDouble(0, Math.PI * 2), m, CollisionShapesUtil.createBoxShape(1.4, 1.4)));
			body.setRestitution(.1f);
			body.setFriction(.9f);
		}
		
		if (SwingUtilities.isMiddleMouseButton(e)) {
			dragFrom = null;
			cameraPosition.x += drag.x;
			cameraPosition.y += drag.y;
			drag.set(0, 0, 0);
		}
		
		if (SwingUtilities.isLeftMouseButton(e)) {
			if (grabConstraint != null) {
				world.removeConstraint(grabConstraint);
				grabConstraint = null;
				world.removeRigidBody(grabBody);
				grabBody = null;
			}
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
		
		if (SwingUtilities.isLeftMouseButton(e)) {
			if (grabBody != null) {
				Vector3d p = viewMatrix.invert(new Matrix4d()).transformPosition(new Vector3d(e.getPoint().x, e.getPoint().y, 0));
				MotionState motionState = MotionStateUtil.createDefaultMotionState((float)p.x, (float)p.y, 0);
				grabBody.setMotionState(motionState);
			}
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
			} else {
			}
			break;
		case KeyEvent.VK_Q:
			for (int i = 0; i < 400; i++) {
				if (e.isShiftDown()) {
				} else {
				}
			}
			break;
		case KeyEvent.VK_1:
			initScene(1);
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
