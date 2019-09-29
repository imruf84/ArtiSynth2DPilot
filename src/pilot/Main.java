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

import org.joml.Matrix4d;
import org.joml.Vector2d;
import org.joml.Vector3d;
import org.python.modules.synchronize;

import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.gui.editorManager.EditorUtils;
import artisynth.core.materials.ConstantAxialMaterial;
import artisynth.core.materials.LinearAxialMaterial;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.Collidable;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemSolver;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.ParticlePlaneConstraint;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.StepAdjustment;
import maspack.geometry.Face;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.render.RenderProps;
import maspack.spatialmotion.SpatialInertia;

public class Main extends JPanel implements MouseWheelListener, MouseListener, MouseMotionListener, ComponentListener, KeyListener {

	private static final long serialVersionUID = -730734892177645353L;
	private Point dragFrom = null;
	private static Vector3d drag = new Vector3d();
	private static Matrix4d viewMatrix = new Matrix4d();
	private static double cameraZoom = 1d;
	private static Vector2d cameraPosition = new Vector2d();
	private static double viewportWidth = 1200;
	private static double viewportHeight = 800;
	
	private static double renderFPS = 30;
	private static double physicsFPS = 50;
	
	private static Object sceneLock = new Object();
	private static MechModel scene;
	private static MechSystemSolver solver;
	private static ParticlePlaneConstraint planeConstraint = null;
	
	public Main() {
		addMouseWheelListener(this);
		addMouseListener(this);
		addMouseMotionListener(this);
		addComponentListener(this);
	}
	
	private static void initScene() {

		synchronized (sceneLock) {
			clearScene();
			initScene1();
		}
	}
	
	private static void clearScene() {
		scene = new MechModel("scene");
		solver = new MechSystemSolver(scene);
		scene.setMaxStepSize(1 / physicsFPS);
		//scene.setPenetrationLimit(0);

		// solver.profileKKTSolveTime = true;
		// scene.setProfiling (true);

		solver.setIntegrator(Integrator.ConstrainedBackwardEuler);
		scene.setGravity(0, 0, -9.8);
		scene.setDefaultCollisionBehavior(true, .5);
		CollisionManager collisionManager = scene.getCollisionManager();
		//collisionManager.setAcceleration(5);
		collisionManager.setReduceConstraints(true);
		scene.setBounds(-1, 0, -1, 1, 0, 0);

		planeConstraint = new ParticlePlaneConstraint(new maspack.matrix.Vector3d(0, 1, 0), new Point3d());
		scene.addConstrainer(planeConstraint);

	}
	
	private static RigidBody addBox(double w, double h, double m, double x, double y, double a, double vx, double vy, double va) {
		
		synchronized (sceneLock) {

			RigidBody box = RigidBody.createBox(null, w, 1, h, 0.1);
			box.setMass(m);
			box.setPose(new RigidTransform3d(x, 0, y, 0, 0, a, 0));
			box.setVelocity(vx, 0, vy, 0, va, 0);

			scene.addRigidBody(box);
/*
			Particle p = new Particle(null, .001, 0, 0, 0);
			scene.addParticle(p);
			scene.attachPoint(p, box, new Point3d(p.getPosition()));
			planeConstraint.addParticle(p);

			p = new Particle(null, .001, w / 2, 0, 0);
			scene.addParticle(p);
			scene.attachPoint(p, box, new Point3d(p.getPosition()));
			planeConstraint.addParticle(p);

			p = new Particle(null, .001, 0, 0, h / 2);
			scene.addParticle(p);
			scene.attachPoint(p, box, new Point3d(p.getPosition()));
			planeConstraint.addParticle(p);
*/
			return box;

		}
	}
	
	private static RigidBody addRandomBox() {
		double w = ThreadLocalRandom.current().nextDouble(.1, 4);
		double h = ThreadLocalRandom.current().nextDouble(.1, 4);
		double m = ThreadLocalRandom.current().nextDouble(.01, 40);
		double x = ThreadLocalRandom.current().nextDouble(-1, 1);
		double y = ThreadLocalRandom.current().nextDouble(Math.max(Math.max(w, h)*1.5, 20));
		double a = ThreadLocalRandom.current().nextDouble(-Math.PI, Math.PI);
		double vx = ThreadLocalRandom.current().nextDouble(-4, 4);
		double vy = ThreadLocalRandom.current().nextDouble(-4, 4);
		double va = ThreadLocalRandom.current().nextDouble(-10, 10);
		
		return addBox(w, h, m, x, y, a, vx, vy, va);
	}
	
	private static void initScene1() {
		//addRandomBox();
		
		
		try {
            /*FemModel3d softBox = FemFactory.createFromMesh(null, MeshFactory.createBox (1, 1, 1), 0);
    		//softBox.setSurfaceMesh(MeshFactory.createBox (1, 1, 1));
    		softBox.setLinearMaterial (1000000, 0.33, true);
    		softBox.setParticleDamping (0.1);
    		softBox.transformGeometry (new RigidTransform3d (0, 0, 4));
    		softBox.setIncompressible (FemModel3d.IncompMethod.AUTO);*/
			FemModel3d softBox = FemFactory.createIcosahedralSphere(null,  1,  1,  1);
    		scene.addModel(softBox);
    		//scene.setCollisionResponse (softBox, Collidable.Deformable);
         }
         catch (Exception e) {
            System.err.println(e.getLocalizedMessage());
         }
		
		RigidBody floor = RigidBody.createBox (null, 40, 40, .5, 20);
		floor.setDynamic(false);
		floor.setPose (new RigidTransform3d (0, 0, -.5/2));
		scene.addRigidBody (floor);
	}
	
	public static void main(String[] args) {

		JFrame frame = new JFrame("Physics");
		frame.setSize((int)viewportWidth, (int)viewportHeight);
		frame.setLocationRelativeTo(null);
		frame.setLayout(new BorderLayout());
		Main main = new Main();
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
						synchronized (sceneLock) {
							solver.solve(0, 1d / physicsFPS, null);
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
		viewMatrix.identity().scale(1, -1, 1).translate(viewportWidth / 2, -viewportHeight / 2, 0).scale(Math.min(viewportWidth, viewportHeight)/4);
		viewMatrix.scale(cameraZoom);
		viewMatrix.translate(cameraPosition.x+drag.x,cameraPosition.y+drag.y,0);
	}
	
	private void renderScene(Graphics g2) {
		
		if (scene == null) {
			return;
		}
		
		synchronized (sceneLock) {

			g2.setColor(Color.red);
			int s = 4;
			for (Particle p : scene.particles()) {
				Vector3d pt = viewMatrix.transformPosition(new Vector3d(p.getPosition().x, p.getPosition().z, 0));
				g2.drawOval((int) (pt.x - s / 2d), (int) (pt.y - s / 2d), s, s);
			}

			for (AxialSpring sp : scene.axialSprings()) {
				artisynth.core.mechmodels.Point fpt = sp.getFirstPoint();
				artisynth.core.mechmodels.Point spt = sp.getSecondPoint();
				Vector3d pt1 = viewMatrix.transformPosition(new Vector3d(fpt.getPosition().x, fpt.getPosition().z, 0));
				Vector3d pt2 = viewMatrix.transformPosition(new Vector3d(spt.getPosition().x, spt.getPosition().z, 0));
				g2.drawLine((int) pt1.x, (int) pt1.y, (int) pt2.x, (int) pt2.y);
			}

			g2.setColor(Color.black);
			for (RigidBody body : scene.rigidBodies()) {
				for (PolygonalMesh mesh : body.getSurfaceMeshes()) {
					for (Face face : mesh.getFaces()) {
						Vertex3d[] vertices = face.getTriVertices();
						
						// Csak az elsülsõ lapokat rajzoljuk ki.
						/*if (vertices[0].getWorldPoint().y < 0 || vertices[1].getWorldPoint().y < 0 || vertices[2].getWorldPoint().y < 0) {
							continue;
						}*/
						
						Vector3d pt0 = viewMatrix.transformPosition(
								new Vector3d(vertices[0].getWorldPoint().x, vertices[0].getWorldPoint().z, 0));
						Vector3d pt1 = viewMatrix.transformPosition(
								new Vector3d(vertices[1].getWorldPoint().x, vertices[1].getWorldPoint().z, 0));
						Vector3d pt2 = viewMatrix.transformPosition(
								new Vector3d(vertices[2].getWorldPoint().x, vertices[2].getWorldPoint().z, 0));

						g2.drawLine((int) pt1.x, (int) pt1.y, (int) pt0.x, (int) pt0.y);
						g2.drawLine((int) pt1.x, (int) pt1.y, (int) pt2.x, (int) pt2.y);
						g2.drawLine((int) pt0.x, (int) pt0.y, (int) pt2.x, (int) pt2.y);
					}
				}
			}
		}
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
		g2.drawLine((int)origo.x, (int)origo.y, (int)unitX.x, (int)unitX.y);
		g2.setColor(Color.green);
		g2.drawLine((int)origo.x, (int)origo.y, (int)unitY.x, (int)unitY.y);
	}

	@Override
	public void mouseWheelMoved(MouseWheelEvent e) {
		double scroll = 1-2*e.getPreciseWheelRotation()/10d;
		cameraZoom *= scroll;
		
		Point p = e.getPoint();
		
		Vector3d v0 = new Vector3d(p.x-viewportWidth/2, p.y-viewportHeight/2, 0);
		Matrix4d m = viewMatrix.invert(new Matrix4d());
		Vector3d t0 = m.transformDirection(v0);
		
		updateViewMatrix();
		
		m = viewMatrix.invert(new Matrix4d());
		Vector3d v1 = new Vector3d(p.x-viewportWidth/2, p.y-viewportHeight/2, 0);
		Vector3d t1 = m.transformDirection(v1);
		
		t1.sub(t0);
		cameraPosition.x+=t1.x;
		cameraPosition.y+=t1.y;
		
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
			drag.set(point.getX()-dragFrom.getX(), -(point.getY()-dragFrom.getY()), 0);
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
		// Not implemented.
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
			addRandomBox();
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
