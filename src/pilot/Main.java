package pilot;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.RenderingHints;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

import org.joml.Matrix4d;
import org.joml.Vector2d;
import org.joml.Vector3d;

import artisynth.core.materials.LinearAxialMaterial;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemSolver;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.ParticlePlaneConstraint;
import artisynth.core.modelbase.StepAdjustment;
import maspack.matrix.Point3d;

public class Main extends JPanel implements MouseWheelListener, MouseListener, MouseMotionListener, ComponentListener {

	private static final long serialVersionUID = -730734892177645353L;
	private Point dragFrom = null;
	private static Vector3d drag = new Vector3d();
	private static Matrix4d viewMatrix = new Matrix4d();
	private static double cameraZoom = 1d;
	private static Vector2d cameraPosition = new Vector2d();
	private static double viewportWidth = 1200;
	private static double viewportHeight = 800;
	
	private static final MechModel mech = new MechModel("mech");
	private static final MechSystemSolver solver = new MechSystemSolver(mech);
	
	public Main() {
		addMouseWheelListener(this);
		addMouseListener(this);
		addMouseMotionListener(this);
		addComponentListener(this);
	}
	
	private static void initScene() {
		
		solver.setIntegrator(Integrator.ConstrainedBackwardEuler);
		
		mech.setGravity (0, 0, -9.8);
		Particle p1 = new Particle("p1", 2, 0, 0, 0);
		Particle p2 = new Particle("p2",.1, -.5, 0, 0);
		AxialSpring spring = new AxialSpring("spr", 0);
		spring.setPoints(p1, p2);
		spring.setMaterial(new LinearAxialMaterial(30, 10));
		mech.addParticle(p1);
		mech.addParticle(p2);
		mech.addAxialSpring(spring);
		p1.setDynamic(false);
		
		Particle p3 = new Particle("p3", 2, -1, 0, 0);
		AxialSpring spring2 = new AxialSpring("spr2", 0);
		spring2.setPoints(p2, p3);
		spring2.setMaterial(new LinearAxialMaterial(30, 10));
		mech.addParticle(p3);
		mech.addAxialSpring(spring2);
		p3.setDynamic(false);
		
		Particle p4 = new Particle("p4", 2, -1, 0, -.5);
		AxialSpring spring3 = new AxialSpring("spr3", 0);
		spring3.setPoints(p2, p4);
		spring3.setMaterial(new LinearAxialMaterial(30, 10));
		mech.addParticle(p4);
		mech.addAxialSpring(spring3);
		
		mech.setBounds(-1, 0, -1, 1, 0, 0);
		
		ParticlePlaneConstraint planecont = new ParticlePlaneConstraint(new maspack.matrix.Vector3d(0, 1, 0), new Point3d());
		planecont.addParticle(p1);
		planecont.addParticle(p2);
		planecont.addParticle(p3);
		planecont.addParticle(p4);
		mech.addConstrainer(planecont);
	}
	
	public static void main(String[] args) {

		JFrame frame = new JFrame("Physics");
		frame.setSize((int)viewportWidth, (int)viewportHeight);
		frame.setLocationRelativeTo(null);
		frame.setLayout(new BorderLayout());
		frame.add(new Main(), BorderLayout.CENTER);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.addWindowListener(new WindowAdapter() {
			@Override
			public void windowOpened(WindowEvent e) {
				updateViewMatrix();
            }
		});
		frame.setVisible(true);
		
		initScene();

		// Szimuláció futtatása.
		new Thread(() -> {
			
			long prevTime = System.nanoTime();
			double renderAcc = 0;
			double renderFPS = 20;
			double updateAcc = 0;
			double updateFPS = 40;
			
			try {

				for (;;) {

					long currentTime = System.nanoTime();
					double dt = (double) (System.nanoTime() - prevTime) / 1000000000d;
					prevTime = currentTime;

					updateAcc += dt;
					if (updateAcc >= 1d / updateFPS) {
						updateAcc = 0;
						solver.solve(0, 1d / updateFPS, new StepAdjustment(1d / updateFPS));
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

	@Override
	protected void paintComponent(Graphics g) {

		super.paintComponent(g);
		Graphics2D g2 = (Graphics2D) g.create();
		g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

		Vector3d origo = viewMatrix.transformPosition(new Vector3d(0, 0, 0));
		Vector3d unitX = viewMatrix.transformPosition(new Vector3d(1, 0, 0));
		Vector3d unitY = viewMatrix.transformPosition(new Vector3d(0, 1, 0));
		g2.setColor(Color.red);
		g2.drawLine((int)origo.x, (int)origo.y, (int)unitX.x, (int)unitX.y);
		g2.setColor(Color.green);
		g2.drawLine((int)origo.x, (int)origo.y, (int)unitY.x, (int)unitY.y);

		// Jelenet kirajzolása.
		g2.setColor(Color.black);
		int s = 4;
		for (Particle p : mech.particles()) {
			Vector3d pt = viewMatrix.transformPosition(new Vector3d(p.getPosition().x, p.getPosition().z, 0));
			g2.drawOval((int)(pt.x-s/2d), (int)(pt.y-s/2d), s, s);
		}
		
		for (AxialSpring sp : mech.axialSprings()) {
			artisynth.core.mechmodels.Point fpt = sp.getFirstPoint();
			artisynth.core.mechmodels.Point spt = sp.getSecondPoint();
			Vector3d pt1 = viewMatrix.transformPosition(new Vector3d(fpt.getPosition().x, fpt.getPosition().z, 0));
			Vector3d pt2 = viewMatrix.transformPosition(new Vector3d(spt.getPosition().x, spt.getPosition().z, 0));
			g2.drawLine((int)pt1.x, (int)pt1.y, (int)pt2.x, (int)pt2.y);
		}
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
}
