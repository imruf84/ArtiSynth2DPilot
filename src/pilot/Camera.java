package pilot;

import java.io.Serializable;

import org.joml.Matrix4d;
import org.joml.Rayd;
import org.joml.Vector3d;
import org.joml.Vector4d;

public class Camera implements Serializable {

	private static final long serialVersionUID = 1L;

	private static double TILE_SIZE_IN_WORLD = 10;;
	
	private double width;
	private double height;
	private static final double HALF_ORTHO_WITH = TILE_SIZE_IN_WORLD ;
	private static final double HALF_ORTHO_HEIGHT = TILE_SIZE_IN_WORLD;
	private static final double CAMERA_PRECISION = 1000d;
	private double translateX;
	private double translateY;
	private double translateZ;
	private double rotateX;
	private double rotateZ;
	private double zoom;
	private final Matrix4d projectionMatrix = new Matrix4d();
	private final Matrix4d invProjectionMatrix = new Matrix4d();
	private final double[] projectionArray = new double[16];
	private final Vector3d eye = new Vector3d();
	private final Vector3d direction = new Vector3d();
	private final int[] viewport = new int[4];
	private double zoomMin;
	private double zoomMax;
	private final Vector4d ortho = new Vector4d();

	public Camera(double width, double height) {
		setViewportSize(width, height);
	}

	public Camera() {
		this(1, 1);
	}

	public final void setViewportSize(double width, double height) {
		this.width = width;
		this.height = height;
	}

	public double getViewportWidth() {
		return width;
	}

	public double getViewportHeight() {
		return height;
	}

	public void setTranslateXYZ(double translateX, double translateY, double translateZ) {
		setTranslateX(translateX);
		setTranslateY(translateY);
		setTranslateZ(translateZ);
	}

	public double getTranslateX() {
		return translateX;
	}

	public void setTranslateX(double translateX) {
		this.translateX = translateX;
	}

	public void setTranslateY(double translateY) {
		this.translateY = translateY;
	}

	public void setTranslateZ(double translateZ) {
		this.translateZ = translateZ;
	}

	public double getTranslateY() {
		return translateY;
	}

	public double getTranslateZ() {
		return translateZ;
	}

	public void setRotateXZ(double rotateX, double rotateZ) {
		setRotateX(rotateX);
		setRotateZ(rotateZ);
	}

	public double getRotateX() {
		return rotateX;
	}

	public void setRotateX(double rotateX) {
		this.rotateX = rotateX;
	}

	public double getRotateZ() {
		return rotateZ;
	}

	public void setRotateZ(double rotateZ) {
		this.rotateZ = rotateZ;
	}

	public double getZoom() {
		return zoom;
	}

	public void setZoom(double zoom) {
		this.zoom = Math.max(getZoomMin(), Math.min(zoom, getZoomMax()));
	}

	public void setZoomLimits(double min, double max) {
		setZoomMin(min);
		setZoomMax(max);
	}

	public double getZoomMin() {
		return zoomMin;
	}

	public double getZoomMax() {
		return zoomMax;
	}

	public void setZoomMin(double zoom) {
		this.zoomMin = zoom;
	}

	public void setZoomMax(double zoom) {
		this.zoomMax = zoom;
	}

	public void dRotateX(double a) {
		setRotateX(getRotateX() + a);
	}

	public void dRotateZ(double a) {
		setRotateZ(getRotateZ() + a);
	}

	public void dRotate(double ax, double az) {
		dRotateX(ax);
		dRotateZ(az);
	}

	public void recalculate() {
		calculateProjectionMatrix(getViewportWidth(), getViewportHeight(), getTranslateX(), getTranslateY(),
				getTranslateZ(), getRotateX(), getRotateZ(), getZoom(), getProjectionMatrix(), getOrtho());
		viewport[2] = (int) getViewportWidth();
		viewport[3] = (int) getViewportHeight();
		getProjectionMatrix().get(projectionArray);
		getProjectionMatrix().invert(getInvProjectionMatrix());
		getInvProjectionMatrix().transformPosition(new Vector3d(), getEye());
		Camera.calculateTransformationMatrix(getRotateX(), getRotateZ(), getZoom()).invert()
				.transformPosition(new Vector3d(0, CAMERA_PRECISION, CAMERA_PRECISION), getDirection());
	}

	public Vector3d getEye() {
		return eye;
	}

	public Vector3d getDirection() {
		return direction;
	}

	public int[] getViewport() {
		return viewport;
	}

	public Matrix4d getInvProjectionMatrix() {
		return invProjectionMatrix;
	}

	public Matrix4d getProjectionMatrix() {
		return projectionMatrix;
	}

	public double[] getProjectionArray() {
		return projectionArray;
	}

	public Vector4d getOrtho() {
		return ortho;
	}

	public Rayd calculateRay(double screenX, double screenY) {
		Vector3d lEye = getEye();
		Vector3d lDir = getDirection().mul(-1).normalize(new Vector3d());
		Vector3d up = new Vector3d(0, 0, 1).normalize();
		Vector3d horizontal = lDir.cross(up, new Vector3d()).normalize();
		horizontal.cross(lDir, up).normalize();

		Vector4d lOrtho = getOrtho();

		Vector3d point = new Vector3d(screenX / getViewportWidth(), -screenY / getViewportHeight(), 0)
				.sub(new Vector3d(.5d, -.5d, 0d));

		Vector3d from = new Vector3d().add(lEye, new Vector3d())
				.add(horizontal.mul(point.x * (lOrtho.y - lOrtho.x) * getZoom(), new Vector3d()))
				.add(up.mul(point.y * (lOrtho.w - lOrtho.z) * getZoom(), new Vector3d()));

		return new Rayd(from, lDir);
	}

	public static Vector3d getOriginFromRay(Rayd ray) {
		return new Vector3d(ray.oX, ray.oY, ray.oZ);
	}

	public static Vector3d getDestinationFromRay(Rayd ray) {
		return new Vector3d(ray.oX + ray.dX, ray.oY + ray.dY, ray.oZ + ray.dZ);
	}

	public static Matrix4d calculateTransformationMatrix(double rx, double rz, double zoom) {
		return new Matrix4d().identity().rotate(Math.toRadians(180), 0, 0, 1).scale(1 / zoom)
				.rotate(Math.toRadians(rx), 1, 0, 0).rotate(Math.toRadians(rz), 0, 0, 1);
	}

	public static Matrix4d calculateProjectionMatrix(double w, double h, double tx, double ty, double tz, double rx,
			double rz, double zoom, Matrix4d dest, Vector4d orthoDest) {

		double lZoom = 1;
		double aspect = w / h;
		double[] ortho;
		if (w > h) {
			ortho = new double[] { -HALF_ORTHO_WITH / lZoom * aspect, HALF_ORTHO_WITH / lZoom * aspect,
					-HALF_ORTHO_HEIGHT / lZoom, HALF_ORTHO_HEIGHT / lZoom };
		} else {
			aspect = 1 / aspect;
			ortho = new double[] { -HALF_ORTHO_WITH / lZoom, HALF_ORTHO_WITH / lZoom,
					-HALF_ORTHO_HEIGHT / lZoom * aspect, HALF_ORTHO_HEIGHT / lZoom * aspect };
		}

		if (orthoDest != null) {
			orthoDest.set(ortho[0], ortho[1], ortho[2], ortho[3]);
		}

		Matrix4d m = (dest == null ? new Matrix4d() : dest);

		return m.identity().ortho(ortho[0], ortho[1], ortho[2], ortho[3], -100, 100).lookAt(0, 1, 1, 0, 0, 0, 0, 0, 1)
				.rotate(Math.toRadians(180), 0, 0, 1).rotate(Math.toRadians(rx), 1, 0, 0)
				.rotate(Math.toRadians(rz), 0, 0, 1).scale(1 / zoom).translate(tx, ty, tz);
	}

	public static Matrix4d calculateProjectionMatrix(double w, double h, double tx, double ty, double tz, double rx,
			double rz, double zoom) {
		return calculateProjectionMatrix(w, h, tx, ty, tz, rx, rz, zoom, null, null);
	}

	public void dTranslateX(double d) {
		setTranslateX(getTranslateX() + d);
	}

	public void dTranslateY(double d) {
		setTranslateY(getTranslateY() + d);
	}

	public void dTranslateZ(double d) {
		setTranslateZ(getTranslateZ() + d);
	}

	public void dTranslate(double dx, double dy, double dz) {
		dTranslateX(dx);
		dTranslateY(dy);
		dTranslateZ(dz);
	}

	public void pan(int dx, int dy) {
		Vector3d p0 = getProjectionMatrix().unproject(0, 0, 0, getViewport(), new Vector3d());
		Vector3d p1 = getProjectionMatrix().unproject(1, 0, 0, getViewport(), new Vector3d());
		Vector3d p2 = getProjectionMatrix().unproject(0, 1, 0, getViewport(), new Vector3d());

		Vector3d dir = getDirection();
		Vector3d up = new Vector3d(0, 0, 1);
		Vector3d u = dir.cross(up, new Vector3d()).normalize();
		Vector3d v = dir.cross(u, new Vector3d()).normalize();

		u.mul(-dx * new Vector3d(p0).sub(p1).length());
		v.mul(dy * new Vector3d(p0).sub(p2).length());

		u.add(v);
		dTranslate(u.x, u.y, u.z);
	}

}