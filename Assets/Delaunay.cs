using System;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using System.Linq;

public static class Helper {
	public static float Dot(this Vector4 v, Vector4 other) {
		return v.x * other.x + v.y * other.y + v.z * other.z + v.w * other.w;
	}
	public static float Dot(this Vector3 v, Vector3 other) {
		return v.x * other.x + v.y * other.y + v.z * other.z;
	}

	public static float Dot(this Vector3 v, renge_pcl.Point other) {
		return v.x * other.x + v.y * other.y + v.z * other.z;
	}

	public static float NormalDot(this Vector3 v, renge_pcl.PointNormal other) {
		return v.x * other.nx + v.y * other.ny + v.z * other.nz;
	}
}

public class Matrix3x3 {
	public Vector3 c1, c2, c3;

	public Matrix3x3() {
		c1 = new Vector3();
		c2 = new Vector3();
		c3 = new Vector3();
	}

	public Matrix3x3(Vector3 v1, Vector3 v2, Vector3 v3) {
		this.c1 = v1;
		this.c2 = v2;
		this.c3 = v3;
	}

	public Vector3 MultiplyPoint(Vector3 p) {
		Vector3 res = new Vector3();
		res.x = c1.x * p.x + c2.x * p.y + c3.x * p.z;
		res.y = c1.y * p.x + c2.y * p.y + c3.y * p.z;
		res.z = c1.z * p.x + c2.z * p.y + c3.z * p.z;

		return res;
	}

	public Matrix3x3 Invert() {
		Matrix3x3 res = new Matrix3x3();
		float invdet = 1.0f / Determinant();

		res.Set(0, 0, (Get(1, 1) * Get(2, 2) - Get(2, 1) * Get(1, 2)) * invdet);
		res.Set(0, 1, (Get(0, 2) * Get(2, 1) - Get(0, 1) * Get(2, 2)) * invdet);
		res.Set(0, 2, (Get(0, 1) * Get(1, 2) - Get(0, 2) * Get(1, 1)) * invdet);
		res.Set(1, 0, (Get(1, 2) * Get(2, 0) - Get(1, 0) * Get(2, 2)) * invdet);
		res.Set(1, 1, (Get(0, 0) * Get(2, 2) - Get(0, 2) * Get(2, 0)) * invdet);
		res.Set(1, 2, (Get(1, 0) * Get(0, 2) - Get(0, 0) * Get(1, 2)) * invdet);
		res.Set(2, 0, (Get(1, 0) * Get(2, 1) - Get(2, 0) * Get(1, 1)) * invdet);
		res.Set(2, 1, (Get(2, 0) * Get(0, 1) - Get(0, 0) * Get(2, 1)) * invdet);
		res.Set(2, 2, (Get(0, 0) * Get(1, 1) - Get(1, 0) * Get(0, 1)) * invdet);

		return res;
	}

	public float Determinant() {
		return	Get(0, 0) * (Get(1, 1) * Get(2, 2) - Get(2, 1) * Get(1, 2)) -
				Get(0, 1) * (Get(1, 0) * Get(2, 2) - Get(1, 2) * Get(2, 0)) +
				Get(0, 2) * (Get(1, 0) * Get(2, 1) - Get(1, 1) * Get(2, 0));
	}

	float Get(int x, int y) {
		var row = GetRow(y + 1);
		switch (x) {
			case 0:
				return row.x;
			case 1:
				return row.y;
			case 2:
				return row.z;
			default:
				break;
		}
		return 0;
	}

	void Set(int x, int y, float val) {
		switch (x) {
			case 0:
				switch (y) {
					case 0:
						c1.x = val;
						break;
					case 1:
						c1.y = val;
						break;
					case 2:
						c1.z = val;
						break;
				}
				break;
			case 1:
				switch (y) {
					case 0:
						c2.x = val;
						break;
					case 1:
						c2.y = val;
						break;
					case 2:
						c2.z = val;
						break;
				}
				break;
			case 2:
				switch (y) {
					case 0:
						c3.x = val;
						break;
					case 1:
						c3.y = val;
						break;
					case 2:
						c3.z = val;
						break;
				}
				break;
		}
	}

	Vector3 GetRow(int index) {
		Vector3 row = new Vector3();
		switch (index) {
			case 1:
				row = new Vector3(c1.x, c2.x, c3.x);
				break;
			case 2:
				row = new Vector3(c1.y, c2.y, c3.y);
				break;
			case 3:
				row = new Vector3(c1.z, c2.z, c3.z);
				break;
			default:
				break;
		}
		return row;
	}

	void SetRow(int index, Vector3 row) {
		switch (index) {
			case 1:
				c1.x = row.x;
				c2.x = row.y;
				c3.x = row.z;
				break;
			case 2:
				c1.y = row.x;
				c2.y = row.y;
				c3.y = row.z;
				break;
			case 3:
				c1.z = row.x;
				c2.z = row.y;
				c3.z = row.z;
				break;
			default:
				break;
		}
	}

	public void RowOpAdd(int row1, int row2, float amt) {
		Vector3 r1 = GetRow(row1);
		Vector3 r2 = GetRow(row2);

		r2 += r1 * amt;

		SetRow(row2, r2);
	}

	public void RowOpSub(int row1, int row2, float amt) {
		Vector3 r1 = GetRow(row1);
		Vector3 r2 = GetRow(row2);

		r2 -= r1 * amt;

		SetRow(row2, r2);
	}

	public void RowOpDiv(int row, float amt) {
		Vector3 r1 = GetRow(row);

		r1 /= amt;

		SetRow(row, r1);
	}

	public void RowOpMul(int row, float amt) {
		Vector3 r1 = GetRow(row);

		r1 *= amt;

		SetRow(row, r1);
	}
}

public struct Tetrahedron {
	public int x0, x1, x2, x3;
	public Vector3 Circumcenter { get; private set; }
	public float Circumradius { get; private set; }
	public Tetrahedron(int i0, int i1, int i2, int i3, Vector3 x0, Vector3 x1, Vector3 x2, Vector3 x3) {
		this.x0 = i0;
		this.x1 = i1;
		this.x2 = i2;
		this.x3 = i3;

		Vector3 v1 = x1 - x0;
		Vector3 v2 = x2 - x0;
		Vector3 v3 = x3 - x0;

		Vector3 v1n = new Vector3(v1.x, v2.x, v3.x);
		Vector3 v2n = new Vector3(v1.y, v2.y, v3.y);
		Vector3 v3n = new Vector3(v1.z, v2.z, v3.z);

		Matrix3x3 mA = new Matrix3x3(v1n, v2n, v3n);
		float p1Dot = x0.Dot(x0);
		Vector3 B = new Vector3(x1.Dot(x1) - p1Dot, x2.Dot(x2) - p1Dot, x3.Dot(x3) - p1Dot) * 0.5f;

		Circumcenter = mA.Invert().MultiplyPoint(B);
		Circumradius = Vector3.Distance(Circumcenter, x0);
	}
}

[RequireComponent(typeof(MeshFilter))]
public class Delaunay : MonoBehaviour {

	Mesh mesh;
	List<Mesh> gizmoMeshes = new List<Mesh>();
	Vector3 max, min;
	Vector3 minMaxDelta;
	Vector3 minMaxAvg;
	float deltaMax;
	Vector3[] points = new Vector3[0];
	List<Vector3> availablePoints = new List<Vector3>();
	Tetrahedron superTetra;
	List<Tetrahedron> tetras = new List<Tetrahedron>();

	void Start() {
		mesh = new Mesh();
		GetComponent<MeshFilter>().mesh = mesh;

		gizmoMeshes = new List<Mesh>();

		points = new Vector3[10];

		for (int i = 0; i < points.Length; i++) {
			var point = new Vector3(UnityEngine.Random.value - 0.5f, UnityEngine.Random.value - 0.5f, UnityEngine.Random.value - 0.5f).normalized * 10.0f;
			points[i] = point;
		}

		availablePoints = points.ToList();

		//points = new Vector3[]{
		//	new Vector3(1.12f, 1.92f, 5.88f),
		//	new Vector3(3.67f, 1.88f, 4.74f),
		//	new Vector3(5.3f, 0.71f, 3.3f),
		//	new Vector3(4.63f, 3.1f, 2.8f),
		//	new Vector3(2.93f, 3.98f, 3.88f),
		//	new Vector3(4.54f, 4.25f, 0.9f),
		//	new Vector3(1.21f, 5.97f, 1.55f),
		//	new Vector3(5.79f, 2.28f, 1.09f)
		//};

		ComputeDelaunayMesh(points);
	}

	int pointIndex = 0;
	int prevPointIndex = -1;
	int circumIndex = 0;
	bool delaunayTested = false;
	bool isDelaunay = true;
	private void Update() {
		circumIndex++;
		if(circumIndex > 0) {
			pointIndex++;
		}
		if (pointIndex != prevPointIndex && pointIndex < points.Length) {
			prevPointIndex = pointIndex;
			circumIndex = 0;
			AddPoint(points[pointIndex], pointIndex);
			clearGizmoMeshes();
			foreach (var item in tetras) {
				AddGizmoTetra(item);
			}
		}
		if(pointIndex >= points.Length && !delaunayTested) {
			delaunayTested = true;
			foreach (var t in tetras) {
				if (!IsDelaunay(t)) {
					isDelaunay = false;
				}
			}
			string delString = "Delaunay!";
			if (!isDelaunay) delString = "NOT Delaunay!";
			Debug.Log("The triangulation is " + delString);
		}
	}

	void ComputeDelaunayMesh(Vector3[] points) {
		//precompute tetras[0]
		//get the maximum and minimum values in the point cloud for x y and z
		max = new Vector3(float.MinValue, float.MinValue, float.MinValue);
		min = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);

		foreach (var i in points) {
			if (max.x < i.x) {
				max.x = i.x;
			}
			if (max.y < i.y) {
				max.y = i.y;
			}
			if (max.z < i.z) {
				max.z = i.z;
			}

			if (min.x > i.x) {
				min.x = i.x;
			}
			if (min.y > i.y) {
				min.y = i.y;
			}
			if (min.z > i.z) {
				min.z = i.z;
			}
		}

		// get min max difference and save the max value of that
		minMaxDelta = max - min;
		float max1 = minMaxDelta.x > minMaxDelta.y ? minMaxDelta.x : minMaxDelta.y;
		deltaMax = max1 > minMaxDelta.z ? max1 : minMaxDelta.z;

		//calculate average of max and min
		minMaxAvg = (min + max) / 2.0f;

		availablePoints.Add(new Vector3(minMaxAvg.x - 4 * deltaMax, minMaxAvg.y - deltaMax, minMaxAvg.z - deltaMax));
		availablePoints.Add(new Vector3(minMaxAvg.x + 4 * deltaMax, minMaxAvg.y - deltaMax, minMaxAvg.z - deltaMax));
		availablePoints.Add(new Vector3(minMaxAvg.x, minMaxAvg.y + 4 * deltaMax, minMaxAvg.z - deltaMax));
		availablePoints.Add(new Vector3(minMaxAvg.x, minMaxAvg.y, minMaxAvg.z + 4 * deltaMax));

		//calculate super tetrahedron
		superTetra = new Tetrahedron(availablePoints.Count - 4, availablePoints.Count - 3, availablePoints.Count - 2, availablePoints.Count - 1, 
			availablePoints[availablePoints.Count-4], availablePoints[availablePoints.Count - 3], availablePoints[availablePoints.Count - 2], availablePoints[availablePoints.Count - 1]);
		tetras.Add(superTetra);

		

		AddGizmoTetra(tetras[0]);

		Debug.LogFormat("{0}\n{1}\n{2}\n{3}", tetras[0].x0, tetras[0].x1, tetras[0].x2, tetras[0].x3);

		Debug.LogFormat("Centroid: {0}\nRadius1: {1}\nRadius2: {2}\nRadius3: {3}\nRadius4: {4}\n", tetras[0].Circumcenter, tetras[0].Circumradius,
			Vector3.Distance(tetras[0].Circumcenter, availablePoints[tetras[0].x1]), 
			Vector3.Distance(tetras[0].Circumcenter, availablePoints[tetras[0].x2]), 
			Vector3.Distance(tetras[0].Circumcenter, availablePoints[tetras[0].x3]));
	}

	private void AddPoint(Vector3 point, int index) {
		List<Tetrahedron> newTetra = new List<Tetrahedron>();
		HashSet<int> toConnect = new HashSet<int>();
		//find point containing tetras and remove them from the current tetra pool
		foreach (var tetra in tetras) {
			if(IsWithinCircumsphereBounds(point, tetra)) {
				toConnect.Add(tetra.x0);
				toConnect.Add(tetra.x1);
				toConnect.Add(tetra.x2);
				toConnect.Add(tetra.x3);
			} else {
				newTetra.Add(tetra);
			}
		}

		tetras = newTetra;

		int count = toConnect.Count;
		var listToConnect = toConnect.ToList();
		for(int i = 0; i < count - 2; ++i) {
			for (int j = i+1; j < count - 1; j++) {
				for (int k = j+1; k < count; k++) {
					tetras.Add(new Tetrahedron(index, listToConnect[i], listToConnect[j], listToConnect[k], availablePoints[index], 
						availablePoints[listToConnect[i]], availablePoints[listToConnect[j]], availablePoints[listToConnect[k]]));
				}
			}
			
		}
	}

	bool IsDelaunay(Tetrahedron t) {
		for (int i = 0; i < availablePoints.Count; i++) {
			if(i != t.x0 && i != t.x1 && i != t.x2 && i != t.x3) {
				if (IsWithinCircumsphereBounds(availablePoints[i], t))
					return false;
			}
		}
		return true;
	}

	bool IsWithinCircumsphereBounds(Vector3 pos, Tetrahedron tetra) {
		if (Vector3.Distance(pos, tetra.Circumcenter) <= tetra.Circumradius)
			return true;
		return false;
	}

	float CalcAverageVectorArr(Vector3[] points, int index) {
		float sum = 0;
		foreach (var item in points) {
			if (index == 0)
				sum += item.x;
			else if (index == 1)
				sum += item.y;
			else
				sum += item.z;
		}

		return sum / 4.0f;
	}

	void ApplyMesh(Vector3[] points, int[] tris) {
		mesh.Clear();
		mesh.vertices = points;
		mesh.triangles = tris;
		mesh.RecalculateNormals();
	}

	void AddGizmoTetra(Tetrahedron t) {
		Mesh mesh1 = new Mesh();
		int[] tris = {
			1, 3, 0,
			3, 2, 0,
			2, 1, 0,
			2, 3, 1
		};
		mesh1.vertices = new Vector3[] { availablePoints[t.x0], availablePoints[t.x1], availablePoints[t.x2], availablePoints[t.x3]};
		mesh1.triangles = tris;
		mesh1.RecalculateNormals();
		gizmoMeshes.Add(mesh1);
	}

	void clearGizmoMeshes() {
		gizmoMeshes.Clear();
	}

	private void OnDrawGizmos() {
		for (int i = 0; i < points.Length; i++) {
			if (pointIndex == i) {
				Gizmos.color = Color.green;
			} else {
				Gizmos.color = Color.white;
			}
				Gizmos.DrawSphere(points[i], 0.1f);
		}

		foreach (var item in gizmoMeshes) {
			Gizmos.DrawWireMesh(item);
		}

		if (tetras == null || tetras.Count == 0) return;
		Gizmos.DrawWireSphere(tetras[circumIndex % tetras.Count].Circumcenter, tetras[circumIndex % tetras.Count].Circumradius);
		Gizmos.color = Color.cyan;
		Gizmos.DrawSphere(tetras[circumIndex % tetras.Count].Circumcenter, 0.1f);

		Gizmos.color = Color.red;
		Gizmos.DrawSphere(superTetra.Circumcenter, 0.1f);
		Gizmos.DrawWireSphere(superTetra.Circumcenter, superTetra.Circumradius);

		Gizmos.DrawRay(new Vector3(), new Vector3(1, 0, 0));
		Gizmos.color = Color.green;
		Gizmos.DrawRay(new Vector3(), new Vector3(0, 1, 0));
		Gizmos.color = Color.blue;
		Gizmos.DrawRay(new Vector3(), new Vector3(0, 0, 1));
	}
}