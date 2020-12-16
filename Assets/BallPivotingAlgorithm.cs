using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using renge_pcl;
using renge_pcl.octree;
using UnityEngine.UI;

public enum PointMeshType {
	Sphere
}

[RequireComponent(typeof(MeshFilter))]
public class BallPivotingAlgorithm : MonoBehaviour {
	Front f;
	PointCloud<PointNormal> cloud;
	float ballRadius = 3;
	List<Triangle> preMesh;
	Pivoter pivoter;
	Mesh mesh;
	MeshFilter meshFilter;
	float startTime;
	public Text text;
	bool running;
	int pivotsPerUpdate = 1;
	int pivotAnimationSteps = 5;
	bool pivotingInAction = false;
	int currentPivotStepNum = 0;
	public GameObject ball;

	private void Awake() {
		MeshRenderer rend = GetComponent<MeshRenderer>();
		rend.sharedMaterial = new Material(Shader.Find("Standard"));
		preMesh = new List<Triangle>();
		mesh = new Mesh();
		meshFilter = GetComponent<MeshFilter>();
		running = false;
	}

	private void OnDrawGizmos() {
		if (cloud != null) {
			for (int i = 0; i < cloud.Count; i++) {
				Gizmos.DrawLine(cloud[i].AsVector3(), cloud[i].AsVector3() + cloud[i].NormalAsVector3(0.2f));
			}
		}
	}

	private void Update() {
		if (running) {
			if (StepBallPivot()) {
				MakeStepMesh();
			}
		}
	}

	public PointCloud<PointNormal> GetPointCloud() {
		return cloud;
	}

	public Mesh GetMesh() {
		return mesh;
	}

	public void RunInUpdate(bool fromShape, PointMeshType shape, int pivotsPerUpdate, int pivotAnimationSteps, int numPoints, float scale, float radius) {
		this.pivotsPerUpdate = pivotsPerUpdate;
		this.pivotAnimationSteps = pivotAnimationSteps;
		running = true;
		ballRadius = radius;
		mesh = meshFilter.mesh;
		ball.transform.localScale = new Vector3(radius * 2, radius * 2, radius * 2);
		//generate a sphere of points for testing purposes

		if (fromShape) {
			cloud = new PointCloud<PointNormal>(numPoints);

			if (shape == PointMeshType.Sphere) {
				for (int i = 0; i < numPoints; i++) {
					var normal = new Vector3(UnityEngine.Random.value - 0.5f, UnityEngine.Random.value - 0.5f, UnityEngine.Random.value - 0.5f).normalized;
					var point = normal * scale;
					cloud.Add(new PointNormal(point.x, point.y, point.z, normal.x, normal.y, normal.z));
				}
			}
		} else {
			cloud = new PointCloud<PointNormal>(mesh.vertexCount);
			var vertices = mesh.vertices;
			var normals = mesh.normals;
			for (int i = 0; i < mesh.vertexCount; i++) {
				Vector3 v = vertices[i];
				Vector3 n = normals[i];
				cloud.Add(new PointNormal(v.x, v.y, v.z, n.x, n.y, n.z));
			}
		}

		GetComponent<VoxelRenderer>().SetFromPointCloud(cloud);
		pivoter = new Pivoter(cloud, ballRadius);
		f = new Front();
	}

	public void Run(int numPoints, float scale, float[] passes) {
		startTime = Time.realtimeSinceStartup;

		mesh = meshFilter.mesh;
		//generate a sphere of points for testing purposes

		cloud = new PointCloud<PointNormal>(mesh.vertexCount);
		//cloud = new PointCloud<PointNormal>(numPoints);
		var vertices = mesh.vertices;
		var normals = mesh.normals;
		for (int i = 0; i < mesh.vertexCount; i++) {
			Vector3 v = vertices[i];
			Vector3 n = normals[i];
			cloud.Add(new PointNormal(v.x, v.y, v.z, n.x, n.y, n.z));
		}

		//for (int i = 0; i < numPoints; i++) {
		//	var normal = new Vector3(UnityEngine.Random.value - 0.5f, UnityEngine.Random.value - 0.5f, UnityEngine.Random.value - 0.5f).normalized;
		//	var point = normal * scale;
		//	cloud.Add(new PointNormal(point.x, point.y, point.z, normal.x, normal.y, normal.z));
		//}

		float initTime = (Time.realtimeSinceStartup - startTime);
		Debug.Log("Point Cloud loaded in: " + initTime + "s");
		startTime = Time.realtimeSinceStartup;

		GetComponent<VoxelRenderer>().SetFromPointCloud(cloud);

		float voxTime = (Time.realtimeSinceStartup - startTime);
		Debug.Log("Renderer initialized in: " + voxTime + "s");
		startTime = Time.realtimeSinceStartup;

		RunBallPivot(passes);
		float triangTime = (Time.realtimeSinceStartup - startTime);
		Debug.Log("Triangulation completed in: " + triangTime + "s");
		Debug.Log("Total Searches: " + pivoter.totalSearches);

		startTime = Time.realtimeSinceStartup;
		MakeMesh();
		float meshTime = Time.realtimeSinceStartup - startTime;
		Debug.Log("Mesh created in: " + meshTime + "s");
		Debug.Log("Tris:" + preMesh.Count);

		text.text = "Point Cloud initialized in: " + initTime + "s\n" +
					"Triangulation completed in: " + triangTime + "s\n" +
					"Mesh created in: " + meshTime + "s";
	}

	Triangle toAdd;
	Vector3 oldPos;
	Vector3 newPos;
	float pivotedAngle;
	Edge pivotEdge;

	bool StepBallPivot() {
		bool updated = false;
		Edge e;
		int i = 0;


		if (pivotingInAction) {
			if(currentPivotStepNum < pivotAnimationSteps) {
				currentPivotStepNum++;
				if (currentPivotStepNum == pivotAnimationSteps) {
					pivotingInAction = false;
					currentPivotStepNum = 0;
					preMesh.Add(toAdd);
					updated = true;
					ball.transform.position = newPos;
				} else {
					//ball.transform.position = oldPos + (newPos - oldPos) * (1.0f / pivotAnimationSteps) * currentPivotStepNum;
					ball.transform.RotateAround(pivotEdge.MiddlePoint, (pivotEdge.Second.Item1 - pivotEdge.MiddlePoint).normalized, Mathf.Rad2Deg * pivotedAngle);
				}
			}
		} else {
			for (i = 0; i < pivotsPerUpdate && (e = f.GetActiveEdge()) != null; i++) {
				Tuple<int, Triangle> t = pivoter.Pivot(e);
				if (t != null && (!pivoter.IsUsed(t.Item1) || f.InFront(t.Item1))) {
					if (pivotsPerUpdate == 1) {
						pivotingInAction = true;
						toAdd = t.Item2;
						oldPos = e.BallCenter;
						newPos = t.Item2.BallCenter;
						ball.transform.position = oldPos;
						//pivotedAngle = (pivoter.PivotedAngle) * (1.0f / pivotAnimationSteps);
						Vector3 a = (oldPos - e.MiddlePoint);
						Vector3 b = newPos - e.MiddlePoint;
						pivotedAngle = Mathf.Acos(a.Dot(b) / (a.magnitude * b.magnitude)) * (1.0f / pivotAnimationSteps);
						pivotEdge = e;
					} else {
						preMesh.Add(t.Item2);
						updated = true;
					}
					f.JoinAndGlue(t, pivoter);
				} else {
					f.SetInactive(e);
				}
			}

			if (i == 0) {
				Triangle tri;
				if ((tri = pivoter.FindSeed()) != null) {
					preMesh.Add(tri);
					updated = true;
					ball.transform.position = tri.BallCenter;
					f.AddEdges(tri);
				} else {
					running = false;
				}
			}
		}
		return updated;
	}

	void RunBallPivot(float[] passes) {
		
		startTime = Time.realtimeSinceStartup;
		f = new Front();

		ballRadius = passes[0];
		pivoter = new Pivoter(cloud, ballRadius);
		Debug.Log("Pivoter initialized in: " + (Time.realtimeSinceStartup - startTime) + "s");

		while (true) {
			Edge e;
			while ((e = f.GetActiveEdge()) != null) {
				Tuple<int, Triangle> t = pivoter.Pivot(e);
				if (t != null && (!pivoter.IsUsed(t.Item1) || f.InFront(t.Item1))) {
					preMesh.Add(t.Item2);
					f.JoinAndGlue(t, pivoter);
				} else {
					f.SetInactive(e);
				}
			}

			Triangle tri;
			if ((tri = pivoter.FindSeed()) != null) {
				preMesh.Add(tri);
				f.AddEdges(tri);
			} else {
				pivoter.FindSeed();
				break;
			}
		}
	}

	void MakeMesh() {
		if (meshFilter == null) meshFilter = GetComponent<MeshFilter>();
		if (mesh == null) mesh = new Mesh();

		Vector3[] vertices = new Vector3[cloud.Count];
		int[] tris = new int[preMesh.Count * 3];

		for (int i = 0; i < vertices.Length; i++) {
			vertices[i] = cloud[i].AsVector3();
		}
		for (int i = 0; i < tris.Length - 2; i += 3) {
			tris[i] = preMesh[i / 3].First.Item2;
			tris[i + 1] = preMesh[i / 3].Second.Item2;
			tris[i + 2] = preMesh[i / 3].Third.Item2;
		}

		mesh.Clear();
		mesh.vertices = vertices;
		mesh.triangles = tris;
		mesh.RecalculateNormals();

		meshFilter.mesh = mesh;
	}

	Vector3[] vertices;
	List<int> tris;
	//what are you doing step-mesh??
	void MakeStepMesh() {
		if (meshFilter == null) meshFilter = GetComponent<MeshFilter>();
		if (mesh == null) mesh = new Mesh();
		if (vertices == null) {
			vertices = new Vector3[cloud.Count];
			for (int i = 0; i < vertices.Length; i++) {
				vertices[i] = cloud[i].AsVector3();
			}
		}
		if (tris == null) tris = new List<int>();
		for (int i = 0; i < (preMesh.Count * 3) - 2; i += 3) {
			tris.Add(preMesh[i / 3].First.Item2);
			tris.Add(preMesh[i / 3].Second.Item2);
			tris.Add(preMesh[i / 3].Third.Item2);
		}

		mesh.Clear();
		mesh.vertices = vertices;
		mesh.triangles = tris.ToArray();
		mesh.RecalculateNormals();

		meshFilter.mesh = mesh;

		preMesh.Clear();
	}
}

class Front {
	LinkedList<Edge> front;
	LinkedListNode<Edge> pos;
	SortedDictionary<int, Dictionary<Edge, LinkedListNode<Edge>>> points;

	public Front() {
		points = new SortedDictionary<int, Dictionary<Edge, LinkedListNode<Edge>>>();
		front = new LinkedList<Edge>();
		pos = front.First;
	}

	internal Edge GetActiveEdge() {
		Edge e = null;
		if (front != null && front.Count > 0) {
			bool firstLoop = true;
			for (LinkedListNode<Edge> it = pos; ; it = it.Next) {
				if (it == null) {
					it = front.First;
				}
				if (!firstLoop && it == pos) {
					break;
				}
				if (it.Value.Active) {
					pos = it;
					e = it.Value;
					break;
				}
			}
		}

		return e;
	}

	internal void AddEdges(Triangle tri) {
		for (int i = 0; i < 3; i++) {
			front.AddLast(tri.GetEdge(i));
			AddEdgePoints(front.Last);
		}
	}

	internal bool InFront(int index) {
		return points.ContainsKey(index);
	}

	internal void JoinAndGlue(Tuple<int, Triangle> tri, Pivoter pivoter) {
		//join and glue prototype
		//if (f.Contains(new Edge(e.First, p)))
		//	Glue(new Edge(p, e.First), new Edge(e.First, p));
		//if (f.Contains(new Edge(e.Second, p)))
		//	Glue(new Edge(p, e.First), new Edge(p, e.First));

		if (!pivoter.IsUsed(tri.Item1)) {
			for (int i = 0; i < 2; i++) {
				Edge e = tri.Item2.GetEdge(i);
				LinkedListNode<Edge> insertionPlace = front.AddBefore(pos, e);
				AddEdgePoints(insertionPlace);
			}

			RemoveEdgePoints(pos.Value);

			bool atEnd = false;
			var tmp = pos.Next;
			if (tmp == null) {
				tmp = pos.Previous;
				atEnd = true;
			}
			front.Remove(pos);
			//move iterator to first added edge
			if (!atEnd)
				pos = tmp.Previous.Previous;
			else
				pos = tmp.Previous;


			pivoter.SetUsed(tri.Item1);
		} else if (InFront(tri.Item1)) {
			int added = 0;
			for (int i = 0; i < 2; i++) {
				Edge e = tri.Item2.GetEdge(i);
				LinkedListNode<Edge> it = IsPresent(e);
				if (it != null) {
					RemoveEdgePoints(it.Value);
					front.Remove(it);
				} else {
					LinkedListNode<Edge> insertionPlace = front.AddBefore(pos, e);
					AddEdgePoints(insertionPlace);
					added--;
				}
			}

			var tmp = pos.Next;
			if (tmp == null) {
				tmp = pos.Previous;
				added++;
			}
			RemoveEdgePoints(pos.Value);
			front.Remove(pos);
			pos = tmp;

			if (added < 0) {
				while (added < 0) {
					pos = pos.Previous;
					added++;
				}
			} else {
				pos = front.First;
			}


		} else {
			SetInactive(pos.Value);
		}
	}

	internal void SetInactive(Edge e) {
		e.Active = false;
		RemoveEdgePoints(e);
		if (front.First == pos) {
			front.Remove(pos);
			pos = front.First;
		} else {
			var tmp = pos.Previous;
			front.Remove(pos);
			pos = tmp;
		}
	}

	private LinkedListNode<Edge> IsPresent(Edge e) {
		int vertex0 = e.First.Item2;
		int vertex1 = e.Second.Item2;

		if (!points.ContainsKey(vertex0) || !points.ContainsKey(vertex1)) {
			return null;
		} else {
			foreach (var pair in points[vertex0]) {
				int v0 = pair.Value.Value.First.Item2;
				int v1 = pair.Value.Value.Second.Item2;
				if ((v0 == vertex1 && v1 == vertex0) || (v0 == vertex0 && v1 == vertex1)) {
					return pair.Value;
				}
			}
		}

		return null;
	}

	private void AddEdgePoints(LinkedListNode<Edge> edge) {
		//add first vertex
		Tuple<PointNormal, int> data = edge.Value.First;
		if (!points.ContainsKey(data.Item2)) {
			points[data.Item2] = new Dictionary<Edge, LinkedListNode<Edge>>();
		}
		points[data.Item2][edge.Value] = edge;

		//add second vertex
		data = edge.Value.Second;
		if (!points.ContainsKey(data.Item2)) {
			points[data.Item2] = new Dictionary<Edge, LinkedListNode<Edge>>();
		}
		points[data.Item2][edge.Value] = edge;
	}

	private void RemoveEdgePoints(Edge edge) {
		//remove first vertex
		Tuple<PointNormal, int> data = edge.First;
		if (points.ContainsKey(data.Item2)) {
			points[data.Item2].Remove(edge);

			if (points[data.Item2].Count == 0) {
				points.Remove(data.Item2);
			}
		}

		//remove second vertex
		data = edge.Second;
		if (points.ContainsKey(data.Item2)) {
			points[data.Item2].Remove(edge);

			if (points[data.Item2].Count == 0) {
				points.Remove(data.Item2);
			}
		}
	}
}

class Pivoter {
	//KDTree<PointNormal> kdtree;
	//OcTree<PointNormal> octree;
	VoxelGrid<PointNormal> vgrid;
	PointCloud<PointNormal> cloud;
	float ballRadius;
	SortedDictionary<int, bool> notUsed;
	public int totalSearches;

	public Pivoter(PointCloud<PointNormal> cloud, float ballRadius) {
		this.cloud = cloud;
		totalSearches = 0;
		//kdtree = new KDTree<PointNormal>();
		//kdtree.SetInputCloud(cloud);
		//octree = new OcTree<PointNormal>();
		//octree.SetInputCloud(cloud, 80);
		SetBallRadius(ballRadius);
		notUsed = new SortedDictionary<int, bool>();

		for (int i = 0; i < cloud.Count; i++) {
			notUsed[i] = false;
		}
	}

	public void SetBallRadius(float radius) {
		if (radius == 0 || cloud == null) return;
		this.ballRadius = radius;
		vgrid = new VoxelGrid<PointNormal>(cloud, ballRadius);
	}

	internal Tuple<int, Triangle> Pivot(Edge e) {
		Tuple<PointNormal, int> v0 = e.First;
		Tuple<PointNormal, int> v1 = e.Second;
		Tuple<PointNormal, int> op = e.OppositeVertex;

		//Vector3 diff1 = 100 * (v0.Item1.AsVector3() - middle);
		//Vector3 diff2 = 100 * (e.BallCenter.AsVector3() - middle);

		//Vector3 y = Vector3.Cross(diff1, diff2).normalized;
		//Vector3 normal = Vector3.Cross(diff2, y).normalized;
		Vector3 normal = (v0.Item1 - e.MiddlePoint).normalized;
		HyperPlane plane = new HyperPlane(normal, e.MiddlePoint);

		Vector3 zeroAngle = op.Item1 - e.MiddlePoint;
		zeroAngle = plane.Projection(zeroAngle).normalized;

		float currentAngle = Mathf.PI;
		Tuple<int, Triangle> output = null;

		int[] indices = GetNeighbors(e.MiddlePoint, ballRadius * 2).ToArray();
		for (int t = 0; t < indices.Length; t++) {
			int index = indices[t];
			if (v0.Item2 == index || v1.Item2 == index || op.Item2 == index)
				continue;

			PointNormal point = cloud[index];
			if (plane.AbsDistance(point) <= ballRadius) {
				Vector3 center;
				if (GetBallCenter(v0.Item2, v1.Item2, index, out center, out _)) {
					List<int> neighborhood = GetNeighbors(center, ballRadius);
					if (!isEmpty(neighborhood, v0.Item2, v1.Item2, index, center))
						continue;

					Vector3 Vij = v1.Item1.AsVector3() - v0.Item1.AsVector3();
					Vector3 Vik = point - v0.Item1.AsVector3();
					Vector3 faceNormal = Vector3.Cross(Vik, Vij).normalized;

					if (!IsOriented(faceNormal, v0.Item1, v1.Item1, cloud[index]))
						continue;

					float cosAngle = zeroAngle.Dot(plane.Projection(center).normalized);
					if (Mathf.Abs(cosAngle) > 1.0f) {
						cosAngle = Mathf.Sign(cosAngle);
					}

					float angle = Mathf.Acos(cosAngle);

					if (output == null || currentAngle > angle) {
						currentAngle = angle;
						output = new Tuple<int, Triangle>(index, new Triangle(v0.Item1, cloud[index], v1.Item1, v0.Item2, index, v1.Item2, center, ballRadius));
					}
				}
			}
		}

		return output;
	}

	internal Triangle FindSeed() {
		float neighborhoodSize = 1.3f;

		Triangle seed = null;
		bool found = false;
		SortedDictionary<ulong, bool> tested = new SortedDictionary<ulong, bool>();
		List<int> removeIndices = new List<int>();

		foreach (KeyValuePair<int, bool> pair in notUsed) {
			if (found) break;

			int index0 = pair.Key;
			if (removeIndices.Contains(index0)) continue;

			int[] indices = GetNeighbors(cloud[index0].AsVector3(), ballRadius * neighborhoodSize).ToArray();
			if (indices.Length < 3)
				continue;

			for (int j = 0; j < indices.Length && !found; j++) {
				int index1 = indices[j];

				if (index1 == index0 || !notUsed.ContainsKey(index1) || removeIndices.Contains(index1))
					continue;

				for (int k = 0; k < indices.Length && !found; k++) {
					int index2 = indices[k];

					if (index1 == index2 || index2 == index0 || !notUsed.ContainsKey(index2) || removeIndices.Contains(index2))
						continue;

					List<int> trio = new List<int>();
					trio.Add(index0);
					trio.Add(index1);
					trio.Add(index2);
					trio.Sort();
					ulong code = Convert.ToUInt64(trio[0]) + Convert.ToUInt64(1e6 * trio[1]) + Convert.ToUInt64(1e12 * trio[2]);

					bool toContinue = false;

					if (tested.TryGetValue(code, out _)) toContinue = true;
					else tested[code] = true;

					if (toContinue) continue;

					Vector3 center;
					Vector3Int sequence;

					if (!found && GetBallCenter(index0, index1, index2, out center, out sequence)) {
						List<int> neighborhood = GetNeighbors(center, ballRadius);
						if (!found && isEmpty(neighborhood, index0, index1, index2, center)) {

							seed = new Triangle(cloud[sequence[0]], cloud[sequence[1]], cloud[sequence[2]], sequence[0], sequence[1], sequence[2], center, ballRadius);

							removeIndices.Add(index0);
							removeIndices.Add(index1);
							removeIndices.Add(index2);

							found = true;
						}
					}
				}
			}
		}
		foreach (var index in removeIndices) {
			notUsed.Remove(index);
		}

		return seed;
	}

	internal PointNormal GetPoint(int index) {
		return cloud[index];
	}

	internal bool IsUsed(int index) {
		return !notUsed.TryGetValue(index, out _);
	}

	internal void SetUsed(int index) {
		notUsed.Remove(index);
	}

	Tuple<Vector3, float> GetCircumscribedCircle(Point p0, Point p1, Point p2) {
		Vector3 d10 = p1 - p0;
		Vector3 d20 = p2 - p0;
		Vector3 d01 = p0 - p1;
		Vector3 d12 = p1 - p2;
		Vector3 d21 = p2 - p1;
		Vector3 d02 = p0 - p2;

		float mag01 = d01.magnitude;
		float mag12 = d12.magnitude;
		float mag02 = d02.magnitude;

		float mag01C12 = Vector3.Cross(d01, d12).magnitude;

		float alpha = (mag12 * mag12 * d01.Dot(d02)) / (2 * mag01C12 * mag01C12);
		float beta = (mag02 * mag02 * d10.Dot(d12)) / (2 * mag01C12 * mag01C12);
		float gamma = (mag01 * mag01 * d20.Dot(d21)) / (2 * mag01C12 * mag01C12);

		Vector3 circumscribedCircleCenter = alpha * p0 + beta * p1 + gamma * p2;
		float circumscribedCircleRadius = (mag01 * mag12 * mag02) / (2 * mag01C12);

		return new Tuple<Vector3, float>(circumscribedCircleCenter, circumscribedCircleRadius);
	}

	bool GetBallCenter(int index0, int index1, int index2, out Vector3 center, out Vector3Int sequence) {
		bool status = false;
		center = new Vector3();

		Point p0 = cloud[index0];
		Point p1 = cloud[index1];
		Point p2 = cloud[index2];
		sequence = new Vector3Int(index0, index1, index2);

		Vector3 v10 = p1 - p0;
		Vector3 v20 = p2 - p0;
		Vector3 normal = Vector3.Cross(v10, v20);

		if (normal.magnitude > 0.0000000001) {
			normal.Normalize();
			if (!IsOriented(normal, cloud[index0], cloud[index1], cloud[index2])) {
				p0 = cloud[index1];
				p1 = cloud[index0];
				sequence = new Vector3Int(index1, index0, index2);

				v10 = p1 - p0;
				v20 = p2 - p0;
				normal = Vector3.Cross(v10, v20).normalized;
			}

			Tuple<Vector3, float> circle = GetCircumscribedCircle(p0, p1, p2);
			float squaredDistance = ballRadius * ballRadius - circle.Item2 * circle.Item2;
			if (squaredDistance > 0) {
				float distance = Mathf.Sqrt(Mathf.Abs(squaredDistance));
				center = circle.Item1 + distance * normal;
				status = true;
			}
		}

		return status;
	}

	bool IsOriented(Vector3 normal, PointNormal normal0, PointNormal normal1, PointNormal normal2) {
		int count = 0;
		count = normal.NormalDot(normal0) < 0 ? count + 1 : count;
		count = normal.NormalDot(normal1) < 0 ? count + 1 : count;
		count = normal.NormalDot(normal2) < 0 ? count + 1 : count;
		return count <= 1;
	}

	bool isEmpty(List<int> data, int index0, int index1, int index2, in Vector3 ballCenter) {
		if (data == null || data.Count <= 0)
			return true;

		for (int i = 0; i < data.Count; i++) {
			if (data[i] == index0 || data[i] == index1 || data[i] == index2)
				continue;
			Vector3 dist = cloud[data[i]] - ballCenter;
			if (Mathf.Abs(dist.magnitude - ballRadius) < 0.0000001)
				continue;

			return false;
		}

		return true;
	}

	List<int> GetNeighbors(Vector3 point, float radius) {
		List<int> indices;
		//kdtree.RadiusSearch(point, radius, out indices, out _);
		totalSearches++;
		//octree.RadiusSearch(point, radius, out indices);
		vgrid.RadiusSearch(point, radius, out indices);
		return indices;
	}
}