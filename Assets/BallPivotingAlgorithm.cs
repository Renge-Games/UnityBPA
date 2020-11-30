using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using renge_pcl;

public class BallPivotingAlgorithm : MonoBehaviour {
	Front f;
	PointCloud<renge_pcl.PointNormal> cloud;
	float ballRadius;
	List<Triangle> mesh;
	Pivoter pivoter;

	void RunBallPivot() {
		pivoter = new Pivoter(cloud, ballRadius);
		while (true) {
			Edge e;
			while((e = f.GetActiveEdge()) != null) {
				PointNormal p;
				if((p = pivoter.Pivot(e)) != null && (!pivoter.IsUsed(p) || f.OnFront(p))){
					OutputTriangle(p, e.First, e.Second);
					f.JoinAndGlue(e, p, pivoter);
				} else {
					MarkAsBoundary(e);
				}
			}

			Triangle tri;
			if ((tri = FindSeedTriangle()) != null) {
				OutputTriangle(tri.First, tri.Second, tri.Third);
				f.Insert(new Edge(tri.First, tri.Second));
				f.Insert(new Edge(tri.Second, tri.Third));
				f.Insert(new Edge(tri.Third, tri.First));
			} else
				break;
		}
	}

	private Triangle FindSeedTriangle() {
		return null;
	}

	private void MarkAsBoundary(Edge e) {

	}

	private void Glue(Edge edge1, Edge edge2) {
		
	}

	private void OutputTriangle(PointNormal p1, PointNormal p2, PointNormal p3) {
		
	}
}

class Triangle {
	public PointNormal First { get; set; }
	public PointNormal Second { get; set; }
	public PointNormal Third { get; set; }

	public Triangle() {
		First = Second = Third = null;
	}

	public Triangle(PointNormal p0, PointNormal p1, PointNormal p2, int index0, int index1, int index2, PointNormal ballCenter, float ballRadius) {

	}
}

class Edge {
	public PointNormal First { get; set; }
	public PointNormal Second { get; set; }
	public PointNormal OppositeVertex { get; set; }
	
	public PointNormal BallCenter { get; private set; }
	public PointNormal MiddlePoint { get; private set; }
	public bool Active { get; set; }
	public float PivotingRadius { get; private set; }


	public Edge() {
		First = Second = OppositeVertex = BallCenter = MiddlePoint = null;
		Active = false;
		PivotingRadius = 0;
	}

	public Edge(renge_pcl.PointNormal first, renge_pcl.PointNormal second, renge_pcl.PointNormal opposite, renge_pcl.PointNormal ballCenter) {
		First = first;
		Second = second;
		OppositeVertex = opposite;
		BallCenter = ballCenter;
		MiddlePoint = new renge_pcl.PointNormal((First.x + Second.x) * 0.5f, (First.y + Second.y) * 0.5f, (First.z + Second.z) * 0.5f);
		Vector3 m = new Vector3(MiddlePoint.x, MiddlePoint.y, MiddlePoint.z);
		Vector3 c = new Vector3(BallCenter.x, BallCenter.y, BallCenter.z);
		PivotingRadius = (m - c).magnitude;

		Active = true;
	}
}

class Front {
	LinkedList<Edge> front;
	LinkedListNode<Edge> pos;
	SortedDictionary<int, SortedDictionary<Edge, LinkedListNode<Edge>>> points;

	public Front() {
		front = new LinkedList<Edge>();
		pos = front.First;
	}

	internal bool Contains(Edge edge) {
		return false;
	}

	internal Edge GetActiveEdge() {
		Edge e = null;
		if(front != null && front.Count > 0) {
			bool firstLoop = true;
			for (LinkedListNode<Edge> it = pos; ; it = it.Next) {
				if(it == null) {
					it = front.First;
				}
				if(!firstLoop && it == pos) {
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

	internal void Insert(Edge edge) {
		front.AddLast(edge);
		AddEdgePoints(front.Last);
	}

	internal bool OnFront(renge_pcl.PointNormal p) {
		return false;
	}

	private LinkedListNode<Edge> IsPresent(Edge edge) {
		return null;
	}

	private void AddEdgePoints(LinkedListNode<Edge> edge) {

	}

	private void RemoveEdgePoints(Edge edge) {

	}

	internal void JoinAndGlue(Edge e, PointNormal p, Pivoter pivoter) {
		//join
		if (f.Contains(new Edge(e.First, p)))
			Glue(new Edge(p, e.First), new Edge(e.First, p));
		if (f.Contains(new Edge(e.Second, p)))
			Glue(new Edge(p, e.First), new Edge(p, e.First));
	}
}

class Pivoter {
	KDTree<PointNormal> kdtree;
	PointCloud<PointNormal> cloud;
	float ballRadius;
	SortedDictionary<int, bool> notUsed;

	public Pivoter(PointCloud<PointNormal> cloud, float ballRadius) {
		this.ballRadius = ballRadius;
		this.cloud = cloud;
		kdtree = new KDTree<PointNormal>();
		kdtree.SetInputCloud(cloud);

		for (int i = 0; i < cloud.Count; i++) {
			notUsed[i] = false;
		}
	}

	internal Tuple<int, Triangle> Pivot(Edge e) {
		throw new NotImplementedException();
	}

	internal Triangle FindSeed() {
		float neighborhoodSize = 1.3f;

		Triangle seed = new Triangle();
		bool found = false;
		SortedDictionary<ulong, bool> tested = new SortedDictionary<ulong, bool>();
		List<int> removeIndices = new List<int>();

		foreach (KeyValuePair<int, bool> pair in notUsed) {
			if (found) break;

			int index0 = pair.Key;
			if (pair.Value) continue;

			List<int> indices = GetNeighbors(cloud[index0], ballRadius * neighborhoodSize);
			if (indices.Count < 3)
				continue;

			for (int j = 0; j < indices.Count; j++) {
				if (!found) {
					int index1 = indices[j];

					bool tmpVal1;
					if (index1 == index0 || !notUsed.TryGetValue(index1, out tmpVal1) || tmpVal1)
						continue;

					for (int k = 0; k < indices.Count && !found; k++) {
						int index2 = indices[k];

						bool tmpVal2;
						if (index1 == index2 || index2 == index0 || !notUsed.TryGetValue(index2, out tmpVal2) || tmpVal2)
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

						if(!found && GetBallCenter(index0, index1, index2, out center, out sequence)) {
							PointNormal ballCenter = new PointNormal(center.x, center.y, center.z);
							List<int> neighborhood = GetNeighbors(ballCenter, ballRadius);
							if (!found && isEmpty(neighborhood, index0, index1, index2, center)) {

								seed = new Triangle(cloud[sequence[0]], cloud[sequence[1]], cloud[sequence[2]], sequence[0], sequence[1], sequence[2], ballCenter, ballRadius);

								removeIndices.Add(index0);
								removeIndices.Add(index1);
								removeIndices.Add(index2);

								notUsed[index0] = true;
								notUsed[index1] = true;
								notUsed[index2] = true;

								found = true;
							}
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

	Tuple<Vector3, float> GetCircumscribedCircle(Vector3 p0, Vector3 p1, Vector3 p2) {
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

		Vector3 p0 = cloud[index0].AsVector3();
		Vector3 p1 = cloud[index1].AsVector3();
		Vector3 p2 = cloud[index2].AsVector3();
		sequence = new Vector3Int(index0, index1, index2);

		Vector3 v10 = p1 - p0;
		Vector3 v20 = p2 - p0;
		Vector3 normal = Vector3.Cross(v10, v20);

		if(normal.magnitude > 0.0000000001) {
			normal.Normalize();
			if(!IsOriented(normal, cloud[index0].NormalAsVector3(), cloud[index1].NormalAsVector3(), cloud[index2].NormalAsVector3())) {
				p0 = cloud[index1].AsVector3();
				p1 = cloud[index0].AsVector3();
				sequence = new Vector3Int(index1, index0, index2);

				v10 = p1 - p0;
				v20 = p2 - p0;
				normal = Vector3.Cross(v10, v20).normalized;
			}

			Tuple<Vector3, float> circle = GetCircumscribedCircle(p0, p1, p2);
			float squaredDistance = ballRadius * ballRadius - circle.Item2 * circle.Item2;
			if(squaredDistance > 0) {
				float distance = Mathf.Sqrt(Mathf.Abs(squaredDistance));
				center = circle.Item1 + distance * normal;
				status = true;
			}
		}

		return status;
	}

	bool IsOriented(Vector3 normal, Vector3 normal0, Vector3 normal1, Vector3 normal2) {
		int count = 0;
		count = normal0.Dot(normal) < 0 ? count + 1 : count;
		count = normal1.Dot(normal) < 0 ? count + 1 : count;
		count = normal2.Dot(normal) < 0 ? count + 1 : count;
		return count <= 1;
	}

	bool isEmpty(List<int> data, int index0, int index1, int index2, Vector3 ballCenter) {
		if (data == null || data.Count <= 0)
			return true;

		for (int i = 0; i < data.Count; i++) {
			if (data[i] == index0 || data[i] == index1 || data[i] == index2)
				continue;
			Vector3 dist = cloud[data[i]].AsVector3() - ballCenter;
			if (Mathf.Abs(dist.magnitude - ballRadius) < 0.0000001)
				continue;

			return false;
		}

		return true;
	}

	List<int> GetNeighbors(PointNormal point, float radius) {
		List<int> indices;
		kdtree.RadiusSearch(point, radius, out indices, out _);
		return indices;
	}
}