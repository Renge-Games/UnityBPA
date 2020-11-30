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
		return null;
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

	Tuple<Vector3, float> GetCircumScribedCircle(Vector3 p0, Vector3 p1, Vector3 p2) {
		return null;
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

			Tuple<Vector3, float> circle = GetCircumScribedCircle(p0, p1, p2);
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