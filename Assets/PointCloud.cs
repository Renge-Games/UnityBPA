using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class PointCloud<T>
	where T : Point {
	
	public List<T> Points { get; set; }
	public int Count { get; set; }

	public PointCloud(int count) {
		Count = count;
		Points = new List<T>(count);
	}

	public void Add(T p) {
		Points.Add(p);
	}

	public void Clear() {
		Points.Clear();
		Count = 0;
	}

	public T Get(int index) {
		if (index >= 0 && index < Count)
			return Points[index];
		else
			return null;
	}
}

public class KDTree<T> where T : Point{
	public List<int> Indices { get; protected set; }
	public PointCloud<T> Input { get; protected set; }
	public float Epsilon { get; set; }
	public int MinPts { get; set; }
	public bool Sorted { get; set; }
	public const int DIMENSION = 3;

	public KDTree(bool sorted = true) {
		Epsilon = 0.0f;
		MinPts = 1;
		Sorted = sorted;
	}

	public void SetInputCloud(PointCloud<T> cloud, List<int> indices = null) {
		Input = cloud;
		Indices = indices == null ? new List<int>() : indices;
	}

	public int RadiusSearch(T point, float radius, List<int> kIndices, List<float> kSqrDistances) {

		List<float> query = new List<float>(3);
		query.Add(point.x);
		query.Add(point.y);
		query.Add(point.z);

		List<int> indices = new List<int>();
		List<float> dists = new List<float>();

		int neighborsInRadius = 0;
		int count = 0;
		ResultSet resultSet = new ResultSet(radius);
		count = resultSet.Size();



		resultSet.Clear();
		FindNeighbors(resultSet, query);

		return neighborsInRadius;
	}

	private void FindNeighbors(ResultSet result, List<float> list) {

	}

	class ResultSet {
		float radius;
		List<DistanceIndex> distIndex;
		int dICount;

		public ResultSet(float radius) {
			this.radius = radius;
			distIndex = new List<DistanceIndex>(1024);
			dICount = 0;
		}

		public void Clear() {
			distIndex.Clear();
			dICount = 0;
		}

		public int Size() {
			return dICount;
		}

		public void Copy(List<int> indices, List<float> dists, bool sorted = true) {
			int numElements = indices.Count;
			if (sorted)
				distIndex.Sort();
			else if(numElements < Size()) {
				distIndex.NthElement(0, numElements, numElements);
			}

			int n = Math.Min(dICount, numElements);
			for (int i = 0; i < n; i++) {
				indices[i] = distIndex[i].index;
				dists[i] = distIndex[i].dist;
			}
		}

		public float WorstDist() {
			return radius;
		}

		public void AddPoint(float dist, int index) {
			distIndex.Add(new DistanceIndex(dist, index));
			dICount++;
		}

		struct DistanceIndex : IComparable<DistanceIndex>{
			public int index;
			public float dist;
			public DistanceIndex(float dist, int index) {
				this.dist = dist;
				this.index = index;
			}

			public int CompareTo(DistanceIndex other) {
				if (dist < other.dist || dist == other.dist && index < other.index)
					return -1;
				if (dist == other.dist && index == other.index)
					return 0;
				return 1;
			}
		}
	}
}

public class Point {
	public float x { get; set; }
	public float y { get; set; }
	public float z { get; set; }

	public Point(float x, float y, float z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}
}

public class PointNormal : Point{
	public float nx { get; set; }
	public float ny { get; set; }
	public float nz { get; set; }
	public float Curvature { get; set; }

	public PointNormal(float x, float y, float z, float nx = 0, float ny = 0, float nz = 0, float curvature = 0) 
		:base(x, y, z){
		this.nx = nx;
		this.ny = ny;
		this.nz = nz;
		this.Curvature = curvature;
	}
}

public static class NthExtensions {
	public static void NthElement<T>(this List<T> array, int startIndex, int nthSmallest, int endIndex) {
		int from = startIndex;
		int to = endIndex;

		// if from == to we reached the kth element
		while (from < to) {
			int r = from, w = to;
			T mid = array[(r + w) / 2];

			// stop if the reader and writer meets
			while (r < w) {
				if (Comparer<T>.Default.Compare(array[r], mid) > -1) { // put the large values at the end
					T tmp = array[w];
					array[w] = array[r];
					array[r] = tmp;
					w--;
				} else { // the value is smaller than the pivot, skip
					r++;
				}
			}

			// if we stepped up (r++) we need to step one down
			if (System.Collections.Generic.Comparer<T>.Default.Compare(array[r], mid) > 0) {
				r--;
			}

			// the r pointer is on the end of the first k elements
			if (nthSmallest <= r) {
				to = r;
			} else {
				from = r + 1;
			}
		}

		return;
	}
}