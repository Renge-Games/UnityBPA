/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2008-2009  Marius Muja (mariusm@cs.ubc.ca). All rights reserved.
 * Copyright 2008-2009  David G. Lowe (lowe@cs.ubc.ca). All rights reserved.
 *
 * THE BSD LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/

using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace renge_pcl {
	public class PointCloud<T>
		where T : Point {

		public List<T> Points { get; set; }
		public int Count { get; set; }

		public PointCloud(int count) {
			Count = count;
			Points = new List<T>(count);
		}

		public bool Empty() {
			return Count <= 0;
		}

		public void Add(T p) {
			Points.Add(p);
			if (Count < Points.Count)
				Count++;
		}

		public void Clear() {
			Points.Clear();
			Count = 0;
		}

		public T this[int key] {
			get {
				return Points[key];
			}
			set {
				Points[key] = value;
			}
		}
	}

	public class VoxelGrid<T> where T : Point {
		PointCloud<T> cloud_;
		float sideLength_;
		int width_, height_, depth_;
		BoundingBox rootBB_;
		Voxel[] voxels_;

		public VoxelGrid(PointCloud<T> cloud, float sideLength) {
			cloud_ = cloud;
			sideLength_ = sideLength;

			CalculateBB();

			width_ = (int)Math.Ceiling(rootBB_.SideLength.x / sideLength_);
			height_ = (int)Math.Ceiling(rootBB_.SideLength.y / sideLength_);
			depth_ = (int)Math.Ceiling(rootBB_.SideLength.z / sideLength_);



			voxels_ = new Voxel[width_ * height_ * depth_];
			for (int i = 0; i < voxels_.Length; i++) {
				voxels_[i] = new Voxel();
			}
			Vector3 min = rootBB_.Center - rootBB_.HalfLength;
			for (int i = 0; i < cloud_.Count; i++) {
				T p = cloud_[i];

				int x = (int)((p.x - min.x) / sideLength_),
					y = (int)((p.y - min.y) / sideLength_),
					z = (int)((p.z - min.z) / sideLength_);

				voxels_[x + width_ * y + width_ * height_ * z].Insert(i);
			}
		}

		public int RadiusSearch(Vector3 point, float radius, out List<int> indices) {
			Vector3 min = rootBB_.Center - rootBB_.HalfLength;
			Vector3Int index = new Vector3Int();
			index.x = (int)((point.x - min.x) / sideLength_);
			index.y = (int)((point.y - min.y) / sideLength_);
			index.z = (int)((point.z - min.z) / sideLength_);

			indices = new List<int>();
			int numVoxels = 3;

			if (radius > sideLength_) {
				index -= new Vector3Int(2, 2, 2);
				numVoxels = 6;
			} else {
				index -= new Vector3Int(1, 1, 1);
			}

			//float sqrRadius = radius * radius;
			for (int x = 0; x < numVoxels; x++) {
				int xi = (index.x + x);
				if (xi >= width_ || xi < 0) continue;
				for (int y = 0; y < numVoxels; y++) {
					int yi = (index.y + y);
					if (yi >= height_ || yi < 0) continue;
					for (int z = 0; z < numVoxels; z++) {
						int zi = (index.z + z);
						if (zi >= depth_ || zi < 0) continue;

						List<int> query = voxels_[xi + yi * width_ + zi * width_ * height_].Get();

						foreach (int i in query) {
							if ((cloud_[i] - point).magnitude <= radius) {
								indices.Add(i);
							}
						}
					}
				}
			}

			return indices.Count;
		}

		void CalculateBB() {
			Vector3 min = new Vector3(), max = new Vector3();
			for (int i = 0; i < cloud_.Count; i++) {
				var p = cloud_[i];
				if (min.x > p.x) min.x = p.x;
				else if (max.x < p.x) max.x = p.x;

				if (min.y > p.y) min.y = p.y;
				else if (max.y < p.y) max.y = p.y;

				if (min.z > p.z) min.z = p.z;
				else if (max.z < p.z) max.z = p.z;
			}

			rootBB_ = new BoundingBox(max - ((max - min) / 2), max - min);
		}

		class Voxel {
			List<int> indices;
			public int IndexCount { get; private set; }

			public Voxel() {
				indices = new List<int>();
			}

			public void Insert(int index) {
				IndexCount++;
				indices.Add(index);
			}

			public List<int> Get() {
				return indices;
			}

			public int this[int key] {
				get {
					return indices[key];
				}
				set {
					indices[key] = value;
				}
			}
		}
	}

	class BoundingBox {
		public Vector3 Center { get; private set; }
		public Vector3 SideLength { get; private set; }
		public Vector3 HalfLength { get; private set; }

		public BoundingBox(Vector3 center, Vector3 sideLength) {
			this.Center = center;
			this.SideLength = sideLength;
			HalfLength = sideLength / 2;
		}

		public bool Contains(in Vector3 point) {
			if (point.x > Center.x + HalfLength.x || point.x < Center.x - HalfLength.x ||
			point.y > Center.y + HalfLength.y || point.y < Center.y - HalfLength.y ||
			point.z > Center.z + HalfLength.z || point.z < Center.z - HalfLength.z) {
				return false;
			}
			return true;
		}

		public bool Contains(in Vector3 point, float radius) {
			float x = Mathf.Max(Center.x - HalfLength.x, Mathf.Min(Center.x + HalfLength.x, point.x));
			float y = Mathf.Max(Center.y - HalfLength.y, Mathf.Min(Center.y + HalfLength.y, point.y));
			float z = Mathf.Max(Center.z - HalfLength.z, Mathf.Min(Center.z + HalfLength.z, point.z));

			float dist = Mathf.Sqrt((x - point.x) * (x - point.x) +
									(y - point.y) * (y - point.y) +
									(z - point.z) * (z - point.z));
			return dist < radius;
		}

		public Vector3Int GetOctantCoords(in Vector3 position) {
			Vector3Int res = new Vector3Int();
			res.x = (int)Math.Floor((position.x - Center.x) / HalfLength.x) + 1;
			res.y = (int)Math.Floor((position.y - Center.y) / HalfLength.y) + 1;
			res.z = (int)Math.Floor((position.z - Center.z) / HalfLength.z) + 1;
			return res;
		}

		public BoundingBox GetOctantBoundingBox(int x, int y, int z) {
			return new BoundingBox(Center + new Vector3((x * 2 - 1) * SideLength.x, (y * 2 - 1) * SideLength.y, (z * 2 - 1) * SideLength.z) / 4, HalfLength);
		}
	}

	public class KDTree<T> where T : Point {
		public PointCloud<T> Cloud { get; protected set; }
		public float Epsilon { get; set; }
		public int MinPts { get; set; }
		public bool Sorted { get; set; }

		public const int DIMENSION = 3;
		const int LEAFMAXSIZE = 10;

		List<int> vind_;
		List<Interval> rootBBox_;
		Node rootNode_;

		public KDTree(bool sorted = true) {
			Epsilon = 0.0f;
			MinPts = 1;
			Sorted = sorted;
			rootBBox_ = new List<Interval>();
			rootNode_ = new Node();
		}

		public void SetInputCloud(PointCloud<T> cloud, List<int> indices = null) {
			Cloud = cloud;
			vind_ = new List<int>();
			for (int i = 0; i < cloud.Count; i++) {
				vind_.Add(i);
			}

			ComputeBoundingBox(rootBBox_);

			rootNode_ = DivideTree(0, Cloud.Count, rootBBox_);
		}

		public int RadiusSearch(T point, float radius, out List<int> indices, out List<float> dists) {

			List<float> query = new List<float>(3);
			query.Add(point.x);
			query.Add(point.y);
			query.Add(point.z);

			indices = new List<int>();
			dists = new List<float>();

			int neighborsInRadius;
			ResultSet resultSet = new ResultSet(radius * radius);
			FindNeighbors(resultSet, query);
			neighborsInRadius = resultSet.Size();

			indices.Capacity = neighborsInRadius;
			dists.Capacity = neighborsInRadius;
			for (int i = 0; i < neighborsInRadius; i++) {
				indices.Add(0);
				dists.Add(.0f);
			}
			if (neighborsInRadius > 0) {
				resultSet.Copy(indices, dists, Sorted);
			}

			return neighborsInRadius;
		}

		void FindNeighbors(ResultSet result, List<float> list) {
			float epsError = 1;
			List<float> dists = new List<float>();
			dists.Add(0); dists.Add(0); dists.Add(0);
			float distsq = ComputeInitialDistances(list, dists);
			SearchLevel(result, list, rootNode_, distsq, dists, epsError);
		}

		void SearchLevel(ResultSet result, List<float> vec, Node node, float mindistsq, List<float> dists, float epsError) {
			if (node.Child1 == null && node.Child2 == null) {
				float worstDist = result.WorstDist();
				for (int i = node.Left; i < node.Right; i++) {
					Point point = Cloud[vind_[i]];
					float dist = Distance(vec, point, DIMENSION);
					if (dist < worstDist) {
						result.AddPoint(dist, vind_[i]);
					}
				}
				return;
			}

			int idx = node.Divfeat;
			float val = vec[idx];
			float diff1 = val - node.DivLow;
			float diff2 = val - node.DivHigh;

			Node bestChild;
			Node otherChild;
			float cutDist;
			if ((diff1 + diff2) < 0) {
				bestChild = node.Child1;
				otherChild = node.Child2;
				cutDist = AccumDistance(val, node.DivHigh);
			} else {
				bestChild = node.Child2;
				otherChild = node.Child1;
				cutDist = AccumDistance(val, node.DivLow);
			}

			SearchLevel(result, vec, bestChild, mindistsq, dists, epsError);

			float dst = dists[idx];
			mindistsq = mindistsq + cutDist - dst;
			dists[idx] = cutDist;
			if (mindistsq * epsError <= result.WorstDist()) {
				SearchLevel(result, vec, otherChild, mindistsq, dists, epsError);
			}
			dists[idx] = dst;
		}

		Node DivideTree(int left, int right, List<Interval> bbox) {
			Node node = new Node();

			if ((right - left) <= LEAFMAXSIZE) {
				node.Child1 = node.Child2 = null;
				node.Left = left;
				node.Right = right;

				for (int i = 0; i < DIMENSION; i++) {
					Interval interval = bbox[i];
					interval.Low = Cloud[left][i];
					interval.High = Cloud[left][i];
					bbox[i] = interval;
				}
				for (int k = left + 1; k < right; k++) {
					for (int i = 0; i < DIMENSION; i++) {
						Interval interval = bbox[i];
						if (Cloud[k][i] < bbox[i].Low) interval.Low = Cloud[k][i];
						if (Cloud[k][i] > bbox[i].High) interval.High = Cloud[k][i];
						bbox[i] = interval;
					}
				}
			} else {
				int idx;
				int cutfeat;
				float cutval;
				MiddleSplit(left, right - left, out idx, out cutfeat, out cutval, bbox);

				node.Divfeat = cutfeat;

				List<Interval> leftBbox = new List<Interval>();
				leftBbox.AddRange(bbox);
				Interval interval = leftBbox[cutfeat];
				interval.High = cutval;
				leftBbox[cutfeat] = interval;
				node.Child1 = DivideTree(left, left + idx, leftBbox);

				List<Interval> rightBbox = new List<Interval>();
				rightBbox.AddRange(bbox);
				interval = rightBbox[cutfeat];
				interval.Low = cutval;
				rightBbox[cutfeat] = interval;
				node.Child2 = DivideTree(left + idx, right, rightBbox);

				node.DivLow = leftBbox[cutfeat].High;
				node.DivHigh = rightBbox[cutfeat].Low;

				for (int i = 0; i < DIMENSION; i++) {
					interval = bbox[i];
					interval.Low = Math.Min(leftBbox[i].Low, rightBbox[i].Low);
					interval.High = Math.Max(leftBbox[i].High, rightBbox[i].High);
					bbox[i] = interval;
				}
			}

			return node;
		}

		private void MiddleSplit(int ind, int count, out int index, out int cutfeat, out float cutval, List<Interval> bbox) {
			float maxSpan = bbox[0].High - bbox[0].Low;
			cutfeat = 0;
			cutval = (bbox[0].High + bbox[0].Low) / 2;
			for (int i = 0; i < DIMENSION; i++) {
				float span = bbox[i].High - bbox[i].Low;
				if (span > maxSpan) {
					maxSpan = span;
					cutfeat = i;
					cutval = (bbox[i].High + bbox[i].Low) / 2;
				}
			}

			float minElem, maxElem;
			ComputeMinMax(ind, count, cutfeat, out minElem, out maxElem);
			cutval = (minElem + maxElem) / 2;
			maxSpan = maxElem - minElem;

			int k = cutfeat;
			for (int i = 0; i < DIMENSION; i++) {
				if (i == k) continue;
				float span = bbox[i].High - bbox[i].Low;
				if (span > maxSpan) {
					ComputeMinMax(ind, count, i, out minElem, out maxElem);
					span = maxElem - minElem;
					if (span > maxSpan) {
						maxSpan = span;
						cutfeat = i;
						cutval = (minElem + maxElem) / 2;
					}
				}
			}
			int lim1, lim2;
			PlaneSplit(ind, count, cutfeat, cutval, out lim1, out lim2);

			if (lim1 > count / 2) index = lim1;
			else if (lim2 < count / 2) index = lim2;
			else index = count / 2;
		}

		private void PlaneSplit(int ind, int count, int cutfeat, float cutval, out int lim1, out int lim2) {
			int left = 0;
			int right = count - 1;
			for (; ; ) {
				while (left <= right && Cloud[vind_[ind + left]][cutfeat] < cutval) ++left;
				while (left <= right && Cloud[vind_[ind + right]][cutfeat] >= cutval) --right;
				if (left > right) break;
				//swap
				int tmp = vind_[ind + left];
				vind_[ind + left] = vind_[ind + right];
				vind_[ind + right] = tmp;

				++left; --right;
			}

			lim1 = left;
			right = count - 1;

			for (; ; ) {
				while (left <= right && Cloud[vind_[ind + left]][cutfeat] < cutval) ++left;
				while (left <= right && Cloud[vind_[ind + right]][cutfeat] >= cutval) --right;
				if (left > right) break;
				//swap
				int tmp = vind_[ind + left];
				vind_[ind + left] = vind_[ind + right];
				vind_[ind + right] = tmp;

				++left; --right;
			}

			lim2 = left;
		}

		private void ComputeMinMax(int ind, int count, int dim, out float minElem, out float maxElem) {
			minElem = Cloud[vind_[ind]][dim];
			maxElem = Cloud[vind_[ind]][dim];
			for (int i = 0; i < count; i++) {
				float val = Cloud[vind_[ind + i]][dim];
				if (val < minElem) minElem = val;
				if (val > maxElem) maxElem = val;
			}
		}

		float ComputeInitialDistances(List<float> vec, List<float> dists) {
			float distsq = 0;

			for (int i = 0; i < dists.Count; i++) {
				if (vec[i] < rootBBox_[i].Low) {
					dists[i] = AccumDistance(vec[i], rootBBox_[i].Low);
					distsq += dists[i];
				}
				if (vec[i] > rootBBox_[i].High) {
					dists[i] = AccumDistance(vec[i], rootBBox_[i].High);
					distsq += dists[i];
				}
			}

			return distsq;
		}

		void ComputeBoundingBox(List<Interval> bbox) {
			bbox.Clear();
			for (int i = 0; i < DIMENSION; i++) {
				bbox.Add(new Interval(Cloud[0][i], Cloud[0][i]));
			}

			for (int k = 0; k < Cloud.Count; k++) {
				for (int i = 0; i < DIMENSION; i++) {
					Interval interval = bbox[i];
					if (Cloud[k][i] < bbox[i].Low) interval.Low = Cloud[k][i];
					if (Cloud[k][i] > bbox[i].High) interval.High = Cloud[k][i];
					bbox[i] = interval;
				}
			}
		}

		private float Distance(List<float> vec, Point point, int size) {
			float result = 0;
			float diff;
			for (int i = 0; i < size; i++) {
				diff = vec[i] - point[i];
				result += diff * diff;
			}

			return result;
		}

		float AccumDistance(float a, float b) {
			return (a - b) * (a - b);
		}

		class Node {
			public int Left { get; set; }
			public int Right { get; set; }
			public int Divfeat { get; set; }
			public float DivLow { get; set; }
			public float DivHigh { get; set; }
			public Node Child1 { get; set; }
			public Node Child2 { get; set; }
		}

		struct Interval {
			public float Low, High;
			public Interval(float low, float high) {
				Low = low;
				High = high;
			}
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
				else if (numElements < Size()) {
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

			struct DistanceIndex : IComparable<DistanceIndex> {
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
		public float x {
			get { return data[0]; }
			set { data[0] = value; }
		}
		public float y {
			get { return data[1]; }
			set { data[1] = value; }
		}
		public float z {
			get { return data[2]; }
			set { data[2] = value; }
		}

		public float SqrMag { get { return x * x + y * y + z * z; } }

		public float Magnitude { get { return Mathf.Sqrt(SqrMag); } }

		float[] data;

		public Point(float x = 0, float y = 0, float z = 0) {
			data = new float[3];
			data[0] = x;
			data[1] = y;
			data[2] = z;
		}
		public float this[int key] {
			get {
				return data[key];
			}
			set {
				data[key] = value;
			}
		}

		public static Vector3 Cross(Point v1, Point v2) {
			float x, y, z;
			x = v1.y * v2.z - v2.y * v1.z;
			y = (v1.x * v2.z - v2.x * v1.z) * -1;
			z = v1.x * v2.y - v2.x * v1.y;
			return new Vector3(x, y, z);
		}

		public static Vector3 operator -(Point a, Point b) {
			Vector3 p = new Vector3(a.x, a.y, a.z);
			p.x -= b.x;
			p.y -= b.y;
			p.z -= b.z;
			return p;
		}

		public static Vector3 operator -(Point a, Vector3 b) {
			Vector3 v = new Vector3(a.x, a.y, a.z);
			return v - b;
		}

		public static Vector3 operator *(float a, Point b) {
			return new Vector3(a * b.x, a * b.y, a * b.z);
		}

		public Vector3 AsVector3() {
			return new Vector3(data[0], data[1], data[2]);
		}
	}

	public class PointNormal : Point {
		public float nx {
			get { return ndata[0]; }
			set { ndata[0] = value; }
		}
		public float ny {
			get { return ndata[1]; }
			set { ndata[1] = value; }
		}
		public float nz {
			get { return ndata[2]; }
			set { ndata[2] = value; }
		}
		public float Curvature { get; set; }

		float[] ndata;

		public PointNormal(float x, float y, float z, float nx = 0, float ny = 0, float nz = 0, float curvature = 0)
			: base(x, y, z) {
			ndata = new float[3];
			ndata[0] = nx;
			ndata[1] = ny;
			ndata[2] = nz;
			this.Curvature = curvature;
		}

		internal void NormalizeNormal() {
			Vector3 n = NormalAsVector3().normalized;
			nx = n.x;
			ny = n.y;
			nz = n.z;
		}

		internal PointNormal GetNormalized() {
			Vector3 n = NormalAsVector3().normalized;
			return new PointNormal(x, y, z, n.x, n.y, n.z);
		}

		internal Vector3 NormalAsVector3(float scale = 1.0f) {
			return new Vector3(ndata[0] * scale, ndata[1] * scale, ndata[2] * scale);
		}
	}

	public class HyperPlane {
		Vector3 normal;
		float offset;
		public HyperPlane(Vector3 n, Vector3 e) {
			normal = n;
			offset = -n.Dot(e);
		}

		public Vector3 Projection(Vector3 p) {
			return (p - SignedDistance(p) * normal);
		}

		public Vector3 Projection<T>(T p) where T : Point {
			return (p - SignedDistance(p) * normal);
		}

		public float SignedDistance(Vector3 p) {
			return normal.Dot(p) + offset;
		}

		public float SignedDistance<T>(T p) where T : Point{
			return normal.Dot(p) + offset;
		}

		public float AbsDistance<T>(T p) where T : Point{
			return Mathf.Abs(SignedDistance(p));
		}
	}

	class Triangle {
		public Tuple<PointNormal, int> First { get; set; }
		public Tuple<PointNormal, int> Second { get; set; }
		public Tuple<PointNormal, int> Third { get; set; }
		public Vector3 BallCenter { get; set; }
		public float BallRadius { get; set; }

		public Triangle() {
			First = Second = Third = null;
			BallRadius = 0;
		}

		public Triangle(PointNormal p0, PointNormal p1, PointNormal p2, int index0, int index1, int index2, Vector3 ballCenter, float ballRadius) {
			First = new Tuple<PointNormal, int>(p0, index0);
			Second = new Tuple<PointNormal, int>(p1, index1);
			Third = new Tuple<PointNormal, int>(p2, index2);
			BallCenter = ballCenter;
			BallRadius = ballRadius;
		}

		public Tuple<PointNormal, int> GetVertex(int index) {
			switch (index) {
				case 0:
					return First;
				case 1:
					return Second;
				case 2:
					return Third;
				default:
					return null;
			}
		}

		public Edge GetEdge(int index) {
			int index0 = index % 3;
			int index1 = (index + 1) % 3;
			int index2 = (index + 2) % 3;
			return new Edge(GetVertex(index0), GetVertex(index1), GetVertex(index2), BallCenter);
		}

		public bool Equals(Triangle tri) {
			int a1 = First.Item2, b1 = Second.Item2, c1 = Third.Item2;
			int a2 = tri.First.Item2, b2 = tri.Second.Item2, c2 = tri.Third.Item2;

			return (a1 == a2 || a1 == b2 || a1 == c2) && (b1 == a2 || b1 == b2 || b1 == c2) && (c1 == a2 || c1 == b2 || c1 == c2);

			//return First.Item2.Equals(tri.First.Item2) && Second.Item2.Equals(tri.Second.Item2) && Third.Item2.Equals(tri.Third.Item2);
		}

		public override bool Equals(object obj) {
			return Equals(obj as Triangle);
		}

		public override int GetHashCode() {
			return First.GetHashCode() ^ Second.GetHashCode() ^ Third.GetHashCode();
		}
	}

	class Edge {
		public Tuple<PointNormal, int> First { get; set; }
		public Tuple<PointNormal, int> Second { get; set; }
		public Tuple<PointNormal, int> OppositeVertex { get; set; }

		public Vector3 BallCenter { get; private set; }
		public Vector3 MiddlePoint { get; private set; }
		public bool Active { get; set; }
		public float PivotingRadius { get; private set; }


		public Edge() {
			First = Second = OppositeVertex = null;
			Active = false;
			PivotingRadius = 0;
		}

		public Edge(Tuple<PointNormal, int> first, Tuple<PointNormal, int> second, Tuple<PointNormal, int> opposite, in Vector3 ballCenter) {
			First = first;
			Second = second;
			OppositeVertex = opposite;
			BallCenter = ballCenter;
			MiddlePoint = new Vector3((First.Item1.x + Second.Item1.x) * 0.5f, (First.Item1.y + Second.Item1.y) * 0.5f, (First.Item1.z + Second.Item1.z) * 0.5f);
			PivotingRadius = (MiddlePoint - BallCenter).magnitude;

			Active = true;
		}

		public bool Equals(Edge edge) {
			int a1 = First.Item2, b1 = Second.Item2, c1 = OppositeVertex.Item2;
			int a2 = edge.First.Item2, b2 = edge.Second.Item2, c2 = edge.OppositeVertex.Item2;

			return (a1 == a2 && b1 == b2 && c1 == c2) || (a1 == b2 && b1 == a2 && c1 == c2);

			//return First.Item2.Equals(edge.First.Item2) && Second.Item2.Equals(edge.Second.Item2) && OppositeVertex.Item2.Equals(edge.OppositeVertex.Item2);
		}

		public override bool Equals(object obj) {
			return Equals(obj as Edge);
		}

		public override int GetHashCode() {
			return First.Item2 ^ Second.Item2 ^ OppositeVertex.Item2;
		}
	}

	public static class MyExtensions {
		//Source:
		//https://github.com/mcraiha/CSharp-nth_element
		//My implementation is slightly modified to be a List extension
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
}