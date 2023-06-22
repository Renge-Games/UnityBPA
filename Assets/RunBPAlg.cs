using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RunBPAlg : MonoBehaviour {
	public GameObject BPAlgObject;
	public float pivotingRadius;
	public bool runInUpdate = true;
	public int pivotsPerUpdate = 1;
	public int pivotAnimationSteps = 10;
	public bool fromShape = true;
	public int shapeVertexCount = 1000;
	public float shapeScale = 10.0f;
	void Start() {
		BallPivotingAlgorithm bpa;
		if (BPAlgObject.TryGetComponent(out bpa)) {
			Debug.Log("Running BPA...");
			if (runInUpdate) {
				bpa.RunInUpdate(fromShape, PointMeshType.Sphere, pivotsPerUpdate, pivotAnimationSteps, shapeVertexCount, shapeScale, pivotingRadius);
			} else {
				float[] passes = new float[] { pivotingRadius };
				bpa.Run(fromShape, PointMeshType.Sphere, shapeVertexCount, shapeScale, passes);
			}

		}
	}
}
