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
	void Start() {
		BallPivotingAlgorithm bpa;
		if (BPAlgObject.TryGetComponent(out bpa)) {
			Debug.Log("Running BPA...");
			if (runInUpdate) {
				bpa.RunInUpdate(fromShape, PointMeshType.Sphere, pivotsPerUpdate, pivotAnimationSteps, 10000, 10.0f, pivotingRadius);
			} else {
				float[] passes = new float[] { pivotingRadius };
				bpa.Run(100000, 10.0f, passes);
			}

		}
	}
}
