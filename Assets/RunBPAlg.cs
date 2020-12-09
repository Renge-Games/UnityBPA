using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RunBPAlg : MonoBehaviour {
	public GameObject BPAlgObject;

	void Start() {
		BallPivotingAlgorithm bpa1;
		if(BPAlgObject.TryGetComponent(out bpa1)) {
			Debug.Log("Running first BP Alg...");
			bpa1.Run(5000, 10.0f, 1.5f);
		}
	}
}
