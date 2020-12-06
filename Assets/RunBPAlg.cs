using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RunBPAlg : MonoBehaviour {
	public GameObject BPAlgObject;

	void Start() {
		BallPivotingAlgorithm bpa1;
		BallPivotingAlgorithm1 bpa2;
		if(BPAlgObject.TryGetComponent(out bpa1)) {
			Debug.Log("Running first BP Alg...");
			bpa1.Run();
		} else if(BPAlgObject.TryGetComponent(out bpa2)){
			Debug.Log("Running second BP Alg...");
			bpa2.Run();
		}
	}
}
