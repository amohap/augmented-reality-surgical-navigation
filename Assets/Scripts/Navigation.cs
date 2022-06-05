using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Vuforia;
using System.IO;

public class Navigation : MonoBehaviour
{
    public static int DegreeBetween(GameObject plannedTrajectory, GameObject screwEntryPoint, GameObject actualTrajectory, GameObject TipSphere){
        double angle = 0.0f;
        double degree = 0.0f;
        Vector3 plannedVector = Vector3.zero;
        Vector3 actualVector = Vector3.zero;

        // find the two vectors from the plannedTrajectory (from Vertebra given) and the actualTrajectory (from the Marker)
        plannedVector = new Vector3(screwEntryPoint.transform.position.x - plannedTrajectory.transform.position.x,  screwEntryPoint.transform.position.y - plannedTrajectory.transform.position.y, screwEntryPoint.transform.position.z - plannedTrajectory.transform.position.z);
        actualVector = new Vector3(TipSphere.transform.position.x - actualTrajectory.transform.position.x, TipSphere.transform.position.y - actualTrajectory.transform.position.y, TipSphere.transform.position.z - actualTrajectory.transform.position.z);
        
        // compute the angle between the two vectors
        double nom = (plannedVector.x * actualVector.x + plannedVector.y * actualVector.y + plannedVector.z * actualVector.z);
        double denum = (Math.Sqrt(Math.Pow(plannedVector.x, 2) + Math.Pow(plannedVector.y, 2) + Math.Pow(plannedVector.z, 2)) * Math.Sqrt(Math.Pow(actualVector.x, 2) + Math.Pow(actualVector.y, 2) + Math.Pow(actualVector.z, 2)));
        angle = Math.Acos(nom/denum);
        
        // convert angle to degree
        degree = angle * 180 / Math.PI;
        return System.Convert.ToInt32(System.Math.Floor(degree));
    }

    public static void Audio(GameObject screwEntryPoint, GameObject TipSphere){
        float dist;

        // reset the victory sound
        TipSphere.GetComponents<AudioSource>()[1].enabled = false;

        // enable distance audio
        TipSphere.GetComponents<AudioSource>()[0].enabled = true;
        
        // compute distance
        dist = Vector3.Distance(TipSphere.transform.position, screwEntryPoint.transform.position)*100; // distance in cm
        Debug.Log("Distance:" + dist);

        // scale volume accordingly: make volume louder if we are moving away from the screw entry point
        TipSphere.GetComponents<AudioSource>()[0].volume = dist / 10f; 
    }

    
}