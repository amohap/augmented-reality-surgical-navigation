using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Vuforia;
using System.IO;

public class SceneManagerNavigation : MonoBehaviour
{
    public GameObject TipSphere;        // Sphere at tip of pointer, to capture landmarks/point cloud
    public Material TipSphereMaterial;     // Sphere material, to create different colored materials
    public GameObject VertebraModel;    // Vertebra model, to transform and show/hide vertebra model
    public Transform[] Landmarks;       // Vertebra landmarks, to compute initial alignment

    // GUI
    public Button LeftButton;
    public Button TopButton;
    public Button RightButton;
    public Button RegisterInitialButton;
    public Button CollectingToggleButton;
    public Button DeleteLastButton;
    public Button RegisterICPButton;
    public Button ResetButton;
    public GameObject collectedSpheresText;
    public GameObject collectedSpheresSavedText;
    
    // buttons/objects for navigation task
    public Button LeftNavigationButton;
    public Button RightNavigationButton;
    public GameObject degreeText;

    // Private
    GameObject[] landmarkSpheres = { null, null, null };    // 3 Landmarks captured by the user using the pointer (will be used for initial alignment)
    bool isCollectingPoints = false;    // Are we currently collecting points?
    List<GameObject> collectedSpheresCurrent = new List<GameObject>();   // Container for current collected points
    List<GameObject> collectedSpheresSaved = new List<GameObject>();  // Container for saved collected points (will be used in ICP registration)
    // navigation task variables
    bool isLeftNavigating = false; // Are we currently doing the navigation task on the left side?
    bool isRightNavigating = false; // Are we currently doing the navigation task on the right side?  
    bool isLeftVictory = false; // Did we find the screw entry point on the left side?
    bool isRightVictory = false; // Did we find the screw entry point of the right side?

    // Start is called before the first frame update
    void Start()
    {
        LeftButton.onClick.AddListener(LeftClicked);
        TopButton.onClick.AddListener(TopClicked);
        RightButton.onClick.AddListener(RightClicked);
        RegisterInitialButton.onClick.AddListener(RegisterInitialClicked);
        CollectingToggleButton.onClick.AddListener(CollectingToggleClicked);
        DeleteLastButton.onClick.AddListener(DeleteLastClicked);
        RegisterICPButton.onClick.AddListener(RegisterICPClicked);
        ResetButton.onClick.AddListener(ResetClicked);

        // buttons for navigation task (ex 3)
        LeftNavigationButton.onClick.AddListener(LeftNavigation);
        RightNavigationButton.onClick.AddListener(RightNavigation);
    }

    private void LeftClicked()
    {
        // Check whether left landmark (landmark[0]) already exists. If yes, destroy it.
        // CODE HERE
        if (landmarkSpheres[0] != null)
        {
            Destroy(landmarkSpheres[0]);
        }
        // Use CreateSphere method to create a red sphere with scale 0.005 and assign resulting sphere to left landmark
        // CODE HERE
        landmarkSpheres[0] = CreateSphere(Color.red, 0.005f);
    }

    private void TopClicked()
    {
        // Check whether top landmark (landmark[1]) already exists. If yes, destroy it.
        // CODE HERE
        if (landmarkSpheres[1] != null)
        {
            Destroy(landmarkSpheres[1]);
        }
        // Use CreateSphere method to create a green sphere with scale 0.005 and assign resulting sphere to top landmark
        // CODE HERE
        landmarkSpheres[1] = CreateSphere(Color.green, 0.005f);
    }

    private void RightClicked()
    {
        // Check whether top landmark (landmark[2]) already exists. If yes, destroy it.
        // CODE HERE
        if (landmarkSpheres[2] != null)
        {
            Destroy(landmarkSpheres[2]);
        }
        // Use CreateSphere method to create a blue sphere with scale 0.005 and assign resulting sphere to right landmark
        // CODE HERE
        landmarkSpheres[2] = CreateSphere(Color.blue, 0.005f);
    }

    private GameObject CreateSphere(Color color, float scale = 0.005f)
    {
        // Use Unity's Instantiate method to create a deep copy of tip sphere
        // CODE HERE
        GameObject cloneTipSphere;
        cloneTipSphere = Instantiate(TipSphere, TipSphere.transform.position, TipSphere.transform.rotation) as GameObject;

        // Use Unity's Instantiate method to create a deep copy of tip sphere material
        // CODE HERE
        Material cloneTipSphereMaterial = Instantiate(TipSphereMaterial); 
        
        // Change the color of the material to the desired color
        // CODE HERE
        cloneTipSphereMaterial.SetColor("_Color", color);

        // Assign the new material as the material of the new sphere (HINT: you need the Unity's GetComponent<>() method)
        // CODE HERE
        cloneTipSphere.GetComponent<Renderer>().material = cloneTipSphereMaterial;        

        // Set the scale of the new sphere to the desired size
        // CODE HERE
        cloneTipSphere.transform.localScale = new Vector3(scale, scale, scale);

        // Return the new sphere
        return cloneTipSphere;
    }

    public void RegisterInitialClicked()
    {
        // Check whether all landmarks were captured
        if (landmarkSpheres[0] == null || landmarkSpheres[1] == null || landmarkSpheres[2] == null)
        {
            Debug.Log("Not all landmarks defined");
            return;
        }

        // Prepare source and target spheres as Vector3 arrays
        Vector3[] source = { Landmarks[0].position, Landmarks[1].position, Landmarks[2].position };
        Vector3[] target = { landmarkSpheres[0].transform.position, landmarkSpheres[1].transform.position, landmarkSpheres[2].transform.position };

        // Compute transformation
        var svdResult = Registration.SVDStep(source, target);

        // Transform vertebra model according to SVD result
        Vector3 axis;
        float angle;
        svdResult.Rotation.ToAngleAxis(out angle, out axis);
        VertebraModel.transform.RotateAround(svdResult.SourceCentroid, axis, angle);
        VertebraModel.transform.position += svdResult.Translation;
    }

    // Update is called once per frame
    void Update()
    {
        // Check whether we are collecting at the moment
        if (isCollectingPoints)
        {
            // Use CreateSphere to create a cyan colored sphere of scale 0.001 and add it to the collectedSpheresCurrent
            // Only add spheres with a distance e.g. > 2 mm to the last added sphere
            // HINT: the first time you add a sphere needs to be treated differently
            // Update the collectedSpheresText
            GameObject sphere;
            GameObject newSphere;
            GameObject lastSphere;

            // CODE HERE
            if (collectedSpheresCurrent.Count == 0){
                if (collectedSpheresSaved.Count > 0){
                    newSphere = CreateSphere(Color.cyan, 0.001f);
                    lastSphere = collectedSpheresSaved[collectedSpheresSaved.Count - 1];
                    float dist = Vector3.Distance(newSphere.transform.position, lastSphere.transform.position);
                    if (dist > 0.002f){
                        collectedSpheresCurrent.Add(newSphere);
                        collectedSpheresText.GetComponent<Text>().text = collectedSpheresCurrent.Count.ToString();
                    }
                    else {
                        Destroy(newSphere);
                    }
                }
                else if (collectedSpheresSaved.Count == 0){
                    sphere = CreateSphere(Color.cyan, 0.001f);
                    collectedSpheresCurrent.Add(sphere);
                    collectedSpheresText.GetComponent<Text>().text = collectedSpheresCurrent.Count.ToString();
                }
            }
            else if (collectedSpheresCurrent.Count > 0) {
                newSphere = CreateSphere(Color.cyan, 0.001f);
                lastSphere = collectedSpheresCurrent[collectedSpheresCurrent.Count - 1];
                float dist = Vector3.Distance(newSphere.transform.position, lastSphere.transform.position);
                if (dist > 0.002f){
                    collectedSpheresCurrent.Add(newSphere);
                    collectedSpheresText.GetComponent<Text>().text = collectedSpheresCurrent.Count.ToString();
                }
                else {
                    Destroy(newSphere);
                }
               
            }
        }

        // Logic for navigation task
        // Left navigation
        if (isLeftNavigating && !isLeftVictory &&!isRightNavigating && VertebraModel.GetComponent<Renderer>().enabled == true){
            GameObject plannedTrajectory = VertebraModel.transform.GetChild(4).gameObject;
            GameObject screwEntryPoint = VertebraModel.transform.GetChild(3).gameObject;
            GameObject actualTrajectory = TipSphere.transform.GetChild(0).gameObject;

            // compute the angle between the planned and the actual trajectory on the fly
            int degree = Navigation.DegreeBetween(plannedTrajectory, screwEntryPoint, actualTrajectory, TipSphere);
            degreeText.GetComponent<Text>().text = degree.ToString();

            // 1. compute distance from tipsphere to screw entry point
            // 2. make louder sound if surgical instrument is further away from the screw entry point
            Navigation.Audio(screwEntryPoint, TipSphere);

            // victory if degree < 5 off and distance is <= 0.5 cm away
            if (degree <= 5 && TipSphere.GetComponents<AudioSource>()[0].volume <= 0.05){
                TipSphere.GetComponents<AudioSource>()[1].enabled = true;
                isLeftVictory = true;
                TipSphere.GetComponents<AudioSource>()[0].enabled = false;
            }
        }
        
        // Right navigation
        else if (isRightNavigating && !isRightVictory && !isLeftNavigating && VertebraModel.GetComponent<Renderer>().enabled == true){
            GameObject plannedTrajectory = VertebraModel.transform.GetChild(5).gameObject; 
            GameObject actualTrajectory = TipSphere.transform.GetChild(0).gameObject;
            GameObject screwEntryPoint = VertebraModel.transform.GetChild(6).gameObject;

            // compute the angle between the planned and the actual trajectory on the fly
            int degree = Navigation.DegreeBetween(plannedTrajectory, screwEntryPoint, actualTrajectory, TipSphere);
            degreeText.GetComponent<Text>().text = degree.ToString();
            
            // 1. compute distance from tipsphere to screw entry point
            // 2. make louder sound if surgical instrument is further away from the screw entry point
            Navigation.Audio(screwEntryPoint, TipSphere);

            // victory if degree < 5 off and distance is <= 0.5 cm away
            if (degree <= 5 && TipSphere.GetComponents<AudioSource>()[0].volume <= 0.05){
                TipSphere.GetComponents<AudioSource>()[1].enabled = true;
                isRightVictory = true;
                TipSphere.GetComponents<AudioSource>()[0].enabled = false;
            }
        }

        else if (!isRightNavigating && !isLeftNavigating){
            TipSphere.GetComponents<AudioSource>()[0].enabled = false;
            TipSphere.GetComponents<AudioSource>()[1].enabled = false;
            isLeftVictory = false;
            isRightVictory = false;
        }
    }

    private void CollectingToggleClicked()
    {
        // Check whether we are starting to collect now. If so
        // 1) check whether the collectedSpheresCurrent container contains any spheres that need to be copied to the collectedSpheresSaved container
        // 2) Clear the collectedSpheresCurrent container
        // 3) Update the texts of collectedSpheresText and collectedSpheresSavedText
        // CODE HERE
         if (!isCollectingPoints && collectedSpheresCurrent.Count > 0)
        {
            collectedSpheresSaved.AddRange(collectedSpheresCurrent);
            collectedSpheresCurrent.Clear();
            collectedSpheresText.GetComponent<Text>().text = collectedSpheresCurrent.Count.ToString();
            collectedSpheresSavedText.GetComponent<Text>().text = collectedSpheresSaved.Count.ToString();
        }
        // Hide/show vertebra model depending on whether we started or stopped collecting points
        // CODE HERE
        if (isCollectingPoints){
            VertebraModel.GetComponent<Renderer>().enabled = true;
        }
        else if (!isCollectingPoints){
            VertebraModel.GetComponent<Renderer>().enabled = false;
        }

        // Toggle the isCollectingPoints variable
        // CODE HERE
        isCollectingPoints = !isCollectingPoints;

        // Change the text of the toggle button to "Start collect" or "Stop collect"
        // CODE HERE
        if (isCollectingPoints){
            GameObject.Find("Start collecting").GetComponentInChildren<Text>().text = "Stop collect";
        }
        else {
            GameObject.Find("Start collecting").GetComponentInChildren<Text>().text = "Start collect";
        }
    }

    private void DeleteLastClicked()
	{
        // Destroy all spheres in collectedSpheresCurrent
        foreach (var sphere in collectedSpheresCurrent)
        {
            Destroy(sphere);
        }

        // Clear the collectedSpheresCurrent container
        collectedSpheresCurrent.Clear();

        // Update the collectedSpheresText and the collectedSpheresSavedText
        collectedSpheresText.GetComponent<Text>().text = collectedSpheresCurrent.Count.ToString();
        collectedSpheresSavedText.GetComponent<Text>().text = collectedSpheresSaved.Count.ToString();
    }

    private void RegisterICPClicked()
    {
        // Make sure user is not collecting points and add remaining collected points to saved points
        if (!isCollectingPoints && collectedSpheresCurrent.Count > 0)
        {
            collectedSpheresSaved.AddRange(collectedSpheresCurrent);
            collectedSpheresCurrent.Clear();
            collectedSpheresText.GetComponent<Text>().text = collectedSpheresCurrent.Count.ToString();
            collectedSpheresSavedText.GetComponent<Text>().text = collectedSpheresSaved.Count.ToString();
        }

        // Check whether enough points were saved
        if (collectedSpheresSaved.Count < 4)
        {
            Debug.Log("Not enough points collected");
            return;
        }

        // Transform model vertices according to current pose and store in List<Vector3> (ICP input)
        var meshVertices = VertebraModel.GetComponent<MeshFilter>().mesh.vertices;
        List<Vector3> modelPoints = new List<Vector3>();
        for (int i = 0; i < meshVertices.Length; ++i)
        {
            modelPoints.Add(VertebraModel.transform.position + VertebraModel.transform.rotation * meshVertices[i]);
        }

        // Store sampled spheres as List<Vector3> (ICP input)
        List<Vector3> sampledPoints = new List<Vector3>();
        for (int i = 0; i < collectedSpheresSaved.Count; ++i)
        {
            sampledPoints.Add(collectedSpheresSaved[i].transform.position);
        }

        // Run ICP
        SVDResult result = Registration.ICP(sampledPoints, modelPoints, 20, 0.0000001);

        // Transform model (as explained in the exercise description, here, we apply the inverse transformation)
        Vector3 axis;
        float angle;
        Quaternion.Inverse(result.Rotation).ToAngleAxis(out angle, out axis);
        VertebraModel.transform.RotateAround(result.TargetCentroid, axis, angle);
        VertebraModel.transform.position -= result.Translation;
    }

    public void ResetClicked()
    {
        // Destroy all landmark spheres
        foreach (var sphere in landmarkSpheres)
        {
            Destroy(sphere);
        }
        // Destroy all collectedSpheresCurrent
        foreach (var sphere in collectedSpheresCurrent)
        {
            Destroy(sphere);
        }
        // Destroy all collectedSpheresSaved
        foreach (var sphere in collectedSpheresSaved)
        {
            Destroy(sphere);
        }

        // Clear containers collectedSpheresCurrent and collectedSpheresSaved
        // CODE HERE
        collectedSpheresCurrent.Clear();
        collectedSpheresSaved.Clear();

        // Update collectedSpheresText and collectedSpheresSavedText
        // CODE HERE
        collectedSpheresText.GetComponent<Text>().text = collectedSpheresCurrent.Count.ToString();
        collectedSpheresSavedText.GetComponent<Text>().text = collectedSpheresSaved.Count.ToString();

        // Reset model transformation
        // reset to the world origin with no rotation
        // CODE HERE
        VertebraModel.transform.position = new Vector3 (0, 0, 0);
        VertebraModel.transform.rotation = new Quaternion (0f, 0f, 0f, 0f);

        // Resets for navigation task:
        isRightNavigating = false;
        isLeftNavigating = false;

        // degree for navigation task
        degreeText.GetComponent<Text>().text = "0";

        // vertebra model planned trajectories & screw entry points
        VertebraModel.transform.GetChild(4).gameObject.SetActive(false); 
        VertebraModel.transform.GetChild(3).gameObject.SetActive(false);
        VertebraModel.transform.GetChild(5).gameObject.SetActive(false); 
        VertebraModel.transform.GetChild(6).gameObject.SetActive(false);

        // actual trajectory
        TipSphere.transform.GetChild(0).gameObject.SetActive(false);

        // audio
        TipSphere.GetComponents<AudioSource>()[0].enabled = false;
        TipSphere.GetComponents<AudioSource>()[1].enabled = false;
        
        // victory
        isLeftVictory = false;
        isRightVictory = false;

    }

    // code for navigation task
    private void LeftNavigation(){

        if (!isRightNavigating && VertebraModel.GetComponent<Renderer>().enabled == true){ // only one task at a time 
            // disable the planned trajectory and the entry point
            if (VertebraModel.transform.GetChild(4).gameObject.activeSelf){
                isLeftNavigating = false;
                VertebraModel.transform.GetChild(4).gameObject.SetActive(false); 
                VertebraModel.transform.GetChild(3).gameObject.SetActive(false);
                // Set the actual trajectory to not being visible
                TipSphere.transform.GetChild(0).gameObject.SetActive(false);
                // Disable set degree to 0
                degreeText.GetComponent<Text>().text = "0";
                }
            // enable the planned trajectory and the entry point
            else {
                isLeftNavigating = true;
                VertebraModel.transform.GetChild(4).gameObject.SetActive(true);
                VertebraModel.transform.GetChild(3).gameObject.SetActive(true);
                // Set the actual trajectory to being visible
                TipSphere.transform.GetChild(0).gameObject.SetActive(true);
            }
        }
        
    }

    private void RightNavigation(){

        if (!isLeftNavigating && VertebraModel.GetComponent<Renderer>().enabled == true){ // only one task at a time
            // disable the planned trajectory and the entry point
            if (VertebraModel.transform.GetChild(5).gameObject.activeSelf){
                isRightNavigating = false;
                VertebraModel.transform.GetChild(5).gameObject.SetActive(false); 
                VertebraModel.transform.GetChild(6).gameObject.SetActive(false);
                // Set the actual trajectory to not being visible
                TipSphere.transform.GetChild(0).gameObject.SetActive(false);
                // Disable set degree to 0
                degreeText.GetComponent<Text>().text = "0";
                }
            // enable the planned trajectory and the entry point
            else {
                isRightNavigating = true;
                VertebraModel.transform.GetChild(5).gameObject.SetActive(true);
                VertebraModel.transform.GetChild(6).gameObject.SetActive(true);
                // Set the actual trajectory to being visible
                TipSphere.transform.GetChild(0).gameObject.SetActive(true);
            }
        }
    }

}