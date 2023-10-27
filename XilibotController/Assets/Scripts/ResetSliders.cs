using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

public class ResetSliders : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {   

	}

	public void OnPointerDownDelegate(PointerEventData data)
    {
        Debug.Log("OnPointerDownDelegate called.");
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
