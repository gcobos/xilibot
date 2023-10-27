using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using UnityEngine.InputSystem;

public class SendPositionOnUpdate : MonoBehaviour { 
 
	public OSC osc;

  [SerializeField] private Slider _sliderThrottle;
  [SerializeField] private Slider _sliderSteering;
  [SerializeField] private Toggle _toggleProMode;

  float oldThrottleValue;
  float oldSteeringValue;

	// Use this for initialization
	void Start () {
    _sliderThrottle.onValueChanged.AddListener((v) => {
        OscMessage message = new OscMessage();
        message.address = "/1/fader1";
        message.values.Add(v);
        osc.Send(message);
    });
    
    _sliderSteering.onValueChanged.AddListener((v) => {
        OscMessage message = new OscMessage();
        message.address = "/1/fader2";
        message.values.Add(v);
        osc.Send(message);
    });

    _toggleProMode.onValueChanged.AddListener((v) => {
        OscMessage message = new OscMessage();
        message.address = "/1/toggle1";
        message.values.Add(v?1:0);
        osc.Send(message);
    });
  }

	void Update()
  {
    /*if (_sliderThrottle.value != 0.5f || oldThrottleValue != _sliderThrottle.value) {
      oldThrottleValue = _sliderThrottle.value;
      _sliderThrottle.value = 0.5f + ((_sliderThrottle.value - 0.5f) * 0.9f);
    }*/
    if (_sliderSteering.value != 0.5f || oldSteeringValue != _sliderSteering.value) {
      oldSteeringValue = _sliderSteering.value;
      _sliderSteering.value = 0.5f + ((_sliderSteering.value - 0.5f) * 0.9f);
    }
  }
}