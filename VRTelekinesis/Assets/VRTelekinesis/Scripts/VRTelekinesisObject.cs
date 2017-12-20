using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VRTelekinesisObject : MonoBehaviour {
    [SerializeField]
    [Tooltip("Add some mesh renderers to flash, graphically indicating that the object is a Telekinesis Object")]
    private Renderer[] m_FlashIndicators;
    [SerializeField]
    [Tooltip("Flash Time")]
    private float time = 1f;

    private Color[] m_ColorA;
    private Color[] m_ColorB;
    private void Start()
    {
        m_ColorA = new Color[m_FlashIndicators.Length];
        m_ColorB = new Color[m_FlashIndicators.Length];
        for (int ix = 0; ix < m_FlashIndicators.Length; ix++)
        {
            // Flash the object. 
            // Determine whether it's bright or dark using YIQ formula.
            float y = 0.2989f * m_FlashIndicators[ix].material.color.r + 0.5870f * m_FlashIndicators[ix].material.color.g + 0.1140f * m_FlashIndicators[ix].material.color.b;
            float i = 0.60f * m_FlashIndicators[ix].material.color.r - 0.28f * m_FlashIndicators[ix].material.color.g - 0.32f * m_FlashIndicators[ix].material.color.b;
            float q = 0.21f * m_FlashIndicators[ix].material.color.r - 0.52f * m_FlashIndicators[ix].material.color.g + 0.31f * m_FlashIndicators[ix].material.color.b;
            float yiq = (y + i + q);

            // If bright, flash dark; If dark, flash bright.
            Color flashColorBase = (yiq >= .5f) ? Color.black : Color.white;
            float r = (m_FlashIndicators[ix].material.color.r + flashColorBase.r) / 2;
            float g = (m_FlashIndicators[ix].material.color.g + flashColorBase.g) / 2;
            float b = (m_FlashIndicators[ix].material.color.b + flashColorBase.b) / 2;

            m_ColorA[ix] = m_FlashIndicators[ix].material.color;
            m_ColorB[ix] = new Color(r, g, b);
        }
    }

    private void Update()
    {
        for (int i = 0; i < m_FlashIndicators.Length; i++)
        {
            m_FlashIndicators[i].material.color = Color.Lerp(m_ColorA[i], m_ColorB[i], Mathf.PingPong(Time.time, time));
        }
    }
}
