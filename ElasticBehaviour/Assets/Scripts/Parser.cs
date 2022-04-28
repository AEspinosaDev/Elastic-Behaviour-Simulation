using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

/// <summary>
/// Parser class useful for reading the TetGen mesh generated files
/// </summary>
public class Parser : MonoBehaviour
{
    private string[] m_NodesRaw;
    private string[] m_TetrasRaw;


    [Tooltip("Insert the .node file generated from TetGen here")]
    [SerializeField] public TextAsset m_NodeFile;
    
    [Tooltip("Insert the .ele file generated from TetGen here")]
    [SerializeField] public TextAsset m_TetraFile;


    void Awake()
    {
        //CultureInfo locale = new CultureInfo("en-US");

        // Ayuda TextAsset: https://docs.unity3d.com/ScriptReference/TextAsset.html
        // Ayuda Unity de String https://docs.unity3d.com/ScriptReference/String.html
        // Ayuda MSDN de String https://docs.microsoft.com/en-us/dotnet/api/system.string?redirectedfrom=MSDN&view=netframework-4.8
        // Ayuda MSDN de String.Split https://docs.microsoft.com/en-us/dotnet/api/system.string.split?view=netframework-4.8

        m_NodesRaw = m_NodeFile.text.Split(new string[] { " ", "\n", "\r" }, StringSplitOptions.RemoveEmptyEntries);

        m_TetrasRaw = m_TetraFile.text.Split(new string[] { " ", "\n", "\r" }, StringSplitOptions.RemoveEmptyEntries);


    }
    /// <summary>
    /// Parse the .node file and return a List with the positions of each one of them
    /// </summary>
    /// <returns></returns>
    public Vector3[] parseNodes()
    {
        int numNodes = int.Parse(m_NodesRaw[0]);
        Vector3[] nodeList = new Vector3[numNodes];

        int idx = 5;
        for (int i = 0; i < numNodes; i++)
        {
            nodeList[i]= new Vector3( float.Parse(m_NodesRaw[idx]), float.Parse(m_NodesRaw[idx+1]), float.Parse(m_NodesRaw[idx+2]));
            idx += 4;
        }

        return nodeList;
    }
    /// <summary>
    /// Parse the .ele file and return a List with the nodes indexes each tetra is composed of. TetGen first element is equal to 1, but this parser change its value to 0 in order 
    /// to be better to work by code.
    /// </summary>
    /// <returns></returns>
    public int[,] parseTetras()
    {
        int numTetras = int.Parse(m_TetrasRaw[0]);
       int[,] tetraList = new int[numTetras,4];

        int idx = 4;
        for (int i = 0; i < numTetras; i++)
        {
            tetraList[i, 0] = int.Parse(m_TetrasRaw[idx]) - 1;
            tetraList[i, 1] = int.Parse(m_TetrasRaw[idx+1]) - 1;
            tetraList[i, 2] = int.Parse(m_TetrasRaw[idx+2]) - 1;
            tetraList[i, 3] = int.Parse(m_TetrasRaw[idx+3]) - 1;

            idx += 5;
        }
        print(numTetras);

        return tetraList;
    }
    /// <summary>
    /// Optimized version of the parser function. This function will set all the data structures necessary for the deforming component ready.
    /// </summary>
    /// <param name="nodesList"></param>
    /// <param name="tetrasList"></param>
    /// <param name="manager"></param>
    public void completeParse(List<Node> nodesList, List<Tetrahedron> tetrasList, DeformableSolidBehaviour manager)
    {
        
        int numNodes = int.Parse(m_NodesRaw[0]);

        int idx = 5;
        Vector3 nodePos;
        for (int i = 0; i < numNodes; i++)
        {
            nodePos = new Vector3(float.Parse(m_NodesRaw[idx]), float.Parse(m_NodesRaw[idx + 1]), float.Parse(m_NodesRaw[idx + 2]));
            nodePos= transform.TransformPoint(nodePos);
            nodesList.Add(new Node(i, new Vector3(nodePos.x, nodePos.y, nodePos.z), manager));
            idx += 4;
        }

        int numTetras = int.Parse(m_TetrasRaw[0]);
        idx = 4;
        for (int i = 0; i < numTetras; i++)
        {
            tetrasList.Add(new Tetrahedron(i,
                nodesList[int.Parse(m_TetrasRaw[idx]) - 1],
                nodesList[int.Parse(m_TetrasRaw[idx + 1]) - 1],
                nodesList[int.Parse(m_TetrasRaw[idx + 2]) - 1],
                nodesList[int.Parse(m_TetrasRaw[idx + 3]) - 1]));

            idx += 5;
        }


    }

}
