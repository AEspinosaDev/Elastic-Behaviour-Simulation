using UnityEngine;
using System.Collections.Generic;
using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;
using UnityEditor;

///<author>
///Antonio Espinosa Garcia
///2022
///

/// <summary>
/// Elastic solid physics manager. Add this to a Scene Object with a mesh and let it flow with the wind
/// </summary>

[RequireComponent(typeof(Parser))]
public class DeformableSolidBehaviour : MonoBehaviour
{
    #region OtherVariables

    [HideInInspector] Parser m_Parser;

    [HideInInspector] Mesh m_Mesh;

    [HideInInspector] List<Tetrahedron> m_Tetras;

    [HideInInspector] int m_NodesCount;
    [HideInInspector] int m_TetrasCount;
    [HideInInspector] int m_VertexCount;



    [HideInInspector] private List<VertexInfo> m_VerticesInfo;

    [HideInInspector] private Vector3[] m_Vertices;


    [HideInInspector] private List<Node> m_Nodes;

    [HideInInspector] private List<Spring> m_Springs;

    [HideInInspector] private List<Node> m_FixedNodes;

    [HideInInspector] private WindZone[] m_WindObjs;

    [HideInInspector] private Vector3 m_AverageWindVelocity;

    [HideInInspector] private float m_SubTimeStep;

    [HideInInspector] public float m_NodeMass;

    [HideInInspector] public bool m_Ready = false;


    #endregion 


    #region InEditorVariables

    [Tooltip("The difference between the solvers lays on the precission they have calculating each vertex future position.")]
    [SerializeField] public Solver m_SolvingMethod;

    [Tooltip("Less time means more precission. On high res meshes is recommended to lower the timestep.")]
    [SerializeField] [Range(0.001f, 0.02f)] private float m_TimeStep;

    [Tooltip("It divides the total timestep into the number of substeps. More means more precission, but higher computational load.")]
    [SerializeField] [Range(1, 20)] private int m_Substeps;

    [Tooltip("Press P to pause/resume.")]
    [SerializeField] public bool m_Paused;

    [SerializeField] public Vector3 m_Gravity;

    [Tooltip("Controls the mass of the entire mesh, assuming it will be equally divided into each node.")]
    [SerializeField] [Range(0, 50)] public float m_MeshMass;

    [Tooltip("Higher values means more reduction in vertex movement.")]
    [SerializeField] [Range(0f, 5f)] public float m_NodeDamping;

    [Tooltip("Higher values means more reduction in spring contraction forces.")]
    [SerializeField] [Range(0f, 5f)] public float m_SpringDamping;

    [Tooltip("Controls the stiffness of the traction springs.These springs control horizontal and vertical movement. The more stiff, the less the mesh will deform and the quicker it will return to its initial state.")]
    [SerializeField] public float m_TractionStiffness;

    [Tooltip("Controls the stiffness of the flexion springs.These springs control shearing and diagonal movement.The more stiff, the less the mesh will deform and the quicker it will return to initial state.")]
    [SerializeField] public float m_FlexionStiffness;

    //------Managed by custom editor class-----//

    [HideInInspector] public bool m_AffectedByWind;

    [HideInInspector] public bool m_FixingByTexture;

    [HideInInspector] public bool m_CanCollide;
    #endregion

    #region ConditionalInEditorVariables
    //------Toggled by custom editor class only if enabled-----//

    [HideInInspector] [Range(0, 1)] public float m_WindFriction;
    [HideInInspector] public WindPrecission m_WindSolverPrecission;

    [HideInInspector] public Texture2D m_Texture;
    public List<GameObject> m_Fixers;

    [HideInInspector] public List<GameObject> m_CollidingMeshes;

    [HideInInspector] public float m_PenaltyStiffness;
    [HideInInspector] public float m_CollisionOffsetDistance;
    #endregion

    public DeformableSolidBehaviour()
    {


        m_FixingByTexture = false;

        m_TimeStep = 0.004f;
        m_Substeps = 5;

        m_Gravity = new Vector3(0.0f, -9.81f, 0.0f);

        m_Paused = true;

        m_TractionStiffness = 20f;
        m_FlexionStiffness = 15f;

        m_MeshMass = 3.63f;

        m_NodeDamping = 0.3f;
        m_SpringDamping = 0.3f;

        m_SolvingMethod = Solver.Simplectic;

        m_FixedNodes = new List<Node>();

        m_AverageWindVelocity = Vector3.zero;

        m_AffectedByWind = false;
        m_WindFriction = 0.5f;
        m_WindSolverPrecission = WindPrecission.High;

        m_CanCollide = false;
        m_PenaltyStiffness = 10f;
        m_CollisionOffsetDistance = 0.3f;

        m_Ready = true;
    }
    public enum Solver
    {
        Explicit = 0,
        Simplectic = 1,
        Midpoint = 2,
        SimplecticWithImplicitCollisions = 3,
    };
    public enum WindPrecission
    {
        High = 1,
        Medium = 2,
        Low = 3,
    }

    #region Initialization Setups
    [ContextMenu("Low Res Mesh Setup")]
    private void LowResSetup()
    {
        m_TimeStep = 0.02f; m_Paused = true;
        m_TractionStiffness = 20f; m_FlexionStiffness = 15f; m_MeshMass = 3.6f; m_NodeDamping = 0.3f; m_SpringDamping = 0.3f;
        m_SolvingMethod = Solver.Simplectic;
        m_WindSolverPrecission = WindPrecission.High;
    }
    [ContextMenu("Medium Res Mesh Setup")]
    private void MedResSetup()
    {
        m_TimeStep = 0.01f; m_Substeps = 2; m_Paused = true;
        m_TractionStiffness = 50f; m_FlexionStiffness = 30f; m_MeshMass = 20f; m_NodeDamping = 0.3f; m_SpringDamping = 0.3f;
        m_SolvingMethod = Solver.Simplectic;
        m_WindSolverPrecission = WindPrecission.Medium;
    }
    [ContextMenu("High Res Mesh Setup")]
    private void HighResSetup()
    {
        m_TimeStep = 0.007f; m_Substeps = 1; m_Paused = true;
        m_TractionStiffness = 100f; m_FlexionStiffness = 80f; m_MeshMass = 50f; m_NodeDamping = 0.3f; m_SpringDamping = 0.3f;
        m_SolvingMethod = Solver.Simplectic;
        m_WindSolverPrecission = WindPrecission.Low;
    }
    #endregion

    #region MonoBehaviour

    public void Start()
    {

        m_Parser = GetComponent<Parser>();

        m_Nodes = new List<Node>();
        m_Tetras = new List<Tetrahedron>();
        m_VerticesInfo = new List<VertexInfo>();

        m_Parser.CompleteParse(m_Nodes, m_Tetras, this);

        m_NodesCount = m_Nodes.Count;
        m_TetrasCount = m_Tetras.Count;


        m_Mesh = GetComponent<MeshFilter>().mesh;

        m_VertexCount = m_Mesh.vertexCount;
        m_Vertices = m_Mesh.vertices;
        //foreach (var item in m_Vertices)
        //{
        //    print(item);
        //}

        CheckContainingTetraPerVertex();

        m_Springs = new List<Spring>();

        EdgeQualityComparer edgeComparer = new EdgeQualityComparer();

        Dictionary<Edge, Edge> edgeDictionary = new Dictionary<Edge, Edge>(edgeComparer);

        Edge repeatedEdge;
        for (int i = 0; i < m_TetrasCount; i++)
        {
            List<Edge> edges = new List<Edge>();
            edges.Add(new Edge(m_Tetras[i].m_A.m_Id, m_Tetras[i].m_B.m_Id, 0));
            edges.Add(new Edge(m_Tetras[i].m_B.m_Id, m_Tetras[i].m_C.m_Id, 0));
            edges.Add(new Edge(m_Tetras[i].m_C.m_Id, m_Tetras[i].m_A.m_Id, 0));
            edges.Add(new Edge(m_Tetras[i].m_D.m_Id, m_Tetras[i].m_A.m_Id, 0));
            edges.Add(new Edge(m_Tetras[i].m_D.m_Id, m_Tetras[i].m_B.m_Id, 0));
            edges.Add(new Edge(m_Tetras[i].m_D.m_Id, m_Tetras[i].m_C.m_Id, 0));

            foreach (var edge in edges)
            {
                if (!edgeDictionary.TryGetValue(edge, out repeatedEdge))
                {
                    m_Springs.Add(new Spring(m_Nodes[edge.m_A], m_Nodes[edge.m_B], this, true));
                    edgeDictionary.Add(edge, edge);
                }
            }
        }

        //Debug
        //print(edgeDictionary.Count);
        //foreach (var e in edgeDictionary)
        //{
        //    print(e.Value.m_A + " " + e.Value.m_B);
        //}
        //



        m_NodeMass = m_MeshMass / m_NodesCount;

        m_SubTimeStep = m_TimeStep / m_Substeps;





        //Attach to fixers
        if (m_FixingByTexture)
            CheckTextureWeights();
        else
            CheckFixers();

        //Look for Wind objs
        //CheckWindObjects();
    }
    public void OnDrawGizmos()
    {
        //if (m_Ready)
        //{
        foreach (var n in m_Nodes)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(n.m_Pos, 0.1f);
        }
        foreach (var s in m_Springs)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(s.m_NodeA.m_Pos, s.m_NodeB.m_Pos);
        }
        int i = 0;
        //foreach (var v in m_Mesh.vertices)
        //{
        //    Gizmos.color = Color.red;
        //    //Gizmos.DrawIcon(v, i.ToString());
        //    Handles.Label(v+new Vector3(0,i*0.1f,0), i.ToString());
        //    i++;
        //}
        Vector3 normal1 = Vector3.Cross(m_Tetras[0].m_B.m_Pos - m_Tetras[0].m_A.m_Pos, m_Tetras[0].m_C.m_Pos - m_Tetras[0].m_A.m_Pos);
        Vector3 normal2 = Vector3.Cross( m_Tetras[0].m_C.m_Pos -  m_Tetras[0].m_A.m_Pos,  m_Tetras[0].m_D.m_Pos -  m_Tetras[0].m_A.m_Pos);
        Vector3 normal3 = Vector3.Cross(   m_Tetras[0].m_D.m_Pos -  m_Tetras[0].m_A.m_Pos, m_Tetras[0].m_B.m_Pos - m_Tetras[0].m_A.m_Pos);
        Vector3 normal4 = Vector3.Cross(m_Tetras[0].m_D.m_Pos - m_Tetras[0].m_B.m_Pos, m_Tetras[0].m_C.m_Pos - m_Tetras[0].m_B.m_Pos);

        Gizmos.color = Color.red;
        Gizmos.DrawLine(m_Tetras[0].m_A.m_Pos, m_Tetras[0].m_A.m_Pos - normal1 );
        Gizmos.DrawLine(m_Tetras[0].m_A.m_Pos, m_Tetras[0].m_A.m_Pos - normal2 );
        Gizmos.DrawLine(m_Tetras[0].m_A.m_Pos, m_Tetras[0].m_A.m_Pos - normal3 );
        Gizmos.DrawLine(m_Tetras[0].m_D.m_Pos, m_Tetras[0].m_D.m_Pos - normal4 );




        //}

    }
    //private void OnDrawGizmosSelected()
    //{

    //    foreach (var n in m_Nodes)
    //    {
    //        Gizmos.color = Color.green;
    //        //Gizmos.DrawSphere(transform.TransformPoint(n.m_Pos), 0.2f);
    //        Gizmos.DrawSphere(n.m_Pos, 0.2f);
    //    }
    //    foreach (var s in m_Springs)
    //    {
    //        Gizmos.color = Color.green;
    //        //Gizmos.DrawLine(transform.TransformPoint(s.m_NodeA.m_Pos), transform.TransformPoint(s.m_NodeB.m_Pos));
    //        Gizmos.DrawLine(s.m_NodeA.m_Pos, s.m_NodeB.m_Pos);
    //    }


    //}

    public void Update()
    {
        if (Input.GetKeyUp(KeyCode.P))
            this.m_Paused = !this.m_Paused;

        m_NodeMass = m_MeshMass / m_NodesCount;

        m_SubTimeStep = m_TimeStep / m_Substeps;

        CheckWindObjects();
        if (m_AffectedByWind && m_WindSolverPrecission == WindPrecission.Low)
        {
            ComputeWindForces();
        }

        foreach (var node in m_FixedNodes)
        {
            if (m_FixingByTexture)
            {
                node.m_Pos = transform.TransformPoint(node.m_offset);
            }
            else
            {
                node.m_Pos = node.m_Fixer.transform.TransformPoint(node.m_offset);

            }
        }

        for (int i = 0; i < m_VertexCount; i++)
        {


            Vector3 newPos = m_VerticesInfo[i].w_A * m_Tetras[m_VerticesInfo[i].tetra_id].m_A.m_Pos +
                m_VerticesInfo[i].w_B * m_Tetras[m_VerticesInfo[i].tetra_id].m_B.m_Pos +
                m_VerticesInfo[i].w_C * m_Tetras[m_VerticesInfo[i].tetra_id].m_C.m_Pos +
                m_VerticesInfo[i].w_D * m_Tetras[m_VerticesInfo[i].tetra_id].m_D.m_Pos;

            //print("antes"+newPos);
            newPos = transform.InverseTransformPoint(newPos);
            //print(newPos);

            m_Vertices[i] = newPos;

        }
        m_Mesh.vertices = m_Vertices;

        m_Mesh.RecalculateNormals();
        m_Mesh.RecalculateTangents();


    }

    public void FixedUpdate()
    {
        if (m_Paused)
            return; // Not simulating

        if (m_AffectedByWind && m_WindSolverPrecission == WindPrecission.Medium)
        {
            ComputeWindForces();
        }

        // Select integration method
        for (int i = 0; i < m_Substeps; i++)
        {
            if (m_AffectedByWind && m_WindSolverPrecission == WindPrecission.High)
            {
                ComputeWindForces();
            }

            switch (m_SolvingMethod)
            {

                case Solver.Explicit: StepExplicit(); break;

                case Solver.Simplectic: StepSimplectic(); break;

                case Solver.Midpoint: StepRK2(); break;

                case Solver.SimplecticWithImplicitCollisions: StepSimplecticWithImplicitCollision(); break;

                default:
                    throw new System.Exception("[ERROR] Should never happen!");

            }
        }

    }

    #endregion

    #region PhysicsSolvers
    /// <summary>
    /// Worst solver. Good for simple assets or arcade physics.
    /// </summary>
    private void StepExplicit()
    {
        foreach (var n in m_Nodes)
        {
            n.m_Force = Vector3.zero;
            if (!m_AffectedByWind) n.m_WindForce = Vector3.zero;
            n.ComputeForces();
        }

        foreach (var s in m_Springs)
        {
            s.ComputeForces();
        }

        foreach (var n in m_Nodes)
        {
            if (!n.m_Fixed)
            {
                if (m_CanCollide)
                {
                    foreach (var obj in m_CollidingMeshes)
                    {
                        int condition = n.isColliding(obj, m_CollisionOffsetDistance);
                        if (condition > 0)
                        {
                            n.ComputeExplicitPenaltyForce(n.ComputeCollision(obj, condition), m_PenaltyStiffness);
                            n.m_Force += n.m_PenaltyForce;
                        }
                    }
                }
                n.m_Pos += m_SubTimeStep * n.m_Vel;
                n.m_Vel += m_SubTimeStep / m_NodeMass * n.m_Force;
            }
        }

    }
    /// <summary>
    /// Better solver. Either not perfect. Recommended.
    /// </summary>
    private void StepSimplectic()
    {

        foreach (var n in m_Nodes)
        {
            n.m_Force = Vector3.zero;
            if (!m_AffectedByWind) n.m_WindForce = Vector3.zero;
            n.ComputeForces();
        }

        foreach (var s in m_Springs)
        {
            s.ComputeForces();
        }

        foreach (var n in m_Nodes)
        {
            if (!n.m_Fixed)
            {
                Vector3 resVel = n.m_Vel + m_SubTimeStep / m_NodeMass * n.m_Force;
                if (m_CanCollide)
                {
                    foreach (var obj in m_CollidingMeshes)
                    {
                        int condition = n.isColliding(obj, m_CollisionOffsetDistance);
                        if (condition > 0)
                        {
                            n.ComputeExplicitPenaltyForce(n.ComputeCollision(obj, condition), m_PenaltyStiffness);
                            n.m_Force += n.m_PenaltyForce;
                            resVel = n.m_Vel + m_SubTimeStep / m_NodeMass * n.m_Force;
                        }
                    }
                }
                n.m_Vel = resVel;
                n.m_Pos += m_SubTimeStep * n.m_Vel;
            }

        }
    }

    /// <summary>
    /// Fairly good solver. Also known as midpoint.
    /// </summary>
    private void StepRK2()
    {
        Vector3[] m_Vel0 = new Vector3[m_Nodes.Count];
        Vector3[] m_Pos0 = new Vector3[m_Nodes.Count];

        //Midpoint
        for (int i = 0; i < m_Nodes.Count; i++)
        {

            m_Vel0[i] = m_Nodes[i].m_Vel;
            m_Pos0[i] = m_Nodes[i].m_Pos;


            m_Nodes[i].m_Force = Vector3.zero;

            if (!m_AffectedByWind) m_Nodes[i].m_WindForce = Vector3.zero;
            m_Nodes[i].ComputeForces();
        }

        foreach (var s in m_Springs)
        {
            s.ComputeForces();
        }

        foreach (var n in m_Nodes)
        {
            if (!n.m_Fixed)
            {
                Vector3 resVel = n.m_Vel + (m_SubTimeStep * 0.5f) / m_NodeMass * n.m_Force;
                if (m_CanCollide)
                {
                    foreach (var obj in m_CollidingMeshes)
                    {
                        int condition = n.isColliding(obj, m_CollisionOffsetDistance);
                        if (condition > 0)
                        {
                            n.ComputeExplicitPenaltyForce(n.ComputeCollision(obj, condition), m_PenaltyStiffness);
                            n.m_Force += n.m_PenaltyForce;
                            resVel = n.m_Vel + (m_SubTimeStep * 0.5f) / m_NodeMass * n.m_Force;
                        }
                    }
                }
                n.m_Vel += resVel;
                n.m_Pos += (m_SubTimeStep * 0.5f) * n.m_Vel;
            }
        }

        //EndPoint
        foreach (var n in m_Nodes)
        {
            n.m_Force = Vector3.zero;

            n.ComputeForces();
        }

        foreach (var s in m_Springs)
        {
            s.ComputeForces();
        }

        for (int i = 0; i < m_Nodes.Count; i++)
        {

            if (!m_Nodes[i].m_Fixed)
            {
                Vector3 resVel = m_Vel0[i] + m_SubTimeStep / m_NodeMass * m_Nodes[i].m_Force;
                if (m_CanCollide)
                {
                    foreach (var obj in m_CollidingMeshes)
                    {
                        int condition = m_Nodes[i].isColliding(obj, m_CollisionOffsetDistance);
                        if (condition > 0)
                        {
                            m_Nodes[i].ComputeExplicitPenaltyForce(m_Nodes[i].ComputeCollision(obj, condition), m_PenaltyStiffness);
                            m_Nodes[i].m_Force += m_Nodes[i].m_PenaltyForce;
                            resVel = m_Vel0[i] + m_SubTimeStep / m_NodeMass * m_Nodes[i].m_Force;
                        }
                    }
                }
                m_Nodes[i].m_Vel = resVel;
                m_Nodes[i].m_Pos = m_Pos0[i] + m_SubTimeStep * m_Nodes[i].m_Vel;
            }
        }


    }
    /// <summary>
    /// Simplectic solver using implicit aproximation for collisions. The best solver.
    /// </summary>
    private void StepSimplecticWithImplicitCollision()
    {
        foreach (var n in m_Nodes)
        {
            n.m_Force = Vector3.zero;
            if (!m_AffectedByWind) n.m_WindForce = Vector3.zero;
            n.ComputeForces();
        }

        foreach (var s in m_Springs)
        {
            s.ComputeForces();
        }

        Vector3 resVel;
        foreach (var n in m_Nodes)
        {
            if (!n.m_Fixed)
            {
                resVel = n.m_Vel + m_SubTimeStep / m_NodeMass * n.m_Force;
                if (m_CanCollide)
                {
                    foreach (var obj in m_CollidingMeshes)
                    {
                        int condition = n.isColliding(obj, m_CollisionOffsetDistance);
                        if (condition > 0)
                        {
                            MatrixXD diff = n.ComputeImplicitPenaltyForce(n.ComputeCollision(obj, condition), m_PenaltyStiffness);

                            MatrixXD i = new DenseMatrixXD(3);
                            i = DenseMatrixXD.CreateIdentity(3);

                            Vector3 b = n.m_Vel + m_SubTimeStep / m_NodeMass * (n.m_PenaltyForce + n.m_Force); //Spring and wind force already computed in n.m_Force

                            VectorXD bProxy = new DenseVectorXD(3);
                            bProxy[0] = b.x; bProxy[1] = b.y; bProxy[2] = b.z;

                            var x = (i - (m_SubTimeStep * m_SubTimeStep / m_NodeMass) * diff).Solve(bProxy);

                            resVel = new Vector3((float)x[0], (float)x[1], (float)x[2]);

                        }
                    }

                }

                n.m_Vel = resVel;
                n.m_Pos += m_SubTimeStep * n.m_Vel;
            }

        }
    }


    #endregion
    /// <summary>
    /// Iterates through all mesh vertices and checks what is the tetrahedron in which each vertex is contained, assigning the weights necesary to compute the position
    /// in the physic solving.
    /// </summary>
    private void CheckContainingTetraPerVertex()
    {
        //Check if node is inside
        for (int i = 0; i < m_VertexCount; i++)
        {
            Vector3 globalPos = transform.TransformPoint(m_Mesh.vertices[i]);
            //print(globalPos);

            foreach (var tetra in m_Tetras)
            {
                if (tetra.PointInside(globalPos))
                {
                    tetra.ComputeVertexWeights(globalPos, out float wA, out float wB, out float wC, out float wD);
                    print(i + " = " + wA+" "+wB+" "+wC+" "+wD);
                    m_VerticesInfo.Add(new VertexInfo(i, tetra.id, wA, wB, wC, wD));
                    print("SI, ESTA DENTRO" + i);
                    break;
                }

            }

        }

    }
    /// <summary>
    /// Checks whether the vertex is inside the fixer colliders in order to put it in a fixed state.
    /// </summary>
    private void CheckFixers()
    {
        foreach (var node in m_Nodes)
        {
            foreach (var obj in m_Fixers)
            {
                Collider collider = obj.GetComponent<Collider>();
                Vector3 n_pos = node.m_Pos;

                if (collider.bounds.Contains(n_pos))
                {
                    node.m_Fixed = true;
                    if (node.m_Fixer != null) Debug.LogWarning("[Warning] More than one fixer assinged to the vertex. It will only be accepted one");
                    node.m_Fixer = obj;
                    node.m_offset = node.m_Fixer.transform.InverseTransformPoint(node.m_Pos);
                    m_FixedNodes.Add(node);

                }
            }
        }
    }
    /// <summary>
    /// Check whether the vertex color value is inside the fixing color condition in order to put it in a fixed state.
    /// </summary>
    private void CheckTextureWeights()
    {
        if (m_Texture != null)
        {
            int textWidth = m_Texture.width;

            foreach (var node in m_Nodes)
            {
                float xCoord = textWidth * node.m_UV.x;
                float yCoord = textWidth * node.m_UV.y;

                Color color = m_Texture.GetPixel((int)xCoord, (int)yCoord);

                float factor = 0.9f;
                if (color.a >= factor)
                {
                    node.m_Fixed = true;
                    m_FixedNodes.Add(node);
                    node.m_offset = transform.InverseTransformPoint(node.m_Pos);
                }
                else
                {
                    node.m_ForceFactor = 1 - color.a;
                }
            }
        }

    }
    /// <summary>
    /// Automatically called on start. Checks for wind objects in order to take them into account to make the wind simulation.
    /// </summary>
    private void CheckWindObjects()
    {
        if (m_AffectedByWind) m_WindObjs = FindObjectsOfType<WindZone>();
        ComputeWindSpeed();
    }
    /// <summary>
    /// Called every update. Computes the average velocity vector of the resulting wind.
    /// </summary>
    private void ComputeWindSpeed()
    {
        m_AverageWindVelocity = Vector3.zero;

        if (m_AffectedByWind)
        {
            int total = m_WindObjs.Length;
            foreach (var obj in m_WindObjs)
            {
                if (obj.gameObject.activeSelf)
                {
                    //Takes in account all wind object parameters to simulate wind variation
                    Vector3 windVel = obj.transform.forward * (obj.windMain + (obj.windPulseMagnitude * obj.windMain * Mathf.Abs(Mathf.Sin(Time.time * (obj.windPulseFrequency * 10)))));
                    m_AverageWindVelocity += windVel;
                }

            }
            if (m_AverageWindVelocity.magnitude > 0)
                m_AverageWindVelocity = new Vector3(m_AverageWindVelocity.x / total, m_AverageWindVelocity.y / total, m_AverageWindVelocity.z / total);
        }
    }
    /// <summary>
    /// Called every wind solving iteration. Computes the applied wind force of every triangle in the mesh. 
    /// </summary>
    private void ComputeWindForces()
    {
        //    for (int i = 0; i < m_ProxyMesh.triangles.Length; i += 3)
        //    {

        //        Node nodeA = m_Nodes[m_ProxyTriangles[i]];
        //        Node nodeB = m_Nodes[m_ProxyTriangles[i + 1]];
        //        Node nodeC = m_Nodes[m_ProxyTriangles[i + 2]];

        //        Vector3 crossProduct = Vector3.Cross(nodeA.m_Pos - nodeB.m_Pos, nodeB.m_Pos - nodeC.m_Pos);

        //        float trisArea = crossProduct.magnitude / 2;

        //        Vector3 trisNormal = crossProduct.normalized;

        //        Vector3 trisSpeed = new Vector3((nodeA.m_Vel.x + nodeB.m_Vel.x + nodeC.m_Vel.x) / 3, (nodeA.m_Vel.y + nodeB.m_Vel.y + nodeC.m_Vel.y) / 3, (nodeA.m_Vel.z + nodeB.m_Vel.z + nodeC.m_Vel.z) / 3);

        //        Vector3 trisWindForce = m_WindFriction * trisArea * Vector3.Dot(trisNormal, m_AverageWindVelocity - trisSpeed) * trisNormal;
        //        Vector3 dispersedForce = trisWindForce / 3;

        //        nodeA.m_WindForce = dispersedForce;
        //        nodeB.m_WindForce = dispersedForce;
        //        nodeC.m_WindForce = dispersedForce;
        //    }
    }
}

